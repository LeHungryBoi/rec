/**
 * rec.c - 433MHz EV1527 RF Decoder for RP2040 (HID Edition)
 *
 * Ported from AVR EV1527 decoder (github.com/aKaReZa75/AVR_ev1527)
 * Uses PIO state machine for hardware-accurate pulse width measurement
 * EV1527 protocol: 24-bit frame = 20-bit Address + 4-bit Key
 *
 * GPIO 18 -> 433MHz receiver DATA pin
 * Output via USB HID (Raw TinyUSB, no CDC/stdio)
 *
 * HID 通信协议:
 *   输出报告 ID 1 (64 bytes): 主机 → 设备, 携带文本命令
 *   输入报告 ID 2 (64 bytes): 设备 → 主机, 携带文本响应/数据
 *
 *   多包传输: 响应超过 61 字节时自动分包, 由主机端重组。
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "pico/binary_info.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/multicore.h"

/* TinyUSB HID */
#include "tusb.h"
#include "usb_descriptors.h"

/* PIO 程序头文件 (由 pioasm 从 radio_rx.pio 生成) */
#include "radio_rx.pio.h"

/* Core 1: EV1527 发射模拟器 */
#include "ev1527_tx.h"

/* ================================================================
 * 硬件配置
 * ================================================================ */

#define RADIO_GPIO_PIN  18
#define LED_PIN         25
#define RF_PIO          pio0
#define RF_PIO_SM       0

/* ================================================================
 * 脉冲环形缓冲区
 * ================================================================ */

#define PULSE_RING_SIZE 256

typedef struct {
    uint32_t data[PULSE_RING_SIZE];
    volatile uint32_t head;
    uint32_t tail;
} pulse_ring_t;

static pulse_ring_t pulse_ring;

static inline bool ring_empty(pulse_ring_t *r) {
    return r->head == r->tail;
}

static inline bool ring_push(pulse_ring_t *r, uint32_t val) {
    uint32_t next = (r->head + 1) & (PULSE_RING_SIZE - 1);
    if (next == r->tail) return false;
    r->data[r->head] = val;
    r->head = next;
    return true;
}

static inline bool ring_pop(pulse_ring_t *r, uint32_t *val) {
    if (ring_empty(r)) return false;
    *val = r->data[r->tail];
    r->tail = (r->tail + 1) & (PULSE_RING_SIZE - 1);
    return true;
}

/* ================================================================
 * PIO 中断处理
 * ================================================================ */

static void __isr __time_critical_func(pio_irq_handler)(void) {
    while (!pio_sm_is_rx_fifo_empty(RF_PIO, RF_PIO_SM)) {
        uint32_t val = pio_sm_get(RF_PIO, RF_PIO_SM);
        ring_push(&pulse_ring, val);
    }
}

/* ================================================================
 * 脉冲解析
 * ================================================================ */

#define PULSE_LEVEL_MASK    0x80000000u
#define PULSE_WIDTH_MASK    0x7FFFFFFFu
#define PIO_CYCLE_NS        8
#define TICK_TO_US(t)       (((t) * 2 * PIO_CYCLE_NS) / 1000)

typedef struct {
    bool     is_high;
    uint32_t ticks;
    uint32_t width_us;
} pulse_t;

static bool parse_pulse(uint32_t raw, pulse_t *p) {
    p->is_high  = (raw & PULSE_LEVEL_MASK) != 0;
    p->ticks    = (raw & PULSE_WIDTH_MASK) * 2;
    p->width_us = TICK_TO_US(raw & PULSE_WIDTH_MASK);
    return p->width_us > 0;
}

/* ================================================================
 * EV1527 协议解码器
 * ================================================================ */

#define EV1527_FRAME_BITS   24
#define EV1527_MAX_INDEX    (EV1527_FRAME_BITS - 1)

#define EV_MIN_PULSE_US     150
#define EV_MAX_PULSE_US     4500

static inline bool ev1527_check_preamble(uint32_t low_us, uint32_t high_us) {
    return (low_us >= 25 * high_us) && (low_us <= 40 * high_us);
}

static inline int ev1527_decode_bit(uint32_t high_us, uint32_t low_us) {
    return (high_us >= (low_us + (low_us >> 1))) ? 1 : 0;
}

typedef union {
    uint32_t rawValue;
    struct {
        uint32_t Address : 20;
        uint32_t Keys    : 4;
        uint32_t Detect  : 1;
        uint32_t Reserve : 7;
    } Bits;
} ev1527_data_t;

typedef enum {
    STATE_FIRST_TRIGGER,
    STATE_WAIT_PREAMBLE,
    STATE_DECODE_BITS,
} ev1527_state_t;

typedef struct {
    ev1527_state_t state;
    ev1527_data_t  data;
    uint8_t        bit_index;
    uint32_t       pending_high_us;
    bool           has_pending_high;
    uint32_t       frame_count;
    absolute_time_t last_pulse_time;
} decoder_t;

static decoder_t decoder;

static void decoder_reset(decoder_t *d) {
    d->state = STATE_FIRST_TRIGGER;
    d->data.rawValue = 0;
    d->bit_index = 0;
    d->pending_high_us = 0;
    d->has_pending_high = false;
}

static void decoder_init(decoder_t *d) {
    decoder_reset(d);
    d->frame_count = 0;
    d->last_pulse_time = get_absolute_time();
}

/* ================================================================
 * HID 通信层
 * ================================================================ */

/* 输入报告包结构 (设备 → 主机)
 * [0]       Report ID = HID_REPORT_ID_RESPONSE (0x02)
 * [1]       Flags: bit7=has_more, bit6:0=sequence
 * [2]       本包数据长度 (0~61)
 * [3..63]   数据载荷
 */
#define HID_PKT_FLAG_MORE   0x80
#define HID_PKT_SEQ_MASK    0x7F
#define HID_PKT_DATA_MAX    61
#define HID_PKT_HEADER      3

/* 发送缓冲: 用于 printf 风格输出后刷新到 HID */
#define TX_BUF_SIZE         512
static char tx_buffer[TX_BUF_SIZE];
static int  tx_pos = 0;

/* 命令接收缓冲 */
#define CMD_BUF_SIZE        64
static char cmd_buffer[CMD_BUF_SIZE];

/**
 * 将 tx_buffer 内容通过 HID 输入报告分包发送。
 * 每包携带最多 HID_PKT_DATA_MAX 字节; 多包时 bit7 置位,
 * 最后一包 bit7 清零。
 *
 * 返回 true 表示全部发送成功, false 表示超时/失败。
 */
static bool hid_flush(void) {
    if (tx_pos <= 0) return true;

    int total = tx_pos;
    int offset = 0;
    int seq = 0;
    bool ok = true;

    while (offset < total) {
        /* 等待 HID IN 端点就绪, 最多等 200ms */
        absolute_time_t deadline = make_timeout_time_ms(200);
        while (!tud_hid_n_ready(0)) {
            tud_task();
            if (time_reached(deadline)) {
                /* 主机没有轮询 IN 端点 — 丢弃剩余数据以免卡死主循环 */
                ok = false;
                goto flush_abort;
            }
        }

        uint8_t report[REPORT_PACKET_SIZE];
        memset(report, 0, REPORT_PACKET_SIZE);
        report[0] = HID_REPORT_ID_RESPONSE;

        int chunk = total - offset;
        if (chunk > HID_PKT_DATA_MAX) chunk = HID_PKT_DATA_MAX;

        report[1] = (uint8_t)(seq & HID_PKT_SEQ_MASK);
        if (offset + chunk < total) {
            report[1] |= HID_PKT_FLAG_MORE;  /* 还有后续包 */
        }
        report[2] = (uint8_t)chunk;
        memcpy(&report[3], tx_buffer + offset, chunk);

        if (!tud_hid_n_report(0, HID_REPORT_ID_RESPONSE, report, REPORT_PACKET_SIZE)) {
            ok = false;
            goto flush_abort;
        }

        offset += chunk;
        seq++;
    }

flush_abort:
    tx_pos = 0;
    return ok;
}

/**
 * HID printf — 格式化字符串到 HID 发送缓冲。
 * 仅追加, 不会自动刷新。调用 hid_flush() 发送。
 */
static int hid_printf(const char *fmt, ...) {
    if (tx_pos >= TX_BUF_SIZE - 128) {
        hid_flush(); /* 缓冲满则自动刷新 */
    }
    int remain = TX_BUF_SIZE - tx_pos - 1;
    if (remain <= 0) return -1;

    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(tx_buffer + tx_pos, remain, fmt, args);
    va_end(args);

    if (n > 0) tx_pos += n;
    if (n >= remain) tx_pos = TX_BUF_SIZE - 1; /* 截断保护 */

    return n;
}

/* 标记: 是否有待处理的命令 */
static volatile bool cmd_pending = false;

/* 全局 raw 模式标记 */
static volatile bool raw_mode = false;

/**
 * HID 输出报告回调 (主机 → 设备)
 * 主机通过 Report ID 1 发送命令文本到设备。
 * WebHID 的 sendReport(reportId, data) 会把 reportId 作为单独参数发送，
 * 因此这里收到的 buffer 只包含报告数据本身:
 *   [0] = 命令长度
 *   [1..] = 命令字符串
 */
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize) {
    (void)instance;
    (void)report_type;

    if (report_id != HID_REPORT_ID_COMMAND) return;
    if (bufsize < 1) return;

    /* buffer[0] = 长度, buffer[1..] = 命令 */
    uint8_t len = buffer[0];
    if (len == 0) return;

    if (len > (REPORT_PACKET_SIZE - 1)) {
        len = REPORT_PACKET_SIZE - 1;
    }

    if ((uint16_t)(len + 1) > bufsize) {
        len = (uint8_t)(bufsize - 1);
    }

    memcpy(cmd_buffer, &buffer[1], len);
    cmd_buffer[len] = '\0';
    cmd_pending = true;
}

/**
 * HID 获取报告回调 (GET_REPORT 控制传输)
 * 主机请求报告时回调。本项目使用中断端点发送数据,
 * 不依赖 GET_REPORT, 因此返回 0。
 */
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type,
                               uint8_t *buffer, uint16_t reqlen) {
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    return 0;
}

/**
 * 读取收到的 HID 命令并处理。
 * 在主循环中调用, 非 ISR 上下文, 可安全发送 HID 报告。
 */
static void process_hid_command(const char *cmd) {
    if (cmd[0] == '\0') return;

    /* Visual acknowledgement for any received command. */
    gpio_put(LED_PIN, 1);

    if (strcmp(cmd, "ping") == 0) {
        hid_printf("PICO LIVE\n");
    } else if (strcmp(cmd, "hello") == 0) {
        hid_printf("Hello from EV1527 RF Decoder (RP2040) [HID]\n");
    } else if (strcmp(cmd, "led") == 0) {
        gpio_put(LED_PIN, !gpio_get(LED_PIN));
        hid_printf("LED toggled (%s)\n", gpio_get(LED_PIN) ? "ON" : "OFF");
    } else if (strcmp(cmd, "status") == 0) {
        hid_printf("Status: OK\n");
        hid_printf("  Interface: USB HID\n");
        hid_printf("  GPIO pin: %d\n", RADIO_GPIO_PIN);
        hid_printf("  Frames decoded: %u\n", decoder.frame_count);
        hid_printf("  State: %s\n",
                   decoder.state == STATE_FIRST_TRIGGER  ? "FIRST_TRIGGER" :
                   decoder.state == STATE_WAIT_PREAMBLE  ? "WAIT_PREAMBLE" :
                   decoder.state == STATE_DECODE_BITS    ? "DECODE_BITS" : "UNKNOWN");
        hid_printf("  Last Address: 0x%05lX\n", decoder.data.Bits.Address);
        hid_printf("  Last Key:     0x%X\n", decoder.data.Bits.Keys);
        hid_printf("  Ring buffer: %lu/%u\n",
                   (pulse_ring.head - pulse_ring.tail) & (PULSE_RING_SIZE - 1),
                   PULSE_RING_SIZE);
    } else if (strcmp(cmd, "reset") == 0) {
        decoder_init(&decoder);
        pulse_ring.head = 0;
        pulse_ring.tail = 0;
        tx_pos = 0;
        raw_mode = false;
        hid_printf("Decoder reset\n");
    } else if (strcmp(cmd, "raw") == 0) {
        raw_mode = true;
        hid_printf("[MODE] Switched to RAW pulse output mode\n");
    } else if (strcmp(cmd, "reboot") == 0) {
        hid_printf("Rebooting application...\n");
        hid_flush();
        sleep_ms(100);
        watchdog_reboot(0, 0, 0);
        while (1) {
            tight_loop_contents();
        }
    } else if (strcmp(cmd, "bootsel") == 0) {
        hid_printf("Entering BOOTSEL mode...\n");
        hid_flush();
        sleep_ms(100);
        reset_usb_boot(0, 0);
        while (1) {
            tight_loop_contents();
        }
    } else {
        hid_printf("ECHO: %s\n", cmd);
    }

    hid_flush();
    sleep_ms(50);
    gpio_put(LED_PIN, 0);
}

/* ================================================================
 * 解码帧输出 (通过 HID)
 * ================================================================ */

static void decoder_output_frame(decoder_t *d) {
    d->frame_count++;

    hid_printf("\n========================================\n");
    hid_printf("  FRAME #%u - EV1527 Decoded\n", d->frame_count);
    hid_printf("========================================\n");
    hid_printf("  Raw Value : 0x%08lX\n", d->data.rawValue);
    hid_printf("  Address   : 0x%05lX (%lu)\n", d->data.Bits.Address, d->data.Bits.Address);
    hid_printf("  Key/Button: 0x%X (%lu)\n", d->data.Bits.Keys, d->data.Bits.Keys);
    hid_printf("----------------------------------------\n");

    hid_printf("  Binary: ");
    for (int i = 23; i >= 0; i--) {
        hid_printf("%d", (d->data.rawValue >> i) & 1);
        if (i == 4) hid_printf(" | ");
        else if (i % 4 == 0 && i > 4) hid_printf(" ");
    }
    hid_printf("\n");
    hid_printf("           <--- 20-bit Address ---> <-Key->\n");
    hid_printf("========================================\n\n");

    hid_flush();
}

/* ================================================================
 * 脉冲处理 (解码状态机)
 * ================================================================ */

static void decoder_process_pulse(decoder_t *d, pulse_t *p) {
    if (p->width_us < EV_MIN_PULSE_US || p->width_us > EV_MAX_PULSE_US) {
        decoder_reset(d);
        return;
    }

    d->last_pulse_time = get_absolute_time();

    switch (d->state) {

    case STATE_FIRST_TRIGGER:
        if (p->is_high) {
            d->pending_high_us = p->width_us;
            d->has_pending_high = true;
            d->state = STATE_WAIT_PREAMBLE;
        }
        break;

    case STATE_WAIT_PREAMBLE:
        if (p->is_high) {
            d->pending_high_us = p->width_us;
            d->has_pending_high = true;
        } else if (d->has_pending_high) {
            uint32_t high_us = d->pending_high_us;
            uint32_t low_us  = p->width_us;
            d->has_pending_high = false;

            if (ev1527_check_preamble(low_us, high_us)) {
                d->state = STATE_DECODE_BITS;
                d->data.rawValue = 0;
                d->bit_index = 0;
                hid_printf("[SYNC] Preamble detected: HIGH=%lu us, LOW=%lu us\n", high_us, low_us);
                hid_flush();
            }
        }
        break;

    case STATE_DECODE_BITS:
        if (p->is_high) {
            d->pending_high_us = p->width_us;
            d->has_pending_high = true;
        } else if (d->has_pending_high) {
            uint32_t high_us = d->pending_high_us;
            uint32_t low_us  = p->width_us;
            d->has_pending_high = false;

            if (ev1527_check_preamble(low_us, high_us)) {
                if (d->bit_index > 0) {
                    if (d->bit_index == EV1527_FRAME_BITS) {
                        d->data.Bits.Detect = 1;
                        decoder_output_frame(d);
                    } else {
                        hid_printf("[DROP] Incomplete frame: %u bits\n", d->bit_index);
                        hid_flush();
                    }
                }
                d->state = STATE_DECODE_BITS;
                d->data.rawValue = 0;
                d->bit_index = 0;
                hid_printf("[SYNC] Preamble detected: HIGH=%lu us, LOW=%lu us\n", high_us, low_us);
                hid_flush();
                break;
            }

            uint32_t total = high_us + low_us;
            if (total < EV_MIN_PULSE_US || total > EV_MAX_PULSE_US) {
                hid_printf("[ERR] Invalid pulse: H=%lu L=%lu us\n", high_us, low_us);
                hid_flush();
                decoder_reset(d);
                break;
            }

            int bit = ev1527_decode_bit(high_us, low_us);
            d->data.rawValue = (d->data.rawValue << 1) | bit;
            d->bit_index++;

            if (d->bit_index > EV1527_MAX_INDEX) {
                d->data.Bits.Detect = 1;
                decoder_output_frame(d);
                decoder_reset(d);
            }
        }
        break;
    }
}

/* ================================================================
 * 原始脉冲输出 (兼容模式, 通过 HID)
 * ================================================================ */

#define RAW_BUFFER_SIZE 512
static uint32_t raw_buffer[RAW_BUFFER_SIZE];
static int raw_index = 0;

static void send_raw_data(void) {
    if (raw_index <= 0) return;

    hid_printf("DATA:");
    for (int i = 0; i < raw_index; i++) {
        pulse_t p;
        parse_pulse(raw_buffer[i], &p);
        hid_printf("%lu", p.width_us);
        if (i < raw_index - 1) hid_printf(",");
    }
    hid_printf(",END\n");

    hid_flush();
    raw_index = 0;
}

/* ================================================================
 * 硬件初始化
 * ================================================================ */

static void init_hardware(void) {
    /* 1. TinyUSB — 初始化设备栈 */
    tusb_init();

    /* 2. 板载 LED */
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    /* 3. RF 接收引脚 (下拉) */
    gpio_init(RADIO_GPIO_PIN);
    gpio_set_dir(RADIO_GPIO_PIN, GPIO_IN);
    gpio_pull_down(RADIO_GPIO_PIN);

    /* 4. 加载并启动 PIO 程序 */
    uint offset = pio_add_program(RF_PIO, &radio_rx_program);
    radio_rx_program_init(RF_PIO, RF_PIO_SM, offset, RADIO_GPIO_PIN);

    /* 5. PIO RX FIFO 中断 */
    pio_set_irq0_source_enabled(RF_PIO, pis_sm0_rx_fifo_not_empty, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);

    /* 6. 初始化解码器和脉冲缓冲 */
    decoder_init(&decoder);
    pulse_ring.head = 0;
    pulse_ring.tail = 0;
}

/* ================================================================
 * 主程序
 * ================================================================ */

int main(void) {
    init_hardware();

    /* 在 Core 1 上启动 EV1527 发射模拟器 */
    multicore_launch_core1(ev1527_tx_run);

    /* LED 启动闪烁 — 同步等待 USB 枚举 (最多 3 秒) */
    {
        absolute_time_t usb_timeout = make_timeout_time_ms(3000);
        for (int i = 0; i < 6 && !time_reached(usb_timeout); i++) {
            gpio_put(LED_PIN, 1);
            sleep_ms(150);
            gpio_put(LED_PIN, 0);
            sleep_ms(150);
            tud_task();
        }
    }

    /* 启动横幅 — 等待 HID 接口准备好 */
    {
        absolute_time_t banner_timeout = make_timeout_time_ms(5000);
        while (!tud_hid_n_ready(0) && !time_reached(banner_timeout)) {
            tud_task();
            sleep_ms(10);
        }
    }

    /* 启动横幅 — 仅当 HID IN 端点已就绪时才发送,
     * 否则跳过, 避免 hid_flush() 阻塞主循环。 */
    if (tud_hid_n_ready(0)) {
        hid_printf("\n");
        hid_printf("========================================\n");
        hid_printf("  EV1527 RF Decoder for RP2040 (HID)\n");
        hid_printf("  Ported from AVR EV1527 library\n");
        hid_printf("  GPIO %d @ 125MHz PIO (RX)\n", RADIO_GPIO_PIN);
        hid_printf("  Interface: USB HID (VID=0x%04X PID=0x%04X)\n", USB_VID, USB_PID);
        hid_printf("========================================\n");
        hid_printf("System Clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);
        hid_printf("\n");
        hid_printf("Core 1 TX Simulator:\n");
        hid_printf("  GPIO %d -> EV1527 frames @ T=%d us\n", EVTX_GPIO_PIN, EVTX_T_US);
        hid_printf("  Address: 0x%05lX, Keys: 0..%d\n", (uint32_t)EVTX_DEFAULT_ADDRESS, EVTX_KEY_COUNT - 1);
        hid_printf("  (Connect GPIO %d <-> GPIO %d with a wire)\n", EVTX_GPIO_PIN, RADIO_GPIO_PIN);
        hid_printf("\n");
        hid_printf("Protocol: EV1527 (24-bit)\n");
        hid_printf("  - 20-bit Address + 4-bit Key/Button\n");
        hid_printf("  - Preamble: LONG LOW (25-40x HIGH)\n");
        hid_printf("  - Bit 0: Short HIGH + Long LOW\n");
        hid_printf("  - Bit 1: Long HIGH + Short LOW\n");
        hid_printf("\n");
        hid_printf("Commands: ping | status | reset | led | raw\n");
        hid_printf("----------------------------------------\n");
        hid_printf("Waiting for 433MHz EV1527 signal...\n\n");
        hid_flush();
    }

    uint32_t heartbeat_counter = 0;

    while (1) {
        /* 处理 TinyUSB 事件, 这会调用 tud_hid_set_report_cb 和刷新端点 */
        tud_task();

        /* 处理收到的 HID 命令 */
        if (cmd_pending) {
            cmd_pending = false;
            process_hid_command(cmd_buffer);
        }

        /* 从环形缓冲区读取脉冲数据 */
        uint32_t raw;
        bool had_data = false;
        while (ring_pop(&pulse_ring, &raw)) {
            had_data = true;
            pulse_t p;
            if (!parse_pulse(raw, &p)) continue;

            if (raw_mode) {
                if (raw_index < RAW_BUFFER_SIZE) {
                    raw_buffer[raw_index++] = raw;
                }
            } else {
                decoder_process_pulse(&decoder, &p);
            }
        }

        /* 原始模式定期发送 */
        if (raw_mode && raw_index > 0) {
            static absolute_time_t last_raw_send = {0};
            absolute_time_t now = get_absolute_time();
            if (absolute_time_diff_us(last_raw_send, now) > 500000) {
                send_raw_data();
                last_raw_send = now;
            }
        }

        /* 无数据时短暂休息 */
        if (!had_data) {
            sleep_us(500);
        }

        /* 心跳 */
        heartbeat_counter++;

        /* 超时提示 (10s 无信号) */
        if (heartbeat_counter % 1000 == 0) {
            absolute_time_t now = get_absolute_time();
            int64_t since_last = absolute_time_diff_us(decoder.last_pulse_time, now);
            if (since_last > 10000000) {
                hid_printf("[WAIT] No signal on GPIO %d for 10s...\n", RADIO_GPIO_PIN);
                hid_flush();
                decoder.last_pulse_time = now;
            }
        }
    }

    return 0;
}
