/**
 * rec.c - 433MHz EV1527 RF Decoder for RP2040
 *
 * Ported from AVR EV1527 decoder (github.com/aKaReZa75/AVR_ev1527)
 * Uses PIO state machine for hardware-accurate pulse width measurement
 * EV1527 protocol: 24-bit frame = 20-bit Address + 4-bit Key
 *
 * GPIO 18 → 433MHz receiver DATA pin
 * Output via USB CDC Serial
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"

// PIO 程序头文件 (由 pioasm 从 radio_rx.pio 生成)
#include "radio_rx.pio.h"

// Core 1: EV1527 发射模拟器 (GPIO 10 输出)
#include "ev1527_tx.h"

// ==================== 硬件配置 ====================

#define RADIO_GPIO_PIN  18      // 433MHz 接收模块数据引脚
#define LED_PIN         25      // Pico 板载 LED
#define RF_PIO          pio0    // 使用 PIO0
#define RF_PIO_SM       0       // 使用状态机 0

// ==================== 脉冲缓冲区 ====================

#define PULSE_RING_SIZE 256     // 环形缓冲区大小 (必须是 2 的幂)

typedef struct {
    uint32_t data[PULSE_RING_SIZE];
    volatile uint32_t head;     // 写入位置 (仅 ISR 写入)
    uint32_t tail;              // 读取位置 (仅主循环读取)
} pulse_ring_t;

static pulse_ring_t pulse_ring;

static inline bool ring_empty(pulse_ring_t *r) {
    return r->head == r->tail;
}

static inline bool ring_push(pulse_ring_t *r, uint32_t val) {
    uint32_t next = (r->head + 1) & (PULSE_RING_SIZE - 1);
    if (next == r->tail) return false;  // 满
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

// ==================== PIO 中断处理 ====================

static void __isr __time_critical_func(pio_irq_handler)(void) {
    while (!pio_sm_is_rx_fifo_empty(RF_PIO, RF_PIO_SM)) {
        uint32_t val = pio_sm_get(RF_PIO, RF_PIO_SM);
        ring_push(&pulse_ring, val);
    }
}

// ==================== 脉冲解析 ====================

// PIO 数据格式: bit31 = 电平(1=高,0=低), bit30:0 = 脉冲宽度(以 2 个 PIO 周期为单位)
// PIO @ 125MHz, 每个 cycle = 8ns, 每 2 个 cycle 计一次数 → 16ns/count
#define PULSE_LEVEL_MASK    0x80000000u
#define PULSE_WIDTH_MASK    0x7FFFFFFFu
#define PIO_CYCLE_NS        8       // 125MHz → 8ns per cycle
#define TICK_TO_US(t)       (((t) * 2 * PIO_CYCLE_NS) / 1000)   // ticks × 2 (cycles) → µs

typedef struct {
    bool     is_high;
    uint32_t ticks;     // 原始 PIO 计数值 (×2 cycles)
    uint32_t width_us;
} pulse_t;

static bool parse_pulse(uint32_t raw, pulse_t *p) {
    p->is_high  = (raw & PULSE_LEVEL_MASK) != 0;
    p->ticks    = (raw & PULSE_WIDTH_MASK) * 2;
    p->width_us = TICK_TO_US(raw & PULSE_WIDTH_MASK);
    return p->width_us > 0;
}

// ==================== EV1527 协议解码器 ====================

// EV1527 帧结构: [Preamble] [20-bit Address] [4-bit Key]
// Preamble: 长 LOW (≈31T) + 短 HIGH (≈1T)
// Bit 0: 短 HIGH (1T) + 长 LOW (3T)
// Bit 1: 长 HIGH (3T) + 短 LOW (1T)
// T ≈ 300-350µs (typical)

#define EV1527_FRAME_BITS   24          // 总帧长度: 20-bit address + 4-bit key
#define EV1527_MAX_INDEX    (EV1527_FRAME_BITS - 1)  // 23

// 时间验证阈值 (µs) — 基于 T≈300µs 的 EV1527 典型值
#define EV_MIN_PULSE_US     150         // 最小有效脉冲 (0.5T)
#define EV_MAX_PULSE_US     4500        // 最大有效脉冲 (~15T, 含 preamble)

// AVR 风格 preamble 检测: LOW 应为 HIGH 的 25~40 倍
// 转换为 µs 后直接比较
static inline bool ev1527_check_preamble(uint32_t low_us, uint32_t high_us) {
    return (low_us >= 25 * high_us) && (low_us <= 40 * high_us);
}

// AVR 风格 bit 判定: HIGH >= 1.5 × LOW → bit 1, 否则 bit 0
static inline int ev1527_decode_bit(uint32_t high_us, uint32_t low_us) {
    return (high_us >= (low_us + (low_us >> 1))) ? 1 : 0;  // high >= 1.5 * low
}

// 解码数据结构 — 与 AVR 版本兼容
typedef union {
    uint32_t rawValue;
    struct {
        uint32_t Address : 20;      // 20-bit 发射器唯一地址
        uint32_t Keys    : 4;       // 4-bit 按键码
        uint32_t Detect  : 1;       // 检测标志: 1=有效帧已接收
        uint32_t Reserve : 7;       // 保留
    } Bits;
} ev1527_data_t;

// 解码状态机 (与 AVR ISR 中的状态变量对应)
typedef enum {
    STATE_FIRST_TRIGGER,    // 等待首次边沿 (对应 AVR firstTime_Trigger=true)
    STATE_WAIT_PREAMBLE,    // 等待 preamble 检测 (对应 AVR preambleDetec=false)
    STATE_DECODE_BITS,      // 解码数据比特 (对应 AVR preambleDetec=true)
} ev1527_state_t;

typedef struct {
    ev1527_state_t state;
    ev1527_data_t  data;            // 解码结果 (与 AVR ev1527_Data 兼容)
    uint8_t        bit_index;       // 当前比特位置 (0-23)
    uint32_t       pending_high_us; // 待配对的高电平宽度
    bool           has_pending_high;
    uint32_t       frame_count;     // 已解码的帧数
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

// 输出解码帧
static void decoder_output_frame(decoder_t *d) {
    d->frame_count++;

    printf("\n========================================\n");
    printf("  FRAME #%u - EV1527 Decoded\n", d->frame_count);
    printf("========================================\n");
    printf("  Raw Value : 0x%08lX\n", d->data.rawValue);
    printf("  Address   : 0x%05lX (%lu)\n", d->data.Bits.Address, d->data.Bits.Address);
    printf("  Key/Button: 0x%X (%lu)\n", d->data.Bits.Keys, d->data.Bits.Keys);
    printf("----------------------------------------\n");

    // 二进制显示 (24 bits)
    printf("  Binary: ");
    for (int i = 23; i >= 0; i--) {
        printf("%d", (d->data.rawValue >> i) & 1);
        if (i == 4) printf(" | ");      // 分隔 Address 和 Key
        else if (i % 4 == 0 && i > 4) printf(" ");
    }
    printf("\n");
    printf("           <--- 20-bit Address ---> <-Key->\n");
    printf("========================================\n\n");
}

// 处理单个脉冲 (高电平或低电平)
static void decoder_process_pulse(decoder_t *d, pulse_t *p) {
    // 噪声过滤
    if (p->width_us < EV_MIN_PULSE_US || p->width_us > EV_MAX_PULSE_US) {
        decoder_reset(d);
        return;
    }

    d->last_pulse_time = get_absolute_time();

    switch (d->state) {

    // ---- 状态 0: 等待首次有效边沿 ----
    case STATE_FIRST_TRIGGER:
        if (p->is_high) {
            // 记录高电平，等待低电平
            d->pending_high_us = p->width_us;
            d->has_pending_high = true;
            d->state = STATE_WAIT_PREAMBLE;
        }
        // 忽略起始低电平
        break;

    // ---- 状态 1: 等待 preamble ----
    case STATE_WAIT_PREAMBLE:
        if (p->is_high) {
            // 新的高电平 → 更新 pending
            d->pending_high_us = p->width_us;
            d->has_pending_high = true;
        } else if (d->has_pending_high) {
            // 低电平到达 → 检查是否为 preamble
            uint32_t high_us = d->pending_high_us;
            uint32_t low_us  = p->width_us;
            d->has_pending_high = false;

            if (ev1527_check_preamble(low_us, high_us)) {
                // Preamble 检测成功！进入解码模式
                d->state = STATE_DECODE_BITS;
                d->data.rawValue = 0;
                d->bit_index = 0;
                printf("[SYNC] Preamble detected: HIGH=%lu us, LOW=%lu us\n", high_us, low_us);
            }
            // 不是 preamble，继续等待
        }
        break;

    // ---- 状态 2: 解码数据比特 ----
    case STATE_DECODE_BITS:
        if (p->is_high) {
            d->pending_high_us = p->width_us;
            d->has_pending_high = true;
        } else if (d->has_pending_high) {
            uint32_t high_us = d->pending_high_us;
            uint32_t low_us  = p->width_us;
            d->has_pending_high = false;

            // 检测新的 preamble (长 LOW = 帧结束/重复)
            if (ev1527_check_preamble(low_us, high_us)) {
                // 如果已收集足够比特，输出当前帧
                if (d->bit_index > 0) {
                    if (d->bit_index == EV1527_FRAME_BITS) {
                        d->data.Bits.Detect = 1;
                        decoder_output_frame(d);
                    } else {
                        printf("[DROP] Incomplete frame: %u bits\n", d->bit_index);
                    }
                }
                // 重新开始新帧
                d->state = STATE_DECODE_BITS;
                d->data.rawValue = 0;
                d->bit_index = 0;
                printf("[SYNC] Preamble detected: HIGH=%lu us, LOW=%lu us\n", high_us, low_us);
                break;
            }

            // 验证脉冲有效性
            uint32_t total = high_us + low_us;
            if (total < EV_MIN_PULSE_US || total > EV_MAX_PULSE_US) {
                printf("[ERR] Invalid pulse: H=%lu L=%lu us\n", high_us, low_us);
                decoder_reset(d);
                break;
            }

            // 解码比特 (AVR 风格: HIGH >= 1.5 * LOW)
            int bit = ev1527_decode_bit(high_us, low_us);
            d->data.rawValue = (d->data.rawValue << 1) | bit;
            d->bit_index++;

            // 检查是否收完 24 bits
            if (d->bit_index > EV1527_MAX_INDEX) {
                d->data.Bits.Detect = 1;
                decoder_output_frame(d);
                decoder_reset(d);
            }
        }
        break;
    }
}

// ==================== 串口命令处理 ====================

#define INPUT_BUFFER_SIZE 128
static char input_buffer[INPUT_BUFFER_SIZE];
static int input_index = 0;

static void process_serial_command(const char *cmd) {
    if (cmd[0] == '\0') return;

    if (strcmp(cmd, "ping") == 0) {
        printf("PICO LIVE\n");
    } else if (strcmp(cmd, "hello") == 0) {
        printf("Hello from EV1527 RF Decoder (RP2040)\n");
    } else if (strcmp(cmd, "led") == 0) {
        gpio_put(LED_PIN, !gpio_get(LED_PIN));
        printf("LED toggled (%s)\n", gpio_get(LED_PIN) ? "ON" : "OFF");
    } else if (strcmp(cmd, "status") == 0) {
        printf("Status: OK\n");
        printf("  GPIO pin: %d\n", RADIO_GPIO_PIN);
        printf("  Frames decoded: %u\n", decoder.frame_count);
        printf("  State: %s\n",
               decoder.state == STATE_FIRST_TRIGGER  ? "FIRST_TRIGGER" :
               decoder.state == STATE_WAIT_PREAMBLE  ? "WAIT_PREAMBLE" :
               decoder.state == STATE_DECODE_BITS    ? "DECODE_BITS" : "UNKNOWN");
        printf("  Last Address: 0x%05lX\n", decoder.data.Bits.Address);
        printf("  Last Key:     0x%X\n", decoder.data.Bits.Keys);
        printf("  Ring buffer: %lu/%u\n",
               (pulse_ring.head - pulse_ring.tail) & (PULSE_RING_SIZE - 1),
               PULSE_RING_SIZE);
    } else if (strcmp(cmd, "reset") == 0) {
        decoder_init(&decoder);
        pulse_ring.head = 0;
        pulse_ring.tail = 0;
        printf("Decoder reset\n");
    } else if (strcmp(cmd, "raw") == 0) {
        printf("RAW mode: next pulses in DATA: format\n");
    } else {
        printf("ECHO: %s\n", cmd);
    }
}

static void handle_serial_input(void) {
    while (true) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT || c == PICO_ERROR_NO_DATA) break;
        if (c < 0) continue;
        if (c == '\r') continue;

        if (c == '\n' || input_index >= INPUT_BUFFER_SIZE - 1) {
            input_buffer[input_index] = '\0';
            if (input_index > 0) {
                process_serial_command(input_buffer);
            }
            input_index = 0;
            continue;
        }

        input_buffer[input_index++] = (char)c;
    }
}

// ==================== 原始脉冲输出 (兼容模式) ====================

#define RAW_BUFFER_SIZE 512
static uint32_t raw_buffer[RAW_BUFFER_SIZE];
static int raw_index = 0;

static void send_raw_data(void) {
    if (raw_index <= 0) return;

    printf("DATA:");
    for (int i = 0; i < raw_index; i++) {
        pulse_t p;
        parse_pulse(raw_buffer[i], &p);
        printf("%lu", p.width_us);
        if (i < raw_index - 1) printf(",");
    }
    printf(",END\n");
    raw_index = 0;
}

// ==================== 硬件初始化 ====================

static void init_hardware(void) {
    // 1. USB CDC 串口
    stdio_init_all();

    // 2. 板载 LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    // 3. RF 接收引脚 (下拉)
    gpio_init(RADIO_GPIO_PIN);
    gpio_set_dir(RADIO_GPIO_PIN, GPIO_IN);
    gpio_pull_down(RADIO_GPIO_PIN);

    // 4. 加载并启动 PIO 程序
    uint offset = pio_add_program(RF_PIO, &radio_rx_program);
    radio_rx_program_init(RF_PIO, RF_PIO_SM, offset, RADIO_GPIO_PIN);

    // 5. PIO RX FIFO 中断
    pio_set_irq0_source_enabled(RF_PIO, pis_sm0_rx_fifo_not_empty, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);

    // 6. 初始化解码器
    decoder_init(&decoder);
    pulse_ring.head = 0;
    pulse_ring.tail = 0;
}

// ==================== 主程序 ====================

int main() {
    init_hardware();

    // 在 Core 1 上启动 EV1527 发射模拟器 (GPIO 10)
    // 物理上需要用跳线将 GPIO 10 连接到 GPIO 18 (接收引脚)
    multicore_launch_core1(ev1527_tx_run);
    printf("[CORE1] EV1527 TX simulator started on GPIO %d\n", EVTX_GPIO_PIN);

    // 启动指示: LED 闪烁 3 次
    for (int i = 0; i < 3; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(150);
        gpio_put(LED_PIN, 0);
        sleep_ms(150);
    }
    sleep_ms(500);

    printf("\n");
    printf("========================================\n");
    printf("  EV1527 RF Decoder for RP2040\n");
    printf("  Ported from AVR EV1527 library\n");
    printf("  GPIO %d @ 125MHz PIO (RX)\n", RADIO_GPIO_PIN);
    printf("========================================\n");
    printf("System Clock: %lu MHz\n", clock_get_hz(clk_sys) / 1000000);
    printf("\n");
    printf("Core 1 TX Simulator:\n");
    printf("  GPIO %d → EV1527 frames @ T=%d us\n", EVTX_GPIO_PIN, EVTX_T_US);
    printf("  Address: 0x%05lX, Keys: 0..%d\n", (uint32_t)EVTX_DEFAULT_ADDRESS, EVTX_KEY_COUNT - 1);
    printf("  (Connect GPIO %d ←→ GPIO %d with a wire)\n", EVTX_GPIO_PIN, RADIO_GPIO_PIN);
    printf("\n");
    printf("Protocol: EV1527 (24-bit)\n");
    printf("  - 20-bit Address + 4-bit Key/Button\n");
    printf("  - Preamble: LONG LOW (25-40x HIGH)\n");
    printf("  - Bit 0: Short HIGH + Long LOW\n");
    printf("  - Bit 1: Long HIGH + Short LOW\n");
    printf("\n");
    printf("Commands: ping | status | reset | led | raw\n");
    printf("----------------------------------------\n");
    printf("Waiting for 433MHz EV1527 signal...\n\n");

    uint32_t heartbeat_counter = 0;
    bool raw_mode = false;

    while (1) {
        handle_serial_input();

        // 检查是否有 "raw" 命令切换模式
        // (简单检测 — 也可通过串口命令处理)
        if (!raw_mode && strstr(input_buffer, "raw") != NULL) {
            raw_mode = true;
            printf("[MODE] Switched to RAW pulse output mode\n");
        }

        // 从环形缓冲区读取脉冲数据
        uint32_t raw;
        while (ring_pop(&pulse_ring, &raw)) {
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

        // 原始模式定期发送
        if (raw_mode && raw_index > 0) {
            static absolute_time_t last_raw_send = {0};
            absolute_time_t now = get_absolute_time();
            if (absolute_time_diff_us(last_raw_send, now) > 500000) {
                send_raw_data();
                last_raw_send = now;
            }
        }

        // 心跳
        heartbeat_counter++;
        if (heartbeat_counter % 200 == 0) {
            gpio_put(LED_PIN, !gpio_get(LED_PIN));
        }

        // 超时提示 (10s 无信号)
        if (heartbeat_counter % 1000 == 0) {
            absolute_time_t now = get_absolute_time();
            int64_t since_last = absolute_time_diff_us(decoder.last_pulse_time, now);
            if (since_last > 10000000) {
                printf("[WAIT] No signal on GPIO %d for 10s...\n", RADIO_GPIO_PIN);
                decoder.last_pulse_time = now;
            }
        }

        sleep_ms(10);
    }

    return 0;
}
