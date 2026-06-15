/**
 * rec.c - 433MHz 射频信号接收解码器
 *
 * 通过 GPIO 18 读取 433MHz 接收模块数据
 * PIO 状态机精确测量脉冲宽度
 * 支持 PT2262 / EV1527 协议解码
 * 解码结果通过 USB CDC 串口实时输出
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/gpio.h"

// PIO 程序头文件 (由 pioasm 从 radio_rx.pio 生成)
#include "radio_rx.pio.h"

// ==================== 硬件配置 ====================

#define RADIO_GPIO_PIN  18      // 433MHz 接收模块数据引脚
#define LED_PIN         25      // Pico 板载 LED
#define RF_PIO          pio0    // 使用 PIO0
#define RF_PIO_SM       0       // 使用状态机 0

// ==================== 脉冲缓冲区 ====================

#define PULSE_RING_SIZE 256     // 环形缓冲区大小 (必须是 2 的幂)

typedef struct {
    uint32_t data[PULSE_RING_SIZE];
    volatile uint32_t head;     // 写入位置
    volatile uint32_t tail;     // 读取位置
} pulse_ring_t;

static pulse_ring_t pulse_ring;

static inline bool ring_empty(pulse_ring_t *r) {
    return r->head == r->tail;
}

static inline bool ring_full(pulse_ring_t *r) {
    return ((r->head + 1) & (PULSE_RING_SIZE - 1)) == r->tail;
}

static inline bool ring_push(pulse_ring_t *r, uint32_t val) {
    if (ring_full(r)) return false;
    r->data[r->head] = val;
    r->head = (r->head + 1) & (PULSE_RING_SIZE - 1);
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
    // 从 RX FIFO 批量读取所有可用数据
    while (!pio_sm_is_rx_fifo_empty(RF_PIO, RF_PIO_SM)) {
        uint32_t val = pio_sm_get(RF_PIO, RF_PIO_SM);
        ring_push(&pulse_ring, val);
    }
}

// ==================== 脉冲解析 ====================

// PIO 数据格式: bit31 = 电平(1=高,0=低), bit30:0 = 脉冲宽度(时钟周期)
// 但由于 PIO 循环每次 2 个时钟周期，实际时间 = 计数值 * 2

#define PULSE_LEVEL_MASK    0x80000000u
#define PULSE_WIDTH_MASK    0x7FFFFFFFu
#define PIO_CYCLE_NS        8       // 125MHz → 8ns per cycle, 测量值需 ×2 → 16ns per count

typedef struct {
    bool     is_high;       // true=高电平, false=低电平
    uint32_t width_ticks;   // 脉冲宽度 (PIO 时钟周期 × 2)
    uint32_t width_us;      // 脉冲宽度 (微秒)
} pulse_t;

static bool parse_pulse(uint32_t raw, pulse_t *p) {
    p->is_high    = (raw & PULSE_LEVEL_MASK) != 0;
    p->width_ticks = (raw & PULSE_WIDTH_MASK) * 2;  // PIO 循环 2 周期
    p->width_us   = p->width_ticks * PIO_CYCLE_NS / 1000;
    return p->width_us > 0;
}

// ==================== 协议解码引擎 ====================

// PT2262 / EV1527 协议参数
// 同步头: 高电平约 1T, 低电平约 31T (PT2262) 或 4T~10T (EV1527)
// 比特 0: 高=1T, 低=3T
// 比特 1: 高=3T, 低=1T
// 帧长度: PT2262 = 12位三态(24比特), EV1527 = 20位+4位(24比特)

#define MIN_PULSE_US        50      // 最小有效脉冲宽度 (μs)
#define MAX_PULSE_US        30000   // 最大有效脉冲宽度 (μs)
#define SYNC_MIN_LOW_US     2000    // 同步头低电平最小宽度 (μs)
#define SYNC_MAX_LOW_US     30000   // 同步头低电平最大宽度 (μs)
#define SYNC_HIGH_MIN_US    100     // 同步头高电平最小宽度 (μs)
#define BIT_TOLERANCE       0.40f   // 比特判定容差 40%
#define MIN_FRAME_BITS      12      // 最少帧比特数
#define MAX_FRAME_BITS      32      // 最多帧比特数

typedef enum {
    STATE_WAIT_SYNC_HIGH,   // 等待同步头高电平
    STATE_WAIT_SYNC_LOW,    // 等待同步头低电平
    STATE_COLLECT_BITS,     // 采集数据比特
} decoder_state_t;

typedef struct {
    decoder_state_t state;
    uint32_t code;              // 解码结果
    uint8_t  bit_count;         // 已收到的比特数
    uint32_t t_unit;            // 自动检测的时间单位 T (μs)
    bool     t_calibrated;      // T 是否已校准
    uint32_t sync_high_us;      // 同步头高电平宽度 (用于校准 T)
    uint32_t frame_count;       // 已解码的帧数
    absolute_time_t last_pulse_time; // 上一次有效脉冲时间
    uint32_t pending_high_us;   // 等待配对的高电平宽度
    bool     has_pending_high;  // 是否有待配对的高电平
} decoder_t;

static decoder_t decoder;

static void decoder_reset(decoder_t *d) {
    d->state = STATE_WAIT_SYNC_HIGH;
    d->code = 0;
    d->bit_count = 0;
    d->t_calibrated = false;
    d->t_unit = 0;
    d->sync_high_us = 0;
    d->pending_high_us = 0;
    d->has_pending_high = false;
}

static void decoder_init(decoder_t *d) {
    decoder_reset(d);
    d->frame_count = 0;
    d->last_pulse_time = get_absolute_time();
}

// 基于 T 值判定比特
// 返回: 0 = bit 0, 1 = bit 1, -1 = 无法判定
static int classify_bit(uint32_t high_us, uint32_t low_us, uint32_t t) {
    if (t == 0) return -1;

    float ratio_h = (float)high_us / (float)t;
    float ratio_l = (float)low_us / (float)t;

    // 比特 0: high ≈ 1T, low ≈ 3T
    if (ratio_h > (1.0f - BIT_TOLERANCE) && ratio_h < (1.0f + BIT_TOLERANCE) &&
        ratio_l > (3.0f - BIT_TOLERANCE * 3) && ratio_l < (3.0f + BIT_TOLERANCE * 3)) {
        return 0;
    }

    // 比特 1: high ≈ 3T, low ≈ 1T
    if (ratio_h > (3.0f - BIT_TOLERANCE * 3) && ratio_h < (3.0f + BIT_TOLERANCE * 3) &&
        ratio_l > (1.0f - BIT_TOLERANCE) && ratio_l < (1.0f + BIT_TOLERANCE)) {
        return 1;
    }

    return -1;
}

// 尝试从同步头高电平推断 T
static uint32_t estimate_t_from_sync(uint32_t sync_high_us) {
    // PT2262 同步头高电平 ≈ 1T
    if (sync_high_us > 50 && sync_high_us < 2000) {
        return sync_high_us;
    }
    return 0;
}

// 尝试从第一对比特自动校准 T
static uint32_t estimate_t_from_bit(uint32_t high_us, uint32_t low_us) {
    uint32_t total = high_us + low_us;
    // 每比特总宽度 = 4T (无论 0 还是 1)
    if (total > 200 && total < 8000) {
        return total / 4;
    }
    return 0;
}

// 输出解码帧
static void output_frame(decoder_t *d) {
    d->frame_count++;

    printf("[FRAME #%u] %u bits, Code: 0x%08lX\n",
           d->frame_count, d->bit_count, d->code);

    // 输出二进制
    printf("  Binary: ");
    for (int i = d->bit_count - 1; i >= 0; i--) {
        printf("%d", (d->code >> i) & 1);
        if (i % 4 == 0 && i > 0) printf(" ");
    }
    printf("\n");

    // 输出三态解码 (PT2262 格式: 每2位一组)
    if (d->bit_count >= 12 && d->bit_count % 2 == 0) {
        printf("  Tri-state: ");
        int nibbles = d->bit_count / 2;
        for (int i = nibbles - 1; i >= 0; i--) {
            int b1 = (d->code >> (i * 2 + 1)) & 1;
            int b2 = (d->code >> (i * 2)) & 1;
            if (b1 && b2)       printf("F");
            else if (!b1 && !b2) printf("0");
            else if (b1)        printf("1");
            else                printf("X"); // 浮空态
        }
        printf("\n");
    }

    // 兼容 capture_viewer.html 的 DATA: 格式
    printf("  T-unit: %lu us\n", d->t_unit);
    printf("---\n");
}

// 处理一个脉冲，驱动解码状态机
static void decoder_process_pulse(decoder_t *d, pulse_t *p) {
    // 忽略过短或过长的脉冲 (噪声)
    if (p->width_us < MIN_PULSE_US || p->width_us > MAX_PULSE_US) {
        decoder_reset(d);
        return;
    }

    d->last_pulse_time = get_absolute_time();

    switch (d->state) {
    case STATE_WAIT_SYNC_HIGH:
        // 等待一个合理宽度的高电平作为同步头开始
        if (p->is_high && p->width_us >= SYNC_HIGH_MIN_US && p->width_us < 5000) {
            d->sync_high_us = p->width_us;
            d->state = STATE_WAIT_SYNC_LOW;
        }
        break;

    case STATE_WAIT_SYNC_LOW:
        if (!p->is_high) {
            // 检查低电平是否符合同步头特征 (很长的低电平)
            if (p->width_us >= SYNC_MIN_LOW_US && p->width_us <= SYNC_MAX_LOW_US) {
                // 同步头确认！进入比特采集
                d->t_unit = estimate_t_from_sync(d->sync_high_us);
                d->t_calibrated = (d->t_unit > 0);
                d->code = 0;
                d->bit_count = 0;
                d->state = STATE_COLLECT_BITS;

                printf("[SYNC] high=%lu us, low=%lu us, T≈%lu us\n",
                       d->sync_high_us, p->width_us, d->t_unit);
            } else {
                // 不是同步头，重新搜索
                decoder_reset(d);
            }
        } else {
            // 连续两个高电平，第一个不是同步头
            if (p->width_us >= SYNC_HIGH_MIN_US && p->width_us < 5000) {
                d->sync_high_us = p->width_us;
                // 保持 STATE_WAIT_SYNC_LOW
            } else {
                decoder_reset(d);
            }
        }
        break;

    case STATE_COLLECT_BITS:
        // 需要成对读取 高+低 来判定一个比特
        {
            if (p->is_high) {
                d->pending_high_us = p->width_us;
                d->has_pending_high = true;
            } else if (!p->is_high && d->has_pending_high) {
                // 有一对 高+低，尝试判定比特
                uint32_t high_us = d->pending_high_us;
                uint32_t low_us = p->width_us;
                d->has_pending_high = false;

                // 如果 T 还没校准，尝试从第一对比特校准
                if (!d->t_calibrated) {
                    uint32_t est_t = estimate_t_from_bit(high_us, low_us);
                    if (est_t > 0) {
                        d->t_unit = est_t;
                        d->t_calibrated = true;
                        printf("[CALIB] T auto-calibrated to %lu us\n", d->t_unit);
                    }
                }

                // 检查是否遇到了新的同步头 (长低电平 = 帧结束)
                if (low_us >= SYNC_MIN_LOW_US) {
                    // 帧结束
                    if (d->bit_count >= MIN_FRAME_BITS) {
                        output_frame(d);
                    } else if (d->bit_count > 0) {
                        printf("[DROP] Only %u bits collected\n", d->bit_count);
                    }

                    // 新的同步头: 高电平已经在这个比特对的高电平里
                    d->sync_high_us = high_us;
                    d->state = STATE_WAIT_SYNC_LOW;
                    break;
                }

                int bit = classify_bit(high_us, low_us, d->t_unit);

                if (bit >= 0) {
                    d->code = (d->code << 1) | bit;
                    d->bit_count++;

                    // 检查是否已收集够比特
                    if (d->bit_count >= MAX_FRAME_BITS) {
                        output_frame(d);
                        decoder_reset(d);
                    }
                } else {
                    // 无法判定，可能是噪声或帧间间隔
                    // 如果已收集足够的比特，输出当前帧
                    if (d->bit_count >= MIN_FRAME_BITS) {
                        output_frame(d);
                    } else if (d->bit_count > 0) {
                        printf("[ERR] Decode error at bit %u (h=%lu l=%lu T=%lu)\n",
                               d->bit_count, high_us, low_us, d->t_unit);
                    }
                    decoder_reset(d);
                }
            }
            // 如果是低电平但没有 pending high，跳过
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
        printf("Hello from 433MHz RF Decoder\n");
    } else if (strcmp(cmd, "led") == 0) {
        gpio_put(LED_PIN, !gpio_get(LED_PIN));
        printf("LED toggled (%s)\n", gpio_get(LED_PIN) ? "ON" : "OFF");
    } else if (strcmp(cmd, "status") == 0) {
        printf("Status: OK\n");
        printf("  GPIO pin: %d\n", RADIO_GPIO_PIN);
        printf("  Frames decoded: %u\n", decoder.frame_count);
        printf("  T-unit: %lu us %s\n", decoder.t_unit,
               decoder.t_calibrated ? "(calibrated)" : "(not calibrated)");
        printf("  Ring buffer: %u/%u\n",
               (pulse_ring.head - pulse_ring.tail) & (PULSE_RING_SIZE - 1),
               PULSE_RING_SIZE);
        printf("  State: %s\n",
               decoder.state == STATE_WAIT_SYNC_HIGH ? "SYNC_SEARCH" :
               decoder.state == STATE_WAIT_SYNC_LOW  ? "SYNC_LOW" :
               decoder.state == STATE_COLLECT_BITS   ? "COLLECTING" : "UNKNOWN");
    } else if (strcmp(cmd, "reset") == 0) {
        decoder_init(&decoder);
        pulse_ring.head = 0;
        pulse_ring.tail = 0;
        printf("Decoder reset\n");
    } else if (strcmp(cmd, "raw") == 0) {
        // 原始脉冲输出模式 (兼容 capture_viewer.html)
        printf("RAW mode: next pulses will be in DATA: format\n");
    } else if (strncmp(cmd, "tunit ", 6) == 0) {
        // 手动设置 T 单位: "tunit 350"
        uint32_t t = atol(cmd + 6);
        if (t > 0) {
            decoder.t_unit = t;
            decoder.t_calibrated = true;
            printf("T-unit manually set to %lu us\n", t);
        } else {
            printf("Invalid T value\n");
        }
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
    // 1. USB CDC 串口初始化
    stdio_init_all();

    // 2. 板载 LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    // 3. RF 接收引脚 (下拉，无信号时为低电平)
    gpio_init(RADIO_GPIO_PIN);
    gpio_set_dir(RADIO_GPIO_PIN, GPIO_IN);
    gpio_pull_down(RADIO_GPIO_PIN);

    // 4. 加载 PIO 程序
    uint offset = pio_add_program(RF_PIO, &radio_rx_program);

    // 5. 初始化 PIO 状态机
    radio_rx_program_init(RF_PIO, RF_PIO_SM, offset, RADIO_GPIO_PIN);

    // 6. 配置 PIO 中断: RX FIFO 非空时触发
    pio_set_irq0_source_enabled(RF_PIO, pis_sm0_rx_fifo_not_empty, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, pio_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);

    // 7. 初始化解码器
    decoder_init(&decoder);

    // 8. 初始化环形缓冲区
    pulse_ring.head = 0;
    pulse_ring.tail = 0;
}

// ==================== 主程序 ====================

int main() {
    init_hardware();

    // 等待 USB CDC 枚举完成
    // LED 闪烁 3 次指示启动阶段
    for (int i = 0; i < 3; i++) {
        gpio_put(LED_PIN, 1);
        sleep_ms(150);
        gpio_put(LED_PIN, 0);
        sleep_ms(150);
    }
    sleep_ms(500);

    printf("\n");
    printf("================================\n");
    printf("  433MHz RF Decoder v2.0\n");
    printf("  GPIO %d - PT2262/EV1527\n", RADIO_GPIO_PIN);
    printf("================================\n");
    printf("System Clock: %u MHz\n", clock_get_hz(clk_sys) / 1000000);
    printf("PIO Clock:    125 MHz (no divider)\n");
    printf("\n");
    printf("Commands:\n");
    printf("  ping      - Verify Pico is alive\n");
    printf("  status    - Show decoder status\n");
    printf("  reset     - Reset decoder state\n");
    printf("  led       - Toggle onboard LED\n");
    printf("  tunit N   - Set T-unit manually (us)\n");
    printf("  raw       - Enable raw pulse output\n");
    printf("  any text  - Echo back\n");
    printf("--------------------------------\n");
    printf("Waiting for 433MHz signal...\n\n");

    uint32_t heartbeat_counter = 0;
    bool raw_mode = false;

    while (1) {
        handle_serial_input();

        // 从环形缓冲区读取脉冲数据
        uint32_t raw;
        while (ring_pop(&pulse_ring, &raw)) {
            pulse_t p;
            if (!parse_pulse(raw, &p)) continue;

            if (raw_mode) {
                // 原始模式: 存入缓冲区供 capture_viewer.html 使用
                if (raw_index < RAW_BUFFER_SIZE) {
                    raw_buffer[raw_index++] = raw;
                }
            } else {
                // 解码模式: 送入协议解码器
                decoder_process_pulse(&decoder, &p);
            }
        }

        // 原始模式下定期发送数据
        if (raw_mode && raw_index > 0) {
            static absolute_time_t last_raw_send = 0;
            absolute_time_t now = get_absolute_time();
            if (absolute_time_diff_us(last_raw_send, now) > 500000) {  // 每 500ms
                send_raw_data();
                last_raw_send = now;
            }
        }

        // 心跳: LED 闪烁 + 串口存活标记
        heartbeat_counter++;
        if (heartbeat_counter % 200 == 0) {
            gpio_put(LED_PIN, !gpio_get(LED_PIN));
        }

        // 超时检测: 如果超过 10 秒没有脉冲，打印等待提示
        if (heartbeat_counter % 1000 == 0) {
            absolute_time_t now = get_absolute_time();
            int64_t since_last = absolute_time_diff_us(decoder.last_pulse_time, now);
            if (since_last > 10000000) {  // 10 秒
                printf("[WAIT] No signal on GPIO %d for 10s...\n", RADIO_GPIO_PIN);
                decoder.last_pulse_time = now;  // 避免重复打印
            }
        }

        sleep_ms(10);
    }

    return 0;
}
