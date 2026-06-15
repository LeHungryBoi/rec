/**
 * ev1527_tx.c - EV1527 发射模拟器实现
 *
 * 在指定 GPIO 上生成符合 EV1527 协议的编码波形。
 * 使用 busy-wait 实现精确微秒级脉冲时序。
 *
 * 波形 (T = EVTX_T_US ≈ 350µs):
 *   Preamble:  HIGH = 1T,   LOW = 31T
 *   Bit 0:     HIGH = 1T,   LOW = 3T
 *   Bit 1:     HIGH = 3T,   LOW = 1T
 *
 * 帧结构 (MSB first):
 *   [Preamble] [Addr19 .. Addr0] [Key3 .. Key0]
 *              <-- 20-bit Address --> <4-bit>
 *
 * 设计运行在 RP2040 Core 1 上，不占用 Core 0 资源。
 */

#include "ev1527_tx.h"
#include "hardware/gpio.h"
#include "pico/time.h"

/* ================================================================
 * 内部函数
 * ================================================================ */

static inline void tx_delay_us(uint32_t us) {
    busy_wait_us_32(us);
}

/** 输出单个 EV1527 比特 */
static void tx_send_bit(bool bit) {
    if (bit) {
        /* Bit 1: 长高 (3T) + 短低 (1T) */
        gpio_put(EVTX_GPIO_PIN, 1);
        tx_delay_us(EVTX_T_US * 3);
        gpio_put(EVTX_GPIO_PIN, 0);
        tx_delay_us(EVTX_T_US);
    } else {
        /* Bit 0: 短高 (1T) + 长低 (3T) */
        gpio_put(EVTX_GPIO_PIN, 1);
        tx_delay_us(EVTX_T_US);
        gpio_put(EVTX_GPIO_PIN, 0);
        tx_delay_us(EVTX_T_US * 3);
    }
}

/** 发送 Preamble 同步头 */
static void tx_send_preamble(void) {
    gpio_put(EVTX_GPIO_PIN, 1);
    tx_delay_us(EVTX_T_US);          /* HIGH = 1T  */
    gpio_put(EVTX_GPIO_PIN, 0);
    tx_delay_us(EVTX_T_US * 31);     /* LOW  = 31T */
}

/** 发送完整 24-bit EV1527 帧 */
static void tx_send_frame(uint32_t address, uint8_t key) {
    tx_send_preamble();

    /* 20-bit Address, MSB first */
    for (int i = 19; i >= 0; i--) {
        tx_send_bit((address >> i) & 1);
    }

    /* 4-bit Key, MSB first */
    for (int i = 3; i >= 0; i--) {
        tx_send_bit((key >> i) & 1);
    }
}

/* ================================================================
 * 公开 API
 * ================================================================ */

void ev1527_tx_run(void) {
    /* 初始化输出引脚：初始低电平 */
    gpio_init(EVTX_GPIO_PIN);
    gpio_set_dir(EVTX_GPIO_PIN, GPIO_OUT);
    gpio_put(EVTX_GPIO_PIN, 0);

    /*
     * 等待一小段时间确保 Core 0 已完成初始化。
     * 避免在 Core 0 的 USB CDC 就绪前就开始灌数据。
     */
    busy_wait_ms(2000);

    uint8_t key_index = 0;

    while (1) {
        /*
         * 轮换 Key: 0, 1, 2, 3, 0, 1, ...
         * Address 始终固定为 EVTX_DEFAULT_ADDRESS
         */
        tx_send_frame(EVTX_DEFAULT_ADDRESS, key_index);
        key_index = (key_index + 1) % EVTX_KEY_COUNT;

        /* 帧间间隔，模拟遥控器重复发送的节奏 */
        busy_wait_ms(EVTX_FRAME_REPEAT_MS);
    }
}
