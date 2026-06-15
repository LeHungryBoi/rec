/**
 * ev1527_tx.h - EV1527 发射模拟器 (RP2040 Core 1)
 *
 * 模拟 EV1527 遥控器发射端，在指定 GPIO 上输出编码信号。
 * 信号可直接跳线连接到主解码器的接收引脚用于闭环测试。
 *
 * 协议: 24-bit frame = 20-bit Address + 4-bit Key
 *   Bit 0: Short HIGH (1T) + Long LOW (3T)
 *   Bit 1: Long HIGH (3T) + Short LOW (1T)
 *   Preamble: Short HIGH (1T) + Very Long LOW (31T)
 *   T ≈ 350µs (典型值)
 */

#ifndef _EV1527_TX_H_
#define _EV1527_TX_H_

#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 可配置参数
 * ================================================================ */

#ifndef EVTX_GPIO_PIN
#define EVTX_GPIO_PIN       17      // 发射模拟信号输出引脚
#endif

#ifndef EVTX_T_US
#define EVTX_T_US           350     // 基本时间单位 T (微秒)
#endif

#ifndef EVTX_FRAME_REPEAT_MS
#define EVTX_FRAME_REPEAT_MS 80    // 帧重复间隔 (毫秒)
#endif

/* ================================================================
 * 模拟遥控器配置
 * ================================================================ */

/** 模拟的发射器地址 (20-bit, 0 ~ 0xFFFFF) */
#define EVTX_DEFAULT_ADDRESS    0x3A5C1

/** 模拟的按键码序列 (轮流发送) */
#define EVTX_KEY_COUNT          4

/* ================================================================
 * API
 * ================================================================ */

/**
 * @brief 在 Core 1 上启动 EV1527 发射模拟器
 *
 * 初始化 GPIO 输出并进入无限循环:
 *   1. 发送 Preamble (1T HIGH + 31T LOW)
 *   2. 发送 20-bit Address (MSB first)
 *   3. 发送 4-bit Key (MSB first)
 *   4. 等待 EVTX_FRAME_REPEAT_MS 后重复
 *
 * 每轮发送不同的 Key (循环 0→1→2→3→0→...)
 * Address 固定为 EVTX_DEFAULT_ADDRESS
 *
 * @note 此函数不会返回，设计为在独立核心上运行
 */
void ev1527_tx_run(void);

#ifdef __cplusplus
}
#endif

#endif /* _EV1527_TX_H_ */
