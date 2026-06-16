/*
 * tusb_config.h - TinyUSB Configuration for RP2040 HID Device
 *
 * Overrides the default Pico SDK TinyUSB configuration to enable
 * HID class while keeping only the USB device stack active.
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* 通用配置                                                            */
/* ------------------------------------------------------------------ */

/* 仅使用设备模式，禁用主机模式 */
#define CFG_TUSB_RHPORT0_MODE       OPT_MODE_DEVICE

/* 设备栈配置 */
#define CFG_TUD_ENABLED             1

/* ------------------------------------------------------------------ */
/* 设备类配置                                                          */
/* ------------------------------------------------------------------ */

/* 禁用不需要的设备类 */
#define CFG_TUD_CDC                 0
#define CFG_TUD_MSC                 0
#define CFG_TUD_MIDI                0
#define CFG_TUD_VENDOR              0

/* 启用 HID 类, 1 个实例 */
#define CFG_TUD_HID                 1

/* 禁用 Audio, BTH, DFU, ECM/RNDIS, USBTPD */
#define CFG_TUD_AUDIO               0
#define CFG_TUD_BTH                 0
#define CFG_TUD_DFU                 0
#define CFG_TUD_ECM_RNDIS           0
#define CFG_TUD_USBTPD              0

/* ------------------------------------------------------------------ */
/* HID 缓冲区配置                                                       */
/* ------------------------------------------------------------------ */

/* HID 输入报告缓冲区大小 (设备 → 主机) */
#define CFG_TUD_HID_EP_BUFSIZE      64

/* HID 输出报告缓冲区大小 (主机 → 设备) */
#define CFG_TUD_HID_EP_OUT_BUFSIZE  64

/* ------------------------------------------------------------------ */
/* 操作系统 / 调度器 — Pico 无 RTOS                                    */
/* ------------------------------------------------------------------ */

#define CFG_TUSB_OS                 OPT_OS_PICO

/* ------------------------------------------------------------------ */
/* 内存对齐                                                            */
/* ------------------------------------------------------------------ */

#define CFG_TUSB_MEM_ALIGN          TU_ATTR_ALIGNED(4)

/* ------------------------------------------------------------------ */
/* 调试输出 (留空则禁用)                                               */
/* ------------------------------------------------------------------ */

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG              0
#endif

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
