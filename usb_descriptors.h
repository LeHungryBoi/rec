/*
 * usb_descriptors.h - USB Descriptors for EV1527 RF Decoder HID Device
 *
 * Declares constants and callback prototypes for the custom HID device.
 * VID:PID = 0xCAFE:0x4005
 */

#ifndef _USB_DESCRIPTORS_H_
#define _USB_DESCRIPTORS_H_

#include "tusb.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 * 设备标识
 * ================================================================ */

#define USB_VID         0xCAFE
#define USB_PID         0x4005

#define USB_BCD_DEVICE  0x0100      /* 版本 1.00 */
#define USB_BCD_USB     0x0200      /* USB 2.0 */

/* ================================================================
 * HID 报告 ID
 * ================================================================ */

#define HID_REPORT_ID_COMMAND   0x01    /* 输出报告: 主机 → 设备 (命令) */
#define HID_REPORT_ID_RESPONSE  0x02    /* 输入报告: 设备 → 主机 (响应) */

/* ================================================================
 * 报告缓冲区大小
 * ================================================================ */

#define REPORT_PACKET_SIZE      64

/* ================================================================
 * 字符串描述符索引
 * ================================================================ */

enum {
    STRID_LANGID   = 0,
    STRID_MANUF    = 1,
    STRID_PRODUCT  = 2,
    STRID_SERIAL   = 3,
};

/* ================================================================
 * 回调函数 (由 TinyUSB 调用, 在 usb_descriptors.c 中实现)
 * ================================================================ */

/* 设备描述符 */
uint8_t const *tud_descriptor_device_cb(void);

/* 配置描述符 */
uint8_t const *tud_descriptor_configuration_cb(uint8_t index);

/* 字符串描述符 */
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid);

/* HID 报告描述符 */
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance);

#ifdef __cplusplus
}
#endif

#endif /* _USB_DESCRIPTORS_H_ */
