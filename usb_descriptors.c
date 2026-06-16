/*
 * usb_descriptors.c - USB Descriptors Implementation
 *
 * USB 设备描述符、配置描述符、字符串描述符、HID 报告描述符。
 *
 * 设备身份: VID=0xCAFE  PID=0x4005
 *
 * HID 报告:
 *   - Report ID 1 (OUT, 64 bytes): 主机发送命令到设备 (OUT端点)
 *   - Report ID 2 (IN, 64 bytes): 设备发送响应到主机 (IN端点)
 */

#include "usb_descriptors.h"

/* ================================================================
 * 设备描述符
 * ================================================================ */

static uint8_t const desc_device[] = {
    18,                             /* bLength */
    TUSB_DESC_DEVICE,               /* bDescriptorType: Device */
    0x00, 0x02,                     /* bcdUSB 2.0 (little-endian) */
    0x00,                           /* bDeviceClass (接口级别定义) */
    0x00,                           /* bDeviceSubClass */
    0x00,                           /* bDeviceProtocol */
    64,                             /* bMaxPacketSize0 */
    U16_TO_U8S_LE(USB_VID),        /* idVendor */
    U16_TO_U8S_LE(USB_PID),        /* idProduct */
    U16_TO_U8S_LE(USB_BCD_DEVICE), /* bcdDevice */
    STRID_MANUF,                    /* iManufacturer */
    STRID_PRODUCT,                  /* iProduct */
    STRID_SERIAL,                   /* iSerialNumber */
    0x01                            /* bNumConfigurations */
};

/* ================================================================
 * 端点编号 (desc_config 和 desc_hid_report 都需要, 提前定义)
 * ================================================================ */

enum {
    ITF_NUM = 0,
    EP_ADDR_IN  = 0x81, /* IN,  endpoint 1 */
    EP_ADDR_OUT = 0x01, /* OUT, endpoint 1 */
};

/* ================================================================
 * HID 报告描述符
 *
 * 必须定义在 desc_config 之前, 因为后者中用到 sizeof(desc_hid_report)
 *
 * 报告结构:
 *   Report ID 2 (IN, 64 bytes)  - 设备发送数据到主机
 *   Report ID 1 (OUT, 64 bytes) - 主机发送命令到设备
 * ================================================================ */

static uint8_t const desc_hid_report[] = {
    /* Usage Page: Vendor Defined 0xFF00 */
    0x06, 0x00, 0xFF,

    /* Usage: Vendor 1 */
    0x09, 0x01,

    /* Collection: Application */
    0xA1, 0x01,

        /* --- Input Report (ID 2, 64 bytes) --- */
        0x85, HID_REPORT_ID_RESPONSE,       /* Report ID = 2 */
        0x09, 0x03,                         /* Usage = 3 */
        0x15, 0x00,                         /* Logical Minimum = 0 */
        0x26, 0xFF, 0x00,                   /* Logical Maximum = 255 */
        0x75, 0x08,                         /* Report Size = 8 bits */
        0x96, REPORT_PACKET_SIZE, 0x00,     /* Report Count = 64 */
        0x81, 0x02,                         /* Input (Data,Var,Abs) */

        /* --- Output Report (ID 1, 64 bytes) --- */
        0x85, HID_REPORT_ID_COMMAND,        /* Report ID = 1 */
        0x09, 0x02,                         /* Usage = 2 */
        0x15, 0x00,                         /* Logical Minimum = 0 */
        0x26, 0xFF, 0x00,                   /* Logical Maximum = 255 */
        0x75, 0x08,                         /* Report Size = 8 bits */
        0x96, REPORT_PACKET_SIZE, 0x00,     /* Report Count = 64 */
        0x91, 0x02,                         /* Output (Data,Var,Abs) */

    /* End Collection */
    0xC0,
};

/* ================================================================
 * 配置描述符
 *
 * 结构:
 *   Configuration (9)
 *     Interface (9)      - HID
 *       HID (9)
 *         Endpoint IN  (7)  - 中断输入 (设备→主机)
 *         Endpoint OUT (7)  - 中断输出 (主机→设备)
 *
 * 总计: 41 字节
 * ================================================================ */

static uint8_t const desc_config[] = {
    /* ---------- 配置描述符 (9 bytes) ---------- */
    9,                              /* bLength */
    TUSB_DESC_CONFIGURATION,        /* bDescriptorType */
    U16_TO_U8S_LE(41),             /* wTotalLength */
    1,                              /* bNumInterfaces */
    1,                              /* bConfigurationValue */
    0,                              /* iConfiguration */
    TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, /* bmAttributes */
    100,                            /* bMaxPower (200 mA) */

    /* ---------- 接口描述符 (9 bytes) ---------- */
    9,                              /* bLength */
    TUSB_DESC_INTERFACE,            /* bDescriptorType */
    ITF_NUM,                        /* bInterfaceNumber */
    0,                              /* bAlternateSetting */
    2,                              /* bNumEndpoints (IN + OUT) */
    TUSB_CLASS_HID,                 /* bInterfaceClass: HID */
    0,                              /* bInterfaceSubClass (none) */
    HID_ITF_PROTOCOL_NONE,          /* bInterfaceProtocol */
    STRID_PRODUCT,                  /* iInterface */

    /* ---------- HID 描述符 (9 bytes) ---------- */
    9,                              /* bLength */
    HID_DESC_TYPE_HID,              /* bDescriptorType: HID */
    U16_TO_U8S_LE(0x0111),         /* bcdHID 1.11 */
    0x00,                           /* bCountryCode */
    1,                              /* bNumDescriptors */
    HID_DESC_TYPE_REPORT,           /* bDescriptorType: Report */
    U16_TO_U8S_LE(sizeof(desc_hid_report)), /* wDescriptorLength */

    /* ---------- Endpoint IN 描述符 (7 bytes) ---------- */
    7,                              /* bLength */
    TUSB_DESC_ENDPOINT,             /* bDescriptorType */
    EP_ADDR_IN,                     /* bEndpointAddress (IN, EP1) */
    TUSB_XFER_INTERRUPT,            /* bmAttributes: Interrupt */
    U16_TO_U8S_LE(REPORT_PACKET_SIZE),/* wMaxPacketSize: 64 */
    1,                              /* bInterval: 1ms */

    /* ---------- Endpoint OUT 描述符 (7 bytes) ---------- */
    7,                              /* bLength */
    TUSB_DESC_ENDPOINT,             /* bDescriptorType */
    EP_ADDR_OUT,                    /* bEndpointAddress (OUT, EP1) */
    TUSB_XFER_INTERRUPT,            /* bmAttributes: Interrupt */
    U16_TO_U8S_LE(REPORT_PACKET_SIZE),/* wMaxPacketSize: 64 */
    1,                              /* bInterval: 1ms */
};

/* ================================================================
 * 字符串描述符
 *
 * TinyUSB 字符串描述符格式 (uint16_t 数组, 小端):
 *   [0] = (TUSB_DESC_STRING << 8) | bLength
 *   [1..n] = UTF-16LE 编码字符
 *   bLength = 2 * (字符数 + 1)
 * ================================================================ */

static uint16_t const string_langid[] = {
    0x0304,     /* bLength=4, bDescriptorType=0x03 */
    0x0409      /* English (US) */
};

static uint16_t const string_manufacturer[] = {
    0x0316,     /* bLength=22, bDescriptorType=0x03 */
    'R', 'F', ' ', 'D', 'e', 'c', 'o', 'd', 'e', 'r'
};

static uint16_t const string_product[] = {
    0x0326,     /* bLength=38, bDescriptorType=0x03 */
    'E', 'V', '1', '5', '2', '7', ' ',
    'H', 'I', 'D', ' ',
    'D', 'e', 'c', 'o', 'd', 'e', 'r'
};

static uint16_t const string_serial[] = {
    0x030A,     /* bLength=10, bDescriptorType=0x03 */
    '0', '0', '0', '1'
};

/* ================================================================
 * 描述符回调
 * ================================================================ */

uint8_t const *tud_descriptor_device_cb(void) {
    return desc_device;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_config;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

    switch (index) {
    case STRID_LANGID:
        return string_langid;
    case STRID_MANUF:
        return string_manufacturer;
    case STRID_PRODUCT:
        return string_product;
    case STRID_SERIAL:
        return string_serial;
    default:
        return NULL;
    }
}

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return desc_hid_report;
}
