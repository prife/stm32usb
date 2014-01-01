#ifndef _USB_H
#define _USB_H

struct descriptor
{
    const unsigned char * desc;
    int len;
};

#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05
#define HID_DESCRIPTOR_TYPE                     0x21

#define DESC_DEVICE_LENGTH        18

#define USB_DESC_CONFIG_SIZE       9
#define USB_DESC_INTERFACE_SIZE    9
#define USB_DESC_HID_SIZE          9
#define USB_DESC_ENDPOINT_SIZE     7
#define USB_DESC_CDCSUB_SIZE     (5+5+4+5)

typedef enum _HID_REQUESTS {
    GET_REPORT = 1,
    GET_IDLE,
    GET_PROTOCOL,

    SET_REPORT = 9,
    SET_IDLE,      //0x0A
    SET_PROTOCOL   //0x0B
} HID_REQUESTS;

typedef enum _CDC_REQUESTS {
    CDC_SET_LINE_CODING = 0x20,
    CDC_GET_LINE_CODING = 0x21,
    CDC_SET_CONTROL_LINE_STATE = 0x22,
} CDC_REQUESTS;

extern const struct descriptor DeviceDesc;
extern const struct descriptor ConfigDesc;
extern const struct descriptor StringDescTable[];
extern const unsigned char * str_desc_name_table[];
extern const struct descriptor ReportDesc[2];
#endif
