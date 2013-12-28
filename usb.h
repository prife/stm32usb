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
#define DESC_DEVICE_LENGTH  18

#define USB_DESC_CONFIG_SIZE       9
#define USB_DESC_INTERFACE_SIZE    9
#define USB_DESC_HID_SIZE          9
#define USB_DESC_ENDPOINT_SIZE     7

extern const struct descriptor DeviceDesc;
extern const struct descriptor ConfigDesc;
extern const struct descriptor StringDescTable[];
extern const unsigned char * str_desc_name_table[];
#endif
