#include "usb.h"
#include "usb_conf.h"

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef char s8;
typedef short s16;
typedef int s32;

/* USB Standard Device Descriptor */
const u8 Joystick_DeviceDescriptor[] = {
	0x12,                       /*bLength */
	USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
	0x10,                       /*bcdUSB */
	0x01,
	0x00,                       /*bDeviceClass*/
	0x00,                       /*bDeviceSubClass*/
	0x00,                       /*bDeviceProtocol*/
	0x40,                       /*bMaxPacketSize40*/
	0x6E,                       /*idVendor (0x3689)*/   //FIXME<-------------------厂商ID字段
	0x09,
	0x04,                       /*idProduct = 0x8762*/  //FIXME<-------------------idProduct
	0x03,
	0x00,                       /*bcdDevice rel. 2.00*/ //FIXME<-------------------加密狗版本号
	0x02,
	0,                          /*Index of string descriptor describing
                                              manufacturer */
	0,                          /*Index of string descriptor describing
                                             product*/
	0,                          /*Index of string descriptor describing the
                                             device serial number */
	0x01                        /*bNumConfigurations*/
}; /* Joystick_DeviceDescriptor */

/* USB Report Descriptor */
const u8 Joystick_ReportDescriptor[] =
{
  0x05,          /*Usage Page(Generic Desktop)*/
  0x01,
  0x09,          /*Usage(Mouse)*/
  0x02,
  0xA1,          /*Collection(Logical)*/
  0x01,
  0x09,          /*Usage(Pointer)*/
  0x01,
  /* 8 */
  0xA1,          /*Collection(Linked)*/
  0x00,
  0x05,          /*Usage Page(Buttons)*/
  0x09,
  0x19,          /*Usage Minimum(1)*/
  0x01,
  0x29,          /*Usage Maximum(3)*/
  0x03,
  /* 16 */
  0x15,          /*Logical Minimum(0)*/
  0x00,
  0x25,          /*Logical Maximum(1)*/
  0x01,
  0x95,          /*Report Count(3)*/
  0x03,
  0x75,          /*Report Size(1)*/
  0x01,
  /* 24 */
  0x81,          /*Input(Variable)*/
  0x02,
  0x95,          /*Report Count(1)*/
  0x01,
  0x75,          /*Report Size(5)*/
  0x05,
  0x81,          /*Input(Constant,Array)*/
  0x01,
  /* 32 */
  0x05,          /*Usage Page(Generic Desktop)*/
  0x01,
  0x09,          /*Usage(X axis)*/
  0x30,
  0x09,          /*Usage(Y axis)*/
  0x31,
  0x09,          /*Usage(Wheel)*/
  0x38,
  /* 40 */
  0x15,          /*Logical Minimum(-127)*/
  0x81,
  0x25,          /*Logical Maximum(127)*/
  0x7F,
  0x75,          /*Report Size(8)*/
  0x08,
  0x95,          /*Report Count(3)*/
  0x03,
  /* 48 */
  0x81,          /*Input(Variable, Relative)*/
  0x06,
  0xC0,          /*End Collection*/
  0x09,
  0x3c,
  0x05,
  0xff,
  0x09,
  /* 56 */
  0x01,
  0x15,
  0x00,
  0x25,
  0x01,
  0x75,
  0x01,
  0x95,
  /* 64 */
  0x02,
  0xb1,
  0x22,
  0x75,
  0x06,
  0x95,
  0x01,
  0xb1,
  /* 72 */
  0x01,
  0xc0
}; /* Joystick_ReportDescriptor */

const struct descriptor ReportDesc =
{ Joystick_ReportDescriptor, sizeof(Joystick_ReportDescriptor), };

/* USB Configuration Descriptor */

#define JOYSTICK_SIZ_CONFIG_DESC (USB_DESC_CONFIG_SIZE    + \
                                  USB_DESC_INTERFACE_SIZE + \
                                  USB_DESC_HID_SIZE       + \
                       USB_DESC_ENDPOINT_SIZE * CONFIG_EP_NUM)

/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
const u8 Joystick_ConfigDescriptor[JOYSTICK_SIZ_CONFIG_DESC] = {
	0x09, /* bLength: Configuation Descriptor size */
	USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
	JOYSTICK_SIZ_CONFIG_DESC,
	/* wTotalLength: Bytes returned */
	0x00,
	0x01,         /*bNumInterfaces: 1 interface*/
	0x01,         /*bConfigurationValue: Configuration value*/
	0x00,         /*iConfiguration: Index of string descriptor describing
                                 the configuration*///<------------??
	0x80,         /*bmAttributes: self powered */
	0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

	/************** Descriptor of Joystick Mouse interface ****************/
	/* 09 */
	0x09,         /*bLength: Interface Descriptor size*/
	USB_INTERFACE_DESCRIPTOR_TYPE,/*bDescriptorType: Interface descriptor type*/
	0x00,         /*bInterfaceNumber: Number of Interface*/
	0x00,         /*bAlternateSetting: Alternate setting*/
	CONFIG_EP_NUM, /*bNumEndpoints*/				    //FIXME<----------------端点数目
	0x03,         /*bInterfaceClass: HID*/
	0,         /*bInterfaceSubClass : 1=BOOT, 0=no boot 其他数值保留*/
	0,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse 其他数值保留*/
	0,            /*iInterface: Index of string descriptor*/
	/******************** Descriptor of Joystick Mouse HID ********************/
	/* 18 */
	0x09,         /*bLength: HID Descriptor size*/
	HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
	0x10,         /*bcdHID: HID Class Spec release number*/
	0x01,
	0x00,         /*bCountryCode: Hardware target country*/
	0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
	0x22,         /*bDescriptorType*/
	sizeof(Joystick_ReportDescriptor),/*wItemLength: Total length of Report descriptor*/
	0x00,
	/******************** Descriptor of Joystick Mouse endpoint ********************/
	/* 27 */
	0x07,          /*bLength: Endpoint Descriptor size*/
	USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/

	0x81,          /*bEndpointAddress: Endpoint Address (IN)*/
	0x03,          /*bmAttributes: Interrupt endpoint*/
	EP0_PACKET_SIZE, /*wMaxPacketSize: x Byte max */      //<FIXME-------------这里的长度需要跟实际发送的长度一致
	0x00,
	0x0A,          /*bInterval: Polling Interval (32 ms)*/
	/* 34 */

#if (CONFIG_EP_NUM == 2)
	/******************** 输出端点描述符 ********************/
	0x07,          /*bLength: Endpoint Descriptor size*/
	USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/

	0x01,          /*bEndpointAddress: Endpoint Address (OUT)*/
	0x03,          /*bmAttributes: Interrupt endpoint*/
	0x40,          /*wMaxPacketSize: x Byte max */ //FIXME<-------------这里的长度需要跟实际发送的长度一致
	0x00,
	0x01,          /*bInterval: Polling Interval (32 ms)*/
	/* 41 */
#endif
};

const struct descriptor DeviceDesc =
{ Joystick_DeviceDescriptor, sizeof(Joystick_DeviceDescriptor), };
const struct descriptor ConfigDesc =
{ Joystick_ConfigDescriptor, sizeof(Joystick_ConfigDescriptor), };

const u8 Joystick_StringLangID[4] = {
	04,/* Size of Language ID */
	USB_STRING_DESCRIPTOR_TYPE,
	0x09,//0x0409 Aemriacan Engligsh
	0x04
};

const u8 Joystick_StringVendor[] = {
	0,//Joystick_StringVendorSize, /* Size of Vendor string */
	USB_STRING_DESCRIPTOR_TYPE,  /* bDescriptorType*/
	/* Manufacturer: "STMicroelectronics" */
	'U', 0, 'S', 0, 'B', 0, 'k', 0, 'e', 0, 'y', 0
};	//14 bytes

const u8 Joystick_StringProduct[] = {
	0,//Joystick_StringProductSize,          /* bLength */
	USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
	'U', 0, 'S', 0, 'B', 0, 'k', 0, 'e', 0, 'y', 0
};
const u8 Joystick_StringSerial[] = {
	0,//Joystick_StringSerialSize,           /* bLength */
	USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
	'J', 0, 'o', 0, 'y', 0, ' ', 0, ' ', 0, ' ', 0, '1', 0, '.', 0,
	'0', 0, '0', 0, '0', 0, '0', 0
};

//u8 str_desc_buffer[256];
const struct descriptor StringDescTable[] =
{
    {Joystick_StringLangID, sizeof(Joystick_StringLangID), },
    {Joystick_StringProduct, sizeof(Joystick_StringProduct), },
    {Joystick_StringVendor, sizeof(Joystick_StringVendor), },
    {Joystick_StringSerial, sizeof(Joystick_StringSerial), },
};

const unsigned char * str_desc_name_table[]=
{
    "Language",
    "Product",
    "Vendor",
    "Serial",
};
