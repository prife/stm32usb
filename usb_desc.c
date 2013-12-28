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
const u8 Joystick_ReportDescriptor[] = {
    0,
};

/* USB Configuration Descriptor */

#define JOYSTICK_SIZ_CONFIG_DESC (USB_DESC_CONFIG_SIZE    + \
                                  USB_DESC_INTERFACE_SIZE + \
                                  USB_DESC_HID_SIZE       + \
                       USB_DESC_ENDPOINT_SIZE * CONFIG_EP_NUM)

#define JOYSTICK_SIZ_REPORT_DESC  1 //FIXME

/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
const u8 Joystick_ConfigDescriptor[JOYSTICK_SIZ_CONFIG_DESC] = {
	0x09, /* bLength: Configuation Descriptor size */
	USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
	JOYSTICK_SIZ_CONFIG_DESC,
	/* wTotalLength: Bytes returned */
	0x00,
	0x01,         /*bNumInterfaces: 1 interface*/
	0x01,         /*bConfigurationValue: Configuration value*/
	0x01,         /*iConfiguration: Index of string descriptor describing
                                 the configuration*///<------------??
	0x80,         /*bmAttributes: self powered */
	0x19,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

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