#include "usb.h"
#include "usb_conf.h"

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef char s8;
typedef short s16;
typedef int s32;

/* USB Standard Device Descriptor */
static const u8 DeviceDescriptor[] = {
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
	0x03,                       /*idProduct = 0x8762*/  //FIXME<-------------------idProduct
	0x00,
	0x00,                       /*bcdDevice rel. 2.00*/ //FIXME<-------------------加密狗版本号
	0x02,
	1,                          /*Index of string descriptor describing
                                              manufacturer */
	2,                          /*Index of string descriptor describing
                                             product*/
	3,                          /*Index of string descriptor describing the
                                             device serial number */
	0x01                        /*bNumConfigurations*/
}; /* DeviceDescriptor */

/* USB Report Descriptor */
static const u8 MouseReportDescriptor[] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,                    // USAGE (Mouse)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x85, 0x01,                    //   REPORT_ID(1)
    0x09, 0x01,                    //   USAGE (Pointer)
    0xa1, 0x00,                    //   COLLECTION (Physical)
    0x05, 0x09,                    //     USAGE_PAGE (Button)
    0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
    0x29, 0x03,                    //     USAGE_MAXIMUM (Button 3)
    0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //     REPORT_SIZE (1)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x81, 0x02,                    //     INPUT (Data,Var,Abs)
    0x75, 0x05,                    //     REPORT_SIZE (5)
    0x95, 0x01,                    //     REPORT_COUNT (1)
    0x81, 0x03,                    //     INPUT (Cnst,Var,Abs)
    0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
    0x09, 0x30,                    //     USAGE (X)
    0x09, 0x31,                    //     USAGE (Y)
    0x09, 0x38,                    //     USAGE (Wheel)
    0x15, 0x81,                    //     LOGICAL_MINIMUM (-127)
    0x25, 0x7f,                    //     LOGICAL_MAXIMUM (127)
    0x75, 0x08,                    //     REPORT_SIZE (8)
    0x95, 0x03,                    //     REPORT_COUNT (3)
    0x81, 0x06,                    //     INPUT (Data,Var,Rel)
    0xc0,                          //   END_COLLECTION
    0xc0                           // END_COLLECTION
};

static const u8 KeyboardReportDescriptor[] =
{
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x85, 0x01,                    //   REPORT_ID(1)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)
    0x95, 0x06,                    //   REPORT_COUNT (6)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
}; /*  KeyboardReportDescriptor */

const struct descriptor ReportDesc[2] =
{ 
    KeyboardReportDescriptor, sizeof(KeyboardReportDescriptor),
    MouseReportDescriptor, sizeof(MouseReportDescriptor),
};


/* USB Configuration Descriptor */

#define CONFIG_DESC_SIZE         (USB_DESC_CONFIG_SIZE        + \
                                  USB_DESC_INTERFACE_SIZE * 2 + \
                                  USB_DESC_HID_SIZE * 2       + \
                       USB_DESC_ENDPOINT_SIZE * CONFIG_EP_NUM)

/*   All Descriptors (Configuration, Interface, Endpoint, Class, Vendor */
const u8 ConfigDescriptor[CONFIG_DESC_SIZE] = {
	0x09, /* bLength: Configuation Descriptor size */
	USB_CONFIGURATION_DESCRIPTOR_TYPE, /* bDescriptorType: Configuration */
	CONFIG_DESC_SIZE,
	/* wTotalLength: Bytes returned */
	0x00,
	0x02,         /*bNumInterfaces: 1 interface*/
	0x01,         /*bConfigurationValue: Configuration value*/
	0x00,         /*iConfiguration: Index of string descriptor describing
                                 the configuration*///<------------??
	0x80,         /*bmAttributes: self powered */
	0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

	/************** Descriptor of  keyboard interface ****************/
	/* 09 */
	0x09,         /*bLength: Interface Descriptor size*/
	USB_INTERFACE_DESCRIPTOR_TYPE,/*bDescriptorType: Interface descriptor type*/
	0x00,         /*bInterfaceNumber: Number of Interface*/
	0x00,         /*bAlternateSetting: Alternate setting*/
	2,            /*bNumEndpoints*/				    //FIXME<----------------端点数目
	0x03,         /*bInterfaceClass: HID*/
	1,         /*bInterfaceSubClass : 1=BOOT, 0=no boot 其他数值保留*/
	1,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse 其他数值保留*/
	0,            /*iInterface: Index of string descriptor*/
	/******************** Descriptor of  keyboard HID ********************/
	/* 18 */
	0x09,         /*bLength: HID Descriptor size*/
	HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
	0x10,         /*bcdHID: HID Class Spec release number*/
	0x01,
	0x21,         /*bCountryCode: Hardware target country*/
	0x01,         /*bNumDescriptors: Number of HID class descriptors to follow*/
	0x22,         /*bDescriptorType*/
	sizeof(KeyboardReportDescriptor),/*wItemLength: Total length of Report descriptor*/
	0x00,
	/******************** Descriptor of  keyboard endpoint ********************/
	/* 27 */
	0x07,          /*bLength: Endpoint Descriptor size*/
	USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/
	0x81,          /*bEndpointAddress: Endpoint Address (IN)*/
	0x03,          /*bmAttributes: Interrupt endpoint*/
	EP0_PACKET_SIZE, /*wMaxPacketSize: x Byte max */      //<FIXME-------------这里的长度需要跟实际发送的长度一致
	0x00,
	0x0A,          /*bInterval: Polling Interval (32 ms)*/
	/* 34 */

	/******************** 输出端点描述符 ********************/
	0x07,          /*bLength: Endpoint Descriptor size*/
	USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/
	0x01,          /*bEndpointAddress: Endpoint Address (OUT)*/
	0x03,          /*bmAttributes: Interrupt endpoint*/
	0x40,          /*wMaxPacketSize: x Byte max */ //FIXME<-------------这里的长度需要跟实际发送的长度一致
	0x00,
	0x0A,          /*bInterval: Polling Interval (32 ms)*/
	/* 41 */

    /************** Descriptor of  Mouse interface ****************/
    0x09, /*bLength: Interface Descriptor size*/
    USB_INTERFACE_DESCRIPTOR_TYPE,/*bDescriptorType: Interface descriptor type*/
    0x01, /*bInterfaceNumber: Number of Interface*/
    0x00, /*bAlternateSetting: Alternate setting*/
    1,    /*bNumEndpoints*/                                 //FIXME<----------------端点数目
    0x03, /*bInterfaceClass: HID*/
    0, /*bInterfaceSubClass : 1=BOOT, 0=no boot 其他数值保留*/
    2, /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse 其他数值保留*/
    0, /*iInterface: Index of string descriptor*/
    /******************** Descriptor of  Mouse HID ********************/
    0x09, /*bLength: HID Descriptor size*/
    HID_DESCRIPTOR_TYPE, /*bDescriptorType: HID*/
    0x10, /*bcdHID: HID Class Spec release number*/
    0x01,
    0x21, /*bCountryCode: Hardware target country*/
    0x01, /*bNumDescriptors: Number of HID class descriptors to follow*/
    0x22, /*bDescriptorType*/
    sizeof(MouseReportDescriptor),/*wItemLength: Total length of Report descriptor*/
    0x00,
    /******************** Descriptor of  Mouse endpoint ********************/
    0x07, /*bLength: Endpoint Descriptor size*/
    USB_ENDPOINT_DESCRIPTOR_TYPE, /*bDescriptorType:*/
    0x82, /*bEndpointAddress: Endpoint Address (IN)*/
    0x03, /*bmAttributes: Interrupt endpoint*/
    EP0_PACKET_SIZE, /*wMaxPacketSize: x Byte max */
    0x00,
    0x0A, /*bInterval: Polling Interval (32 ms)*/
};

const struct descriptor DeviceDesc =
{ DeviceDescriptor, sizeof(DeviceDescriptor), };
const struct descriptor ConfigDesc =
{ ConfigDescriptor, sizeof(ConfigDescriptor), };

static const u8 StringLangID[4] = {
	04,/* Size of Language ID */
	USB_STRING_DESCRIPTOR_TYPE,
	0x09,//0x0409 Aemriacan Engligsh
	0x04
};

static const u8 StringVendor[] = {
	0,//_StringVendorSize, /* Size of Vendor string */
	USB_STRING_DESCRIPTOR_TYPE,  /* bDescriptorType*/
	/* Manufacturer: "STMicroelectronics" */
	'U', 0, 'S', 0, 'B', 0, 'k', 0, 'e', 0, 'y', 0
};	//14 bytes

static const u8 StringProduct[] = {
	0,//StringProductSize,          /* bLength */
	USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
	'U', 0, 'S', 0, 'B', 0, 'k', 0, 'e', 0, 'y', 0
};
static const u8 StringSerial[] = {
	0,//StringSerialSize,           /* bLength */
	USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'J', 0, 'o', 0, 'y', 0, 'S', 0, 't', 0, 'i', 0, 'c', 0,
    'k', 0, 'M', 0, 'o', 0, 'u', 0, 's', 0, 'e'
};

//u8 str_desc_buffer[256];
const struct descriptor StringDescTable[] =
{
    {StringLangID, sizeof(StringLangID), },
    {StringVendor, sizeof(StringVendor), },
    {StringProduct, sizeof(StringProduct), },
    {StringSerial, sizeof(StringSerial), },
};

const unsigned char * str_desc_name_table[]=
{
    "Language",
    "Vendor",
    "Product",
    "Serial",
};
