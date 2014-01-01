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
	0x02,                       /*bDeviceClass: 0x02=CDC*/
	0x00,                       /*bDeviceSubClass*/
	0x00,                       /*bDeviceProtocol*/
	0x40,                       /*bMaxPacketSize40*/
	0x6E,                       /*idVendor (0x3689)*/   //FIXME<-------------------����ID�ֶ�
	0x09,
	0x04,                       /*idProduct = 0x8762*/  //FIXME<-------------------idProduct
	0x00,
	0x00,                       /*bcdDevice rel. 2.00*/ //FIXME<-------------------���ܹ��汾��
	0x02,
	1,                          /*Index of string descriptor describing
                                              manufacturer */
	2,                          /*Index of string descriptor describing
                                             product*/
	3,                          /*Index of string descriptor describing the
                                             device serial number */
	0x01                        /*bNumConfigurations*/
}; /* DeviceDescriptor */

const struct descriptor ReportDesc[2];

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

	/************** Descriptor of interface0 <CDC class> ****************/
	/* 09 */
	0x09,         /*bLength: Interface Descriptor size*/
	USB_INTERFACE_DESCRIPTOR_TYPE,/*bDescriptorType: Interface descriptor type*/
	0x00,         /*bInterfaceNumber: Number of Interface*/
	0x00,         /*bAlternateSetting: Alternate setting*/
	1,            /*bNumEndpoints*/				    //FIXME<----------------�˵���Ŀ
	0x02,         /*bInterfaceClass: 0x03=HID, 0x02=CDC*/
	2,         /*bInterfaceSubClass  */
               /*         HID class : 1=BOOT, 0=no boot, Other Reserved */
               /*         CDC class : 0x02=Abstract Control Model */
	1,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse ������ֵ����*/
	0,            /*iInterface: Index of string descriptor*/
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
