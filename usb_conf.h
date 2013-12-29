#ifndef USB_H
#define USB_H

#define USB_LOG

#ifdef USB_LOG
#define TRACE  rt_kprintf
#define DUMPHEX dumphex
#else
#define TRACE(...)
#define DUMPHEX(...)
#endif

#define USB_CNTR_MASK  \
    (CNTR_CTRM  | CNTR_WKUPM \
     | CNTR_ERRM  | CNTR_RESETM)

#define _ClearISTR(bit) _SetISTR((uint16_t)(~(bit)))

enum usb_cable_type
{
    USB_CABLE_CONNECT = 0,
    USB_CABLE_DISCONNECT = !USB_CABLE_CONNECT,
};

#define CONFIG_EP_NUM   1
#define EP0_PACKET_SIZE 64
#endif
