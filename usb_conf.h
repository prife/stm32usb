#ifndef USB_H
#define USB_H

#define TRACE  rt_kprintf

#define USB_CNTR_MASK  \
    (CNTR_CTRM  | CNTR_WKUPM | \
     CNTR_ERRM  | CNTR_RESETM)

#define _ClearISTR(bit) _SetISTR((uint16_t)(~(bit)))

enum usb_cable_type
{
    USB_CABLE_CONNECT = 0,
    USB_CABLE_DISCONNECT = !USB_CABLE_CONNECT,
};

#include <usb_regs.h>
#endif
