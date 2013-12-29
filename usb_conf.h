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

#define USB_DISCONNECT                      GPIOF
#define USB_DISCONNECT_PIN                  GPIO_Pin_11
#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOF

#define CONFIG_EP_NUM   1
#define EP0_PACKET_SIZE 64

/* Packet Memroy */
#define PMA_ADDR 0x40006000L
#if 1
#define EPREG_TXBUF_ADDR(nr)  ((u32*)(((_GetBTABLE() + (nr << 3) + 0) << 1) + PMA_ADDR))
#define EPREG_TXBUF_SIZE(nr)  ((u32*)(((_GetBTABLE() + (nr << 3) + 2) << 1) + PMA_ADDR))
#define EPREG_RXBUF_ADDR(nr)  ((u32*)(((_GetBTABLE() + (nr << 3) + 4) << 1) + PMA_ADDR))
#define EPREG_RXBUF_SIZE(nr)  ((u32*)(((_GetBTABLE() + (nr << 3) + 6) << 1) + PMA_ADDR))
#else
#define GET_EP_TXBUF_ADDR(nr)  ((_GetBTABLE() + (nr << 4) + 0))
#define GET_EP_TXBUF_SIZE(nr)  ((_GetBTABLE() + (nr << 4) + 4))
#define GET_EP_RXBUF_ADDR(nr)  ((_GetBTABLE() + (nr << 4) + 8))
#define GET_EP_RXBUF_SIZE(nr)  ((_GetBTABLE() + (nr << 4) + 12))
#endif

/* EP0  */
/* rx/tx buffer base address */
#define ENDP0_RXADDR        (0x18)
#define ENDP0_TXADDR        (0x58)

/* EP1  */
/* tx buffer base address */
#define ENDP1_TXADDR        (0x100)

#endif
