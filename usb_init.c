#define USE_STDPERIPH_DRIVER
#include <rtthread.h>
#include <stm32f10x.h>
#include <usb_conf.h>

#define USB_DISCONNECT                      GPIOF
#define USB_DISCONNECT_PIN                  GPIO_Pin_11
#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOF

void usb_isr_init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the USB interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void usb_clock_init(void)
{
  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

void usb_cablepin_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable USB_DISCONNECT GPIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);

  /* Configure USB pull-up pin */
  GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
}

void usb_cable_config(enum usb_cable_type state)
{
    if (state == USB_CABLE_CONNECT)
        GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
    else
        GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
}

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

#define ENDP0_RXADDR        (0x18)
#define ENDP0_TXADDR        (0x58)

void usb_power_on()
{
    /* reset CNTR */
    _SetCNTR(CNTR_FRES);//force reset
    _SetCNTR(0);//clear reset bit
    _SetISTR(0);//clear pending isrs

    /* enable some interrupts */
    _SetCNTR(USB_CNTR_MASK);
}

void usb_init()
{
    usb_isr_init();
    usb_clock_init();
    usb_cablepin_init();
    usb_cable_config(USB_CABLE_CONNECT);
    usb_power_on();
}

/**
 * nr: endpoint id
 * flag: Enable/Disable
 */
int usb_endpoint_config()
{
#if 0
    _SetEPType(ENDP0, EP_CONTROL);
    _SetEPTxStatus(ENDP0, EP_TX_STALL);
    _SetEPRxAddr(ENDP0, ENDP0_RXADDR);
    _SetEPTxAddr(ENDP0, ENDP0_TXADDR);
    _ClearEP_KIND(ENDP0);
    _SetEPRxCount(ENDP0, 64);
    _SetEPRxStatus(ENDP0, EP_RX_VALID);

    _SetEPAddress(0, 0);
#else
    *EPREG_RXBUF_ADDR(ENDP0) = ENDP0_RXADDR;
    *EPREG_RXBUF_SIZE(ENDP0) = ((64>>5)<<10) | 0x8000;

    // more bit need set
    _SetENDPOINT(0, EP_RX_VALID | EP_CONTROL);
#endif
    /* enable USB endpoint and config EP0 */
    _SetDADDR(DADDR_EF | 0);
}

struct ep_buf
{
    u32 * ptr;
    u32 len;
};

void dumphex(u32 * buf, u16 bytes)
{
    int i, count;
    int value;

    count = bytes/4 + ((bytes%4)?1:0);
    rt_kprintf("bytes:%d, count:%d\n", bytes, count);
    for (i=0; i<count; i++)
    {
        value = buf[i];
#if 1
        rt_kprintf("%02x %02x %02x %02x ",(char)value, (char)(value>>8),
                        (char)(value>>16), (char)(value>>24));
#else
        rt_kprintf("%02x %02x ",(char)value, (char)(value>>8));
#endif
        if ((i+1)%4 == 0)
            rt_kprintf("\n");
    }
    rt_kprintf("\n");
}

void get_endpoint_buf(int ep_nr, struct ep_buf *ep)
{
    ep->ptr = (u32*)(((u16)*EPREG_RXBUF_ADDR(ep_nr)) * 2 + PMA_ADDR);
    ep->len = (u16)(*EPREG_RXBUF_SIZE(ep_nr)) & 0x3FF;

    TRACE("EPREG_RXBUF_ADDR:%x\n", *EPREG_RXBUF_ADDR(ep_nr));
    TRACE("EPREG_RXBUF_SIZE:%x\n", *EPREG_RXBUF_SIZE(ep_nr));
    TRACE("ptr:%p, len:%d\n", ep->ptr, ep->len);
    dumphex(ep->ptr, ep->len);
    //dumphex((u32*)PMA_ADDR, 256);
}

#define TAG "USB ISR:"
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    static __IO uint16_t istr;
    istr = _GetISTR();
    if (istr & ISTR_CTR)
    {
        static struct ep_buf ep;
        //int nr;
        //int dir;
        TRACE(TAG "ctr:\n");//Read only?
        TRACE((istr & ISTR_DIR) ? "OUT":"IN");
        TRACE(":%d\n",istr & ISTR_EP_ID);

        if (istr & ISTR_DIR)
            get_endpoint_buf(istr & ISTR_EP_ID, &ep);

        //clear EndPoint RX interrupt flag
        _SetENDPOINT((istr & ISTR_EP_ID), ~EP_CTR_RX);
    }

    if (istr & ISTR_DOVR)
    {
        TRACE(TAG "DOver\n");
        _ClearISTR(ISTR_DOVR);
    }

    if (istr & ISTR_RESET)
    {
        TRACE(TAG "reset\n");
        _ClearISTR(ISTR_RESET);
        usb_endpoint_config();
    }

    if (istr & ISTR_ERR)
    {
        TRACE(TAG "ERR\n");
        _ClearISTR(ISTR_ERR);
    }

    if (istr & ISTR_WKUP)
    {
        TRACE(TAG "WKUP\n");
        _ClearISTR(ISTR_WKUP);
    }

    if (istr & ISTR_SUSP)
    {
        TRACE(TAG "SUSP\n");
        _ClearISTR(ISTR_SUSP);
    }

    if (istr & ISTR_SOF)
    {
        TRACE(TAG "SOF\n");
        _ClearISTR(ISTR_SOF);
    }

    if (istr & ISTR_ESOF)
    {
        TRACE(TAG "ESOF\n");
        _ClearISTR(ISTR_ESOF);
    }
}

__asm void SystemReset(void)
{
    /* http://hi.baidu.com/imapla/item/6ad07fdf995627ea3dc2cbe8 */
    MOV R0, #1           //;
    MSR FAULTMASK, R0    //; 清除FAULTMASK 禁止一切中断产生
    LDR R0, =0xE000ED0C  //;
    LDR R1, =0x05FA0004  //;
    STR R1, [R0]         //; 系统软件复位
deadloop
    B deadloop           //; 死循环
}

#include <finsh.h>
FINSH_FUNCTION_EXPORT(usb_init, usb init)
FINSH_FUNCTION_EXPORT_ALIAS(SystemReset, reset, reset system);
FINSH_FUNCTION_EXPORT(dumphex, dump hex)