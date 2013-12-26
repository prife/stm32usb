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

void usb_plugin()
{
    GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
}

void usb_plugout()
{
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
    char buffer[1024];
    u32 len;
};

void dumphex(u8 * buf, u16 count)
{
    int i;

    for (i=0; i<count; i++)
    {
        rt_kprintf("%02x ", buf[i]);
        if ((i+1)%16 == 0)
            rt_kprintf("\n");
    }
    if ((i+1)%16 != 0)
        rt_kprintf("\n");
}

void get_endpoint_buf(int ep_nr, struct ep_buf *ep)
{
    int i;
    u32 * ptr = (u32*)(((u16)*EPREG_RXBUF_ADDR(ep_nr)) * 2 + PMA_ADDR);
    ep->len = (u16)(*EPREG_RXBUF_SIZE(ep_nr)) & 0x3FF;

    //TRACE("EPREG_RXBUF_ADDR:%x\n", *EPREG_RXBUF_ADDR(ep_nr));
    //TRACE("EPREG_RXBUF_SIZE:%x\n", *EPREG_RXBUF_SIZE(ep_nr));
    //TRACE("ptr:%p, len:%d\n", ptr, ep->len);

    //dumphex(ptr, ep->len);
    for (i=0; i<ep->len; i+=2)
    {
        *(rt_uint16_t *)(&ep->buffer[i]) = (*ptr++) & 0xFFFF;
    }
    //dumphex((u32*)PMA_ADDR, 256);
}

struct std_device_reqeust
{
    rt_uint8_t bmRequestType;
            //D7[Dir] : 0 HOST->Device
            //          1 HOST<-Device
            //D6-D5
            //        :00 stadard
            //        :01 class
            //        :10 vendor
            //        :11 reserved
            //D4-D0
            //     :00000 Device
            //     :00001 interface
            //     :00010 endpoint
            //     :00011 other
            //     :xxx00 reserved
    rt_uint8_t bRequest;
    rt_uint16_t rt_wValue;
    rt_uint16_t wIndex;
    rt_uint16_t wLength;
};

enum std_device_reqeust_type
{
    GET_STATUS = 0,
    CLEAR_FEATURE = 1,
    SET_FEATURE = 3,
    SET_ADDRESS = 5,
    GET_DESCRIPTOR = 6,
    SET_DESCRIPTOR = 7,
    GET_CONFIGURATION = 8,
    SET_CONFIGURATION = 9,
    GET_INTERFACE = 10,
    SET_INTERFACE = 11,
    SYNCH_FRAME = 12,
};

short net2host_16bit(char * buffer)
{
    return ((((short)buffer[1]) << 8) | buffer[0]);
}

int net2host_32bit(char * buffer)
{
    return ((((int)buffer[3]) << 24) |
            (((int)buffer[2]) << 16) |
            (((int)buffer[1]) << 8)  |
            (((int)buffer[0])));
}

int handle_packet_setup(struct ep_buf *ep)
{
#define REQUEST_TYPE          ((1<<6)|(1<<5))
#define REQUEST_TYPE_STDARD   0
#define REQUEST_TYPE_CLASS    ((0<<6)|(1<<5))
#define REQUEST_TYPE_VENDOR   ((1<<6)|(0<<5))
#define REQUEST_TYPE_RESERVED ((1<<6)|(1<<5))

    rt_uint8_t bmRequestType;
    rt_uint8_t bRequest;
    rt_uint16_t wValue;
    rt_uint16_t wIndex;
    rt_uint16_t wLength;

    bmRequestType = ep->buffer[0];
    bRequest = ep->buffer[1];
    wValue = net2host_16bit(&ep->buffer[2]);
    wIndex = net2host_16bit(&ep->buffer[4]);
    wLength = net2host_16bit(&ep->buffer[6]);

    dumphex(ep->buffer, ep->len);
    if (bmRequestType & 0x80)
    {
        //OUT
        switch (bmRequestType & REQUEST_TYPE)
        {
        case REQUEST_TYPE_STDARD:
            TRACE("USB input stdard reqeust:");
            switch(bRequest)
            {
            case GET_STATUS :
                TRACE("get_status\n");
                break;
            case CLEAR_FEATURE :
                TRACE("clear_feature\n");
                break;
            case SET_FEATURE :
                TRACE("set_feature\n");
                break;
            case SET_ADDRESS :
                TRACE("set_address\n");
                break;
            case GET_DESCRIPTOR :
                TRACE("get_descriptor\n");
                break;
            case SET_DESCRIPTOR :
                TRACE("set_descriptor\n");
                break;
            case GET_CONFIGURATION :
                TRACE("get_configuration\n");
                break;
            case SET_CONFIGURATION :
                TRACE("set_configration\n");
                break;
            case GET_INTERFACE :
                TRACE("get_interface\n");
                break;
            case SET_INTERFACE :
                TRACE("set_interface\n");
                break;
            case SYNCH_FRAME :
                TRACE("synch_frame\n");
                break;
            default:
                TRACE("unkown!\n");
                break;
            }
            break;
        case REQUEST_TYPE_CLASS:
            TRACE("USB input Class qeuset\n");
            break;
        case REQUEST_TYPE_VENDOR:
            TRACE("USB input Vendor qeuset\n");
            break;
        case REQUEST_TYPE_RESERVED:
            TRACE("USB input Reserved qeuset\n");
            break;
        }

    }
    else
    {
        //IN
        switch (bmRequestType & REQUEST_TYPE)
        {
        case REQUEST_TYPE_STDARD:
            TRACE("USB output stadard qeuset\n");
            switch(bRequest)
            {
            case GET_STATUS :
                TRACE("get_status\n");
                break;
            case CLEAR_FEATURE :
                TRACE("clear_feature\n");
                break;
            case SET_FEATURE :
                TRACE("set_feature\n");
                break;
            case SET_ADDRESS :
                TRACE("set_address\n");
                break;
            case GET_DESCRIPTOR :
                TRACE("get_descriptor\n");
                break;
            case SET_DESCRIPTOR :
                TRACE("set_descriptor\n");
                break;
            case GET_CONFIGURATION :
                TRACE("get_configuration\n");
                break;
            case SET_CONFIGURATION :
                TRACE("set_configration\n");
                break;
            case GET_INTERFACE :
                TRACE("get_interface\n");
                break;
            case SET_INTERFACE :
                TRACE("set_interface\n");
                break;
            case SYNCH_FRAME :
                TRACE("synch_frame\n");
                break;
            default:
                TRACE("unkown!\n");
                break;
            }
        case REQUEST_TYPE_CLASS:
            TRACE("USB output Class qeuset\n");
            break;
        case REQUEST_TYPE_VENDOR:
            TRACE("USB output Vendor qeuset\n");
            break;
        case REQUEST_TYPE_RESERVED:
            TRACE("USB output Reserved qeuset\n");
            break;
        }
    }

    return 0;
}

int handle_packet_out(struct ep_buf *ep)
{
    dumphex(ep->buffer, ep->len);
    //TODO

    return 0;
}

int handle_packet_in(struct ep_buf *ep)
{
    dumphex(ep->buffer, ep->len);
    //TODO

    return 0;
}

#define TAG "USB ISR:"
static struct ep_buf ep;
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    static __IO uint16_t istr;
    istr = _GetISTR();
    if (istr & ISTR_CTR)
    {
        int ep_id;
        //int dir;
        TRACE((istr & ISTR_DIR) ? "OUT":"IN");
        ep_id = istr & ISTR_EP_ID;
        TRACE("endpoint:%d ctr\n", ep_id);

        if (istr & ISTR_DIR) //OUT
        {
            int ep_value;
            ep_value = _GetENDPOINT(ep_id);
            get_endpoint_buf(ep_id, &ep);
            if (ep_value & (EP_CTR_RX | EP_SETUP))
            { //SETUP
                if (ep_id == 0)
                {
                    TRACE("%d \t SETUP\n", __LINE__);
                    handle_packet_setup(&ep);
                }
                else
                {
                    TRACE("%d \t error! setup should only to endpoint 0\n", __LINE__);
                }
            }
            else
            { //OUT
                handle_packet_out(&ep);
            }
        }
        else //IN
        {
            int ep_value;
            ep_value = _GetENDPOINT(ep_id);
            if (ep_value & (EP_CTR_RX | EP_SETUP))
            {
                //should never be here! I guess
                TRACE("\t:%d SETUP\n", __LINE__);
                handle_packet_setup(&ep);
            }
            else
            {
                handle_packet_in(&ep);
            }
        }

        //clear EndPoint RX interrupt flag
        _SetENDPOINT(ep_id, ~EP_CTR_RX);
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

void reset()
{
    usb_plugout();
    SystemReset();
}

#include <finsh.h>
FINSH_FUNCTION_EXPORT(usb_init, usb init)
FINSH_FUNCTION_EXPORT(reset, reset system)
FINSH_FUNCTION_EXPORT(dumphex, dump hex)
