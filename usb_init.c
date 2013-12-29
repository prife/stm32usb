#include <rtthread.h>
#include <stm32f10x.h>
#include <string.h>
#include "usb_conf.h"
#include "usb.h"
#include <usb_regs.h>

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif

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

enum descriptor_type
{
    DESC_DEVICE = 1,
    DESC_CONFIGURATION = 2,
    DESC_STRING = 3,
    DESC_INTERFACE = 4,
    DESC_ENDPOINT = 5,
    DESC_REPORT= 0x22
};

struct ep_buf
{
    u8 buffer[1024];
    const u8 * send_buffer;
    u32 len;
};

//for send data more than the max size of the endpoint
struct usb_send_data
{
    int ep; //endpoint
    int total;
    int sent; //alread send bytes
    const u8 * buf;
};
struct usb_send_data send_status;

//for set address
static int set_address_flag;
static int gAddress;

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

void usb_power_on()
{
    /* reset CNTR */
    _SetCNTR(CNTR_FRES);//force reset
    _SetCNTR(0);//clear reset bit
    _SetISTR(0);//clear pending isrs

    /* enable some interrupts */
    _SetCNTR(USB_CNTR_MASK);
}

void rt_mouse_thread_entry(void * para)
{
    extern unsigned char JoyState(void);
    extern void Joystick_Send(unsigned char Keys);
    unsigned char key;
    while(1)
    {
        //rt_thread_delay(RT_TICK_PER_SECOND / 100);
        if ((key = JoyState()) != 0)
            Joystick_Send(key);
    }
}

void usb_init()
{
    usb_isr_init();
    usb_clock_init();
    usb_cablepin_init();
    usb_cable_config(USB_CABLE_CONNECT);
    usb_power_on();
    /* ADD your init code here */
    {
        rt_thread_t thread;
        extern void Joystick_gpio_init(void);
        Joystick_gpio_init();
        thread = rt_thread_create("init",
                                   rt_mouse_thread_entry, RT_NULL,
                                   512, 30, 20);
        RT_ASSERT(thread != NULL);
        rt_thread_startup(thread);
    }
}

void usb_ep_config()
{
    /* enable some interrupts */
    _SetCNTR(USB_CNTR_MASK);

    /* Initialize Endpoint 0 */
    _SetEPType(ENDP0, EP_CONTROL);
    _SetEPTxStatus(ENDP0, EP_TX_STALL);
    _SetEPRxAddr(ENDP0, ENDP0_RXADDR);
    _SetEPTxAddr(ENDP0, ENDP0_TXADDR);
    _ClearEP_KIND(ENDP0);
    _SetEPRxCount(ENDP0, EP0_PACKET_SIZE );
    _SetEPRxStatus(ENDP0, EP_RX_VALID);

    /* Initialize Endpoint 1 */
    _SetEPType(ENDP1, EP_INTERRUPT);
    _SetEPTxAddr(ENDP1, ENDP1_TXADDR);
    _SetEPTxCount(ENDP1, 4);
    _SetEPRxStatus(ENDP1, EP_RX_DIS);
    _SetEPTxStatus(ENDP1, EP_TX_NAK);

    _SetEPAddress(0, 0);
    /* enable USB endpoint and config EP0 */
    _SetDADDR(DADDR_EF | 0);
}

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

void ep_get(int ep_nr, struct ep_buf *ep)
{
    int i;
    u32 * ptr = (u32*)(((u16)*EPREG_RXBUF_ADDR(ep_nr)) * 2 + PMA_ADDR);
    ep->len = (u16)(*EPREG_RXBUF_SIZE(ep_nr)) & 0x3FF;

    for (i=0; i<ep->len; i+=2)
    {
        *(rt_uint16_t *)(&ep->buffer[i]) = (*ptr++) & 0xFFFF;
    }
}

void ep_get2(int ep_nr, struct ep_buf *ep, int len)
{
    int i;
    u32 * ptr = (u32*)(((u16)*EPREG_RXBUF_ADDR(ep_nr)) * 2 + PMA_ADDR);

    for (i=0; i<len; i+=2)
    {
        *(rt_uint16_t *)(&ep->buffer[i]) = (*ptr++) & 0xFFFF;
    }
}

void ep_send(int ep_nr, const u8 * buf, int len)
{
    int i;
    u32 * ptr = (u32*)(((u16)*EPREG_TXBUF_ADDR(ep_nr)) * 2 + PMA_ADDR);

    TRACE("send ep<%d>: ptr %p, len %d\n", ep_nr, ptr, len);
    DUMPHEX((u8*)buf, len);
    for (i=0; i<len; i+=2)
    {
        (*ptr++) = ((rt_uint16_t)buf[i+1] << 8) | buf[i];
    }
    if (len & 0x1)
        (*ptr++) = buf[i];

    _SetEPTxCount(ep_nr, len);
	_SetEPTxStatus(ep_nr, EP_TX_VALID);
}
//enable ep N valid
void enable_endpoint_rx(int ep_nr, int count)
{
    _SetEPRxCount(ep_nr, count);
	_SetEPRxStatus(ep_nr, EP_RX_VALID);
}

short net2host_16bit(u8 * buffer)
{
    return ((((short)buffer[1]) << 8) | buffer[0]);
}

int net2host_32bit(u8 * buffer)
{
    return ((((int)buffer[3]) << 24) |
            (((int)buffer[2]) << 16) |
            (((int)buffer[1]) << 8)  |
            (((int)buffer[0])));
}

void set_address(int address)
{
    //USB_DAADR
    _SetEPAddress(0, 0);
    _SetEPAddress(1, 1);//FIXME: is this usable?
    _SetDADDR((address & 0x7F) | DADDR_EF);
}

/* config: ? TODO */
void set_configration(int config)
{
    if (config)
    {
        //TODO: add what?
    }
    //just response an 0-Byte DATA packet
    ep_send(0, NULL, 0);
}

int handle_packet_setup(struct ep_buf *ep)
{
#define REQUEST_TYPE          ((1<<6)|(1<<5))
#define REQUEST_TYPE_STDARD   0
#define REQUEST_TYPE_CLASS    ((0<<6)|(1<<5))
#define REQUEST_TYPE_VENDOR   ((1<<6)|(0<<5))
#define REQUEST_TYPE_RESERVED ((1<<6)|(1<<5))
    int send_len;
    //<----------------------
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

    DUMPHEX(ep->buffer, ep->len);
    if (bmRequestType & 0x80)
    {
        //IN
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
            {
                TRACE("get_descriptor: ");
                switch (wValue >> 8) // descriptor type;
                {
                case DESC_DEVICE:
                {
                    TRACE("device_desc\n");
                    RT_ASSERT(send_len <= 64); //FIXME
                    ep_send(0, DeviceDesc.desc, DeviceDesc.len);
                    break;
                }
                case DESC_CONFIGURATION:
                    TRACE("config_desc\n");
                    send_len = MIN(wLength, ConfigDesc.len);
                    RT_ASSERT(send_len <= 64); //FIXME
                    ep_send(0, ConfigDesc.desc, send_len);
                    break;
                case DESC_STRING:
                    TRACE("string_desc:%d-->",(wValue & 0xFF));
                    if ((wValue & 0xFF) < 4)
                    {
                        static u8 str_buf[256];
                        static const struct descriptor * p;
                        TRACE("%s\n", str_desc_name_table[wValue & 0xFF]);
                        p = &StringDescTable[(wValue & 0xFF)];
                        memcpy(str_buf, p->desc, p->len);
                        str_buf[0] = p->len;
                        RT_ASSERT(p->len <= 64);
                        ep_send(0, str_buf, p->len);
                    }
                    else
                    {
                        TRACE("-->bad argument!\n");
                    }
                    break;
                case DESC_INTERFACE:
                    TRACE("interface_desc\n");
                    break;
                case DESC_ENDPOINT:
                    TRACE("endpoint_desc\n");
                    break;
                case DESC_REPORT:
                    TRACE("report_desc\n");
                    //FIXME: refine code
                    send_status.ep = 0;
                    send_status.total = ReportDesc.len;
                    send_len = MIN(ReportDesc.len, EP0_PACKET_SIZE);
                    ep_send(0, ReportDesc.desc, send_len);
                    send_status.sent = send_len;
                    send_status.buf = ReportDesc.desc;
                    break;
                default:
                    TRACE("<%d> unknown_desc!\n", wValue >> 8);
                    break;
                }
                break;
            }
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
            TRACE("USB input Class qeuset:");
            switch(bRequest)
            {
            case GET_REPORT:
                TRACE("GET_REPORT\n");
                break;
            case GET_IDLE:
                TRACE("GET_IDLE\n");
                break;
            case GET_PROTOCOL:
                TRACE("GET_PROTOCOL\n");
                break;
            case SET_REPORT:
                TRACE("SET_REPORT\n");
                break;
            case SET_IDLE:
                TRACE("SET_IDLE\n");
                break;
            case SET_PROTOCOL:
                TRACE("SET_PROTOCOL\n");
                break;
            default:
                TRACE("bad request!\n");
                break;
            }
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
        //OUT
        switch (bmRequestType & REQUEST_TYPE)
        {
        case REQUEST_TYPE_STDARD:
            TRACE("USB output stadard qeuset: ");
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
                TRACE("new address:0x%x\n", wValue);
                RT_ASSERT(wLength == 0);
                //set_address(wValue);
                set_address_flag = 1;
                gAddress = wValue;
                ep_send(0, RT_NULL, wLength);
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
                TRACE("set_configration<%d>\n", wValue & 0xFF);
                //(wValue & 0xFF) is the config index
                set_configration(wValue & 0xFF);
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
            TRACE("USB output Class qeuset:");
            switch(bRequest)
            {
            case GET_REPORT:
                TRACE("GET_REPORT\n");
                break;
            case GET_IDLE:
                TRACE("GET_IDLE\n");
                break;
            case GET_PROTOCOL:
                TRACE("GET_PROTOCOL\n");
                break;
            case SET_REPORT:
                TRACE("SET_REPORT\n");
                break;
            case SET_IDLE:
                TRACE("SET_IDLE\n");
                ep_send(0, NULL, 0);
                break;
            case SET_PROTOCOL:
                TRACE("SET_PROTOCOL\n");
                break;
            default:
                TRACE("bad request!\n");
                break;
            }
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
    //TODO
    return 0;
}

int handle_packet_in(struct ep_buf *ep)
{
    //TODO

    return 0;
}

static struct ep_buf ep;
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    static __IO uint16_t istr;
    istr = _GetISTR();
    if (istr & USB_CNTR_MASK & ISTR_RESET)
    {
        _ClearISTR(ISTR_RESET); //do not go into CLR
        usb_ep_config();
        TRACE("\n####### USB 复位中断 reset ######\n");
    }

    if (istr & USB_CNTR_MASK & ISTR_ERR)
    {
        _ClearISTR(ISTR_ERR);
        TRACE("####### USB 总线错误中断 ERR ######\n");
    }

    if (istr & USB_CNTR_MASK & ISTR_WKUP)
    {
        _ClearISTR(ISTR_WKUP);
        TRACE("####### USB 唤醒中断WKUP #######\n");
    }

    if (istr & USB_CNTR_MASK & ISTR_SUSP)
    {
        _ClearISTR(ISTR_SUSP);
        TRACE(" ####### USB 挂起中断SUSP #######\n");
    }

    if (istr & USB_CNTR_MASK & ISTR_SOF)
    {
        _ClearISTR(ISTR_SOF);
        TRACE("####### USB 帧起始中断 SOF #######\n");
    }

    if (istr & USB_CNTR_MASK & ISTR_ESOF)
    {
        _ClearISTR(ISTR_ESOF);
        TRACE("####### USB 帧起始中断ESOF #######\n");
    }

    if (istr & USB_CNTR_MASK & ISTR_DOVR)
    {
        _ClearISTR(ISTR_DOVR);
        TRACE("####### USB DMA溢出中断 DOver #######\n");
    }

    if (istr & ISTR_CTR)
    {
        int ep_id;
        ep_id = istr & ISTR_EP_ID;
        _SetEPRxStatus(ep_id, EP_RX_NAK);
        _SetEPTxStatus(ep_id, EP_TX_NAK);
        if (istr & ISTR_DIR) //OUT
        {
            int ep_value;
            ep_value = _GetENDPOINT(ep_id);
            ep_get(ep_id, &ep);

            //clear EndPoint RX interrupt flag
            _ClearEP_CTR_RX(ep_id);
            if (ep_value & EP_SETUP)
            { //SETUP
                TRACE("[%4d] USB端点<%d> SETUP packet, reg<0x%x>, len:<%d>\n", __LINE__, ep_id, ep_value, ep.len);
                if (ep_id == 0)
                {
                    if (ep.len == 0)
                    {
                        rt_memset(ep.buffer, 0, sizeof(ep.buffer));
                        ep_get2(0, &ep, 8);
                        TRACE("[%4d] dump 8 bytes:\n", __LINE__);
                        DUMPHEX(ep.buffer, 8);
                    }
                    handle_packet_setup(&ep);
                }
                else
                {
                    TRACE("[%4d] error! I guess setup should only to endpoint 0\n", __LINE__);
                }
            }
            else
            { //OUT
                //how to handle this?!
                TRACE("[%4d] USB端点<%d> out packet, reg:<0x%x>, len:<%d>\n", __LINE__, ep_id, ep_value, ep.len);
                //handle_packet_out(&ep);
            }
        }
        else //IN
        {
            int ep_value;
            _ClearEP_CTR_TX(ep_id);
            ep_value = _GetENDPOINT(ep_id);
            if (set_address_flag)//FIXME
            {
                TRACE("[%4d] USB IN 中断 set address<0x%x>, reg:<0x%x>\n", __LINE__, gAddress, ep_value);
                set_address(gAddress);
                set_address_flag = 0;
            }

            if (ep_value & EP_SETUP)
            { //SETUP PACKET
                TRACE("[%4d] USB端点<%d> IN 中断 SETUP packet, reg<0x%x>\n", __LINE__, ep_id, ep_value);
                RT_ASSERT(!"should never be here! I guess\n");
                handle_packet_setup(&ep);
            }
            else
            { //IN PAKCET
                //考虑如果要发送的数据包很长则需要继续发送，并考虑0长度包发送。
                if (send_status.sent < send_status.total)
                {
                    int len;
                    len = MIN(send_status.total-send_status.sent, EP0_PACKET_SIZE);
                    ep_send(0, send_status.buf + send_status.sent, len);
                    send_status.sent += len;
                    TRACE("[%4d] USB端点<%d> IN 中断 IN packet, reg<0x%x>\n", __LINE__, ep_id, ep_value);
                } else if ((send_status.sent > 0)
                         && ((send_status.sent %  EP0_PACKET_SIZE) == 0))
                {
                    ep_send(send_status.ep, RT_NULL, 0);
                    TRACE("[%4d] USB端点<%d> IN 中断 IN packet, reg<0x%x>\n", __LINE__, ep_id, ep_value);
                }
                else
                {
                    TRACE("[%4d] USB端点<%d> IN 中断 IN packet, reg<0x%x>\n", __LINE__, ep_id, ep_value);
                }
                //handle_packet_in(&ep);
            }
        }
        enable_endpoint_rx(ep_id, EP0_PACKET_SIZE);
    }

    return ;
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
