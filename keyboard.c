#include <stm32f10x.h>
#include "usb_regs.h"
#include "usb_conf.h"
#include "usb.h"

#define JOY_UP              GPIO_Pin_11    /* PG.11 */
#define JOY_DOWN            GPIO_Pin_8     /* PG.8 */
#define JOY_LEFT            GPIO_Pin_13    /* PG.13 */
#define JOY_RIGHT           GPIO_Pin_14    /* PG.14 */
#define JOY_LEFT_BUTTON     GPIO_Pin_0     /* PA.0 */
#define JOY_RIGHT_BUTTON    GPIO_Pin_13    /* PC.13 */

#define CURSOR_STEP     5
#define DOWN            1
#define LEFT            2
#define RIGHT           3
#define UP              4
#define LEFT_BUTTON     5
#define RIGHT_BUTTON    6 

extern void ep_send(int ep_nr, const u8 * buf, int len);

void Joystick_gpio_init()
{
  GPIO_InitTypeDef GPIO_InitStructure; 

  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA
          | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOG , ENABLE); 
  /**
  *  WAKEUP -> PA0 , TAMPER -> PC13 , JOY_UP -> PG.11 , JOY_DOWN -> PG.8 
  *  JOY_LEFT -> PG.13 , JOY_RIGHT -> PG.14     
  */ 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_Init(GPIOG, &GPIO_InitStructure);
}

unsigned char JoyState(void)
{
  /* "right" key is pressed */
  if (!GPIO_ReadInputDataBit(GPIOG, JOY_RIGHT))
  {
    return RIGHT;
  }

  /* "left" key is pressed */
  if (!GPIO_ReadInputDataBit(GPIOG, JOY_LEFT))
  {
    return LEFT;
  }
  /* "up" key is pressed */
  if (!GPIO_ReadInputDataBit(GPIOG, JOY_UP))
  {
    return UP;
  }
  /* "down" key is pressed */
  if (!GPIO_ReadInputDataBit(GPIOG, JOY_DOWN))
  {
    return DOWN;
  }
  if ( GPIO_ReadInputDataBit(GPIOA, JOY_LEFT_BUTTON))
  {
    return LEFT_BUTTON;
  }
   if ( !GPIO_ReadInputDataBit(GPIOC, JOY_RIGHT_BUTTON))
  {
    return RIGHT_BUTTON;
  }
  /* No key is pressed */
  else
  {
    return 0;
  }
}

#include <string.h>
//Key value: http://www.amobbs.com/thread-4332557-1-1.html
void Joystick_Send(unsigned char Keys)
{
    static unsigned char buf[8];
    int i = 2;

    memset(buf, 0, 8);
    switch (Keys)
    {
    case LEFT:
        buf[i] = 4; i++;
        break;
    case RIGHT:
        buf[i] = 5; i++;
        break;
    case UP:
        buf[i] = 6; i++;
        break;
    case DOWN:
        buf[i] = 7; i++;
        break;
    case LEFT_BUTTON:
        buf[i] = 8; i++;
        break;
    case RIGHT_BUTTON:
        buf[i] = 0x39; i++; //CapsLock
        break;
    default:
        break;
    }

    ep_send(ENDP1, buf, 8);
}

void usb_hw_ep_config(void)
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
    _SetEPRxAddr(ENDP1, ENDP1_RXADDR);
    _SetEPTxAddr(ENDP1, ENDP1_TXADDR);
    _SetEPTxCount(ENDP1, 8);
    _SetEPRxStatus(ENDP1, EP_RX_VALID);
    _SetEPTxStatus(ENDP1, EP_TX_NAK);

    _SetEPAddress(0, 0);
    _SetEPAddress(1, 1);
    /* enable USB endpoint and config EP0 */
    _SetDADDR(DADDR_EF | 0);
}

#include <rtthread.h>

void rt_keyboard_thread_entry(void * para)
{
    unsigned char key=0;
    while(1)
    {
        if ((key = JoyState()) == 0)
            continue;
        rt_thread_delay(RT_TICK_PER_SECOND / 100);
        if (JoyState() != key)
            continue;

        Joystick_Send(key);
        while(JoyState() != 0);
        Joystick_Send(0);
    }
}

void usb_app_init(void)
{
    rt_thread_t thread;
    Joystick_gpio_init();
    thread = rt_thread_create("init",
                               rt_keyboard_thread_entry, RT_NULL,
                               512, 30, 20);
    RT_ASSERT(thread != RT_NULL);
    rt_thread_startup(thread);
}