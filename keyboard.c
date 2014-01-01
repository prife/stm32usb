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

enum HWKEYS
{
    ARROW_UP = 1,//indicate that the key is up
    DOWN,
    LEFT,
    RIGHT,
    UP,

    KEY_SEPERATE=63,

    BTN_UP,//indicate that the key is up
    LEFT_BUTTON,
    RIGHT_BUTTON,
};

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
    static unsigned char buf[9];
    int i = 3;

    memset(buf, 0, sizeof(buf));
    buf[0] = 1;
    if (Keys == ARROW_UP)
    {
        ep_send(ENDP2, buf, 5);
        return ;
    }

    if (Keys == BTN_UP)
    {
        ep_send(ENDP1, buf, 9);
        return ;
    }

    if (Keys <= KEY_SEPERATE)
    {
        unsigned char X=0, Y=0;
        switch(Keys)
        {
        case LEFT:
          X -= CURSOR_STEP;
          break;
        case RIGHT:
          X += CURSOR_STEP;
          break;
        case UP:
          Y -= CURSOR_STEP;
          break;
        case DOWN:
          Y += CURSOR_STEP;
          break;
        }
        buf[2] = X;
        buf[3] = Y;
        ep_send(ENDP2, buf, 5);
        return ;
    }

    switch (Keys)
    {
    case LEFT_BUTTON:
        buf[i] = 4; i++;//'A'
        break;
    case RIGHT_BUTTON:
        buf[i] = 5; i++;//'B'
        break;
    default:
        break;
    }

    ep_send(ENDP1, buf, 9);
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

    /* Initialize Endpoint 1 for keyboard */
    _SetEPType(ENDP1, EP_INTERRUPT);
    _SetEPRxAddr(ENDP1, ENDP1_RXADDR);
    _SetEPTxAddr(ENDP1, ENDP1_TXADDR);
    _SetEPTxCount(ENDP1, 8);
    _SetEPRxStatus(ENDP1, EP_RX_VALID);
    _SetEPTxStatus(ENDP1, EP_TX_NAK);

    /* Initialize Endpoint 2 for mouse */
    _SetEPType(ENDP2, EP_INTERRUPT);
    //_SetEPRxAddr(ENDP2, ENDP2_RXADDR);
    _SetEPTxAddr(ENDP2, ENDP2_TXADDR);
    _SetEPTxCount(ENDP2, 4);
    _SetEPRxStatus(ENDP2, EP_RX_DIS);
    _SetEPTxStatus(ENDP2, EP_TX_NAK);

    _SetEPAddress(0, 0);
    _SetEPAddress(1, 1);
    _SetEPAddress(2, 2);
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
        while(JoyState() != 0)
        {
            if (key <= KEY_SEPERATE)
            {
                Joystick_Send(key);
                rt_thread_delay(RT_TICK_PER_SECOND / 100);
            }
        }
        if(key > KEY_SEPERATE)
            Joystick_Send(BTN_UP);
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