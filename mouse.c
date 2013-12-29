#include <stm32f10x.h>
#include "usb_regs.h"

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

void Joystick_Send(unsigned char Keys)
{
  static unsigned char Mouse_Buffer[4] = {0, 0, 0, 0};
  char X = 0, Y = 0,MouseButton=0;

  switch (Keys)
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
	   
	case LEFT_BUTTON:
      MouseButton = MouseButton|0x01;
      break;

 	case RIGHT_BUTTON:
      MouseButton = MouseButton|0x02;
      break;

    default:
      return;
  }

  /* prepare buffer to send */
  Mouse_Buffer[0] = MouseButton;
  Mouse_Buffer[1] = X;
  Mouse_Buffer[2] = Y;
  
  ep_send(ENDP1, Mouse_Buffer, 4);
}

