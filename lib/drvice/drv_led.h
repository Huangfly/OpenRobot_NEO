#ifndef _DRV_LED_H
#define _DRV_LED_H

#ifdef __cplusplus
 extern "C" {
#endif
//#include <intrinsics.h>
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F7������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/6/10
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//LED�˿ڶ���
#define LED_ON(n)		(n>5 ? __nop() : HAL_GPIO_WritePin(GPIOC,(uint16_t)(GPIO_PIN_1<<n),GPIO_PIN_RESET))
#define LED_OFF(n)		(n>5 ? __nop() : HAL_GPIO_WritePin(GPIOC,(uint16_t)(GPIO_PIN_1<<n),GPIO_PIN_SET))

#define BEEP_ONCE		{HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);delay(100);HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);}
#define BEEP_LONG		{HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);delay(500);HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);}



#define LED_TOGGLE(n) ((n>5) ? __nop() : HAL_GPIO_TogglePin(GPIOC, (uint16_t)(GPIO_PIN_1<<n))) //LED�����ƽ��ת


void drv_Led_Init(void); //LED��ʼ������

#ifdef __cplusplus
 }
#endif

#endif
