#ifndef _DRV_LED_H
#define _DRV_LED_H

#ifdef __cplusplus
 extern "C" {
#endif
//#include <intrinsics.h>
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F7开发板
//LED驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/6/10
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

//LED端口定义
#define LED_ON(n)		(n>5 ? __nop() : HAL_GPIO_WritePin(GPIOC,(uint16_t)(GPIO_PIN_1<<n),GPIO_PIN_RESET))
#define LED_OFF(n)		(n>5 ? __nop() : HAL_GPIO_WritePin(GPIOC,(uint16_t)(GPIO_PIN_1<<n),GPIO_PIN_SET))

#define BEEP_ONCE		{HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);delay(100);HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);}
#define BEEP_LONG		{HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_RESET);delay(500);HAL_GPIO_WritePin(GPIOC,GPIO_PIN_5,GPIO_PIN_SET);}



#define LED_TOGGLE(n) ((n>5) ? __nop() : HAL_GPIO_TogglePin(GPIOC, (uint16_t)(GPIO_PIN_1<<n))) //LED输出电平翻转


void drv_Led_Init(void); //LED初始化函数

#ifdef __cplusplus
 }
#endif

#endif
