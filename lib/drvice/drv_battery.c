/*
 *  drv_battry.c
 *
 *  Created on: 2016. 7. 13.
 *      Author: Baram, PBHP
 */
#include "stm32f746xx.h"
#include "stm32f7xx_hal.h"
#include "drv_battery.h"
//-- internal definition
//

ADC_HandleTypeDef hADC3;


int drv_adc_init()
{
  hADC3.Instance                   = ADC3;
  hADC3.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;
  hADC3.Init.Resolution            = ADC_RESOLUTION_12B;
  hADC3.Init.ScanConvMode          = DISABLE;
  hADC3.Init.ContinuousConvMode    = DISABLE;
  hADC3.Init.DiscontinuousConvMode = DISABLE;
  hADC3.Init.NbrOfDiscConversion   = 0;
  hADC3.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  hADC3.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  hADC3.Init.NbrOfConversion       = 1;
  hADC3.Init.DMAContinuousRequests = DISABLE;
  hADC3.Init.EOCSelection          = DISABLE;

  if (HAL_ADC_Init(&hADC3) != HAL_OK)
  {
    return -1;
  }

  return 0;
}

void drv_battery_init( uint32_t ulPin )
{
  GPIO_InitTypeDef GPIO_InitStruct;


  if( g_Pin2PortMapArray[ulPin].GPIOx_Port == NULL ) return;


  HAL_GPIO_DeInit(g_Pin2PortMapArray[ulPin].GPIOx_Port, g_Pin2PortMapArray[ulPin].Pin_abstraction);

  GPIO_InitStruct.Pin = g_Pin2PortMapArray[ulPin].Pin_abstraction;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(g_Pin2PortMapArray[ulPin].GPIOx_Port, &GPIO_InitStruct);
}



