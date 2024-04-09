/*
 * Bafang LCD 860C/850C firmware
 *
 * Copyright (C) Casainho, 2018, 2019, 2020
 *
 * Released under the GPL License, Version 3
 */

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stdio.h"

#include "pins.h"

void pins_init (void)
{
  // enable clocks
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
                         RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC |
#ifdef DISPLAY_860C_V13
                         RCC_APB2Periph_GPIOD |
#endif
						 RCC_APB2Periph_ADC1 |
                         RCC_APB2Periph_AFIO,
                         ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Pin = SYSTEM_POWER_ON_OFF__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SYSTEM_POWER_ON_OFF__PORT, &GPIO_InitStructure);

#if defined(DISPLAY_860C_V12) || defined(DISPLAY_860C_V13)								   
  GPIO_InitStructure.GPIO_Pin = SYSTEM_POWER_2_ON_OFF__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(SYSTEM_POWER_2_ON_OFF__PORT, &GPIO_InitStructure);
#endif

  GPIO_InitStructure.GPIO_Pin = BUTTON_ONOFF__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(BUTTON_ONOFF__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = BUTTON_UP__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(BUTTON_UP__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = BUTTON_DOWN__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(BUTTON_DOWN__PORT, &GPIO_InitStructure);

#if defined(DISPLAY_860C) || defined(DISPLAY_860C_V12) || defined(DISPLAY_860C_V13)
  GPIO_InitStructure.GPIO_Pin = BUTTON_M__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(BUTTON_M__PORT, &GPIO_InitStructure);
#endif

  GPIO_InitStructure.GPIO_Pin = LCD_BACKLIGHT__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(LCD_BACKLIGHT__PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = USB_CHARGE__PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(USB_CHARGE__PORT, &GPIO_InitStructure);
}

void system_power(uint32_t ui32_state)
{
  if(ui32_state)
  {
    GPIO_SetBits(SYSTEM_POWER_ON_OFF__PORT, SYSTEM_POWER_ON_OFF__PIN);
#if defined(DISPLAY_860C_V12) || defined(DISPLAY_860C_V13)
    GPIO_SetBits(SYSTEM_POWER_2_ON_OFF__PORT, SYSTEM_POWER_2_ON_OFF__PIN);
#endif
    GPIO_SetBits(USB_CHARGE__PORT, USB_CHARGE__PIN);
  }
  else
  {
    GPIO_ResetBits(SYSTEM_POWER_ON_OFF__PORT, SYSTEM_POWER_ON_OFF__PIN);
#if defined(DISPLAY_860C_V12) || defined(DISPLAY_860C_V13)
    GPIO_ResetBits(SYSTEM_POWER_2_ON_OFF__PORT, SYSTEM_POWER_2_ON_OFF__PIN);
#endif												
    GPIO_ResetBits(USB_CHARGE__PORT, USB_CHARGE__PIN);
  }
}
