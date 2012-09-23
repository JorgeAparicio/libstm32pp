/*******************************************************************************
 *
 * Copyright (C) 2012 Jorge Aparicio <jorge.aparicio.r@gmail.com>
 *
 * This file is part of libstm32pp.
 *
 * libstm32pp is free software: you can redistribute it and/or modify it under
 * the terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * libstm32pp is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
 * for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with libstm32pp. If not, see <http://www.gnu.org/licenses/>.
 *
 ******************************************************************************/

// This file places the STM32 microcontrollers' interrupts in an appropriate
// memory section. This header file is actually a source file, include it in
// another source file. e.g. for the bareCortexM project, include this file
// ONLY in the file named "interrupt.cpp"
#pragma once

#include "interrupt.hpp"

typedef void (*pvf)();

namespace interrupt {
  __attribute__ ((section(".interrupt_vector")))
  pvf interruptVector[] = {
      WWDG, // 0x0040
      PVD, // 0x0044
#if defined VALUE_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX
      TAMP_STAMP,  // 0x0048
      RTC_WKUP,  // 0x004C
#else
      TAMPER,  // 0x0048
      RTC,// 0x004C
#endif
      FLASH,  // 0x0050
      RCC,  // 0x0054
      EXTI0,  // 0x0058
      EXTI1,  // 0x005C
      EXTI2,  // 0x0060
      EXTI3,  // 0x0064
      EXTI4,  // 0x0068
#ifdef STM32F1XX
      DMA1_Channel1,  // 0x006C
      DMA1_Channel2,// 0x0070
      DMA1_Channel3,// 0x0074
      DMA1_Channel4,// 0x0078
      DMA1_Channel5,// 0x007C
      DMA1_Channel6,// 0x0080
      DMA1_Channel7,// 0x0084
#else
      DMA1_Stream0,  // 0x006C
      DMA1_Stream1,  // 0x0070
      DMA1_Stream2,  // 0x0074
      DMA1_Stream3,  // 0x0078
      DMA1_Stream4,  // 0x007C
      DMA1_Stream5,  // 0x0080
      DMA1_Stream6,  // 0x0084
#endif // STM32F1XX
#ifdef VALUE_LINE
      ADC1,  // 0x0088
#elif defined STM32F2XX || \
      defined STM32F4XX
      ADC,  // 0x0088
#else
      ADC1_2,  // 0x0088
#endif
#ifdef VALUE_LINE
      0,  // 0x008C
      0,// 0x0090
#elif defined CONNECTIVITY_LINE || \
      defined STM32F2XX || \
      defined STM32F4XX
      CAN1_TX,  // 0x008C
      CAN1_RX0,  // 0x0090
#else
      USB_HP_CAN1_TX,  // 0x008C
      USB_LP_CAN1_RX0,// 0x0090
#endif
#ifndef VALUE_LINE
      CAN1_RX1,  // 0x0094
      CAN1_SCE,  // 0x0098
#else
      0,  // 0x0094
      0,// 0x0098
#endif
      EXTI9_5,  // 0x009C
#ifdef VALUE_LINE
      TIM1_BRK_TIM15,  // 0x00A0
      TIM1_UP_TIM6,// 0x00A4
      TIM1_TRG_COM_TIM17,// 0x00A8
#elif defined XL_DENSITY || \
      defined STM32F2XX  || \
      defined STM32F4XX
      TIM1_BRK_TIM9,  // 0x00A0
      TIM1_UP_TIM10,  // 0x00A4
      TIM1_TRG_COM_TIM11,  // 0x00A8
#else
      TIM1_BRK,  // 0x00A0
      TIM1_UP,// 0x00A4
      TIM1_TRG_COM,// 0x00A8
#endif
      TIM1_CC,  // 0x00AC
      TIM2,  // 0x00B0
      TIM3,  // 0x00B4
      TIM4,  // 0x00B8
      I2C1_EV,  // 0x00BC
      I2C1_ER,  // 0x00C0
      I2C2_EV,  // 0x00C4
      I2C2_ER,  // 0x00C8
      SPI1,  // 0x00CC
      SPI2,  // 0x00D0
      USART1,  // 0x00D4
      USART2,  // 0x00D8
      USART3,  // 0x00DC
      EXTI15_10,  // 0x00E0
      RTCAlarm,  // 0x00E4
#ifdef VALUE_LINE
      CEC,  // 0x00E8
#elif defined CONNECTIVITY_LINE || \
      defined STM32F2XX || \
      defined STM32F4XX
      OTG_FS_WKUP,  // 0x00E8
#else
      USB_WakeUp,  // 0x00E8
#endif
#ifdef VALUE_LINE
      TIM12,  // 0x00EC
      TIM13,// 0x00F0
      TIM14,// 0x00F4
#elif defined CONNECTIVITY_LINE
      0,  // 0x00EC
      0,// 0x00F0
      0,// 0x00F4
#elif defined XL_DENSITY || \
      defined STM32F2XX || \
      defined STM32F4XX
      TIM8_BRK_TIM12,  // 0x00EC
      TIM8_UP_TIM13,  // 0x00F0
      TIM8_TRG_COM_TIM14,  // 0x00F4
#else
      TIM8_BRK,  // 0x00EC
      TIM8_UP,// 0x00F0
      TIM8_TRG_COM,// 0x00F4
#endif
#if defined VALUE_LINE || \
    defined CONNECTIVITY_LINE
      0,  // 0x00F8
#else
      TIM8_CC,  // 0x00F8
#endif
#if defined VALUE_LINE || \
    defined CONNECTIVITY_LINE
      0,  // 0x00FC
#elif defined STM32F2XX || \
      defined STM32F4XX
      ADC3,  // 0x00FC
#else
      DMA1_Stream7,  // 0x00FC
#endif
#ifdef CONNECTIVITY_LINE
      0,  // 0x0100
#else
      FSMC,  // 0x0100
#endif
#if defined VALUE_LINE || \
    defined CONNECTIVITY_LINE
      0,  // 0x0104
#else
      SDIO,  // 0x0104
#endif
      TIM5,  // 0x0108
      SPI3,  // 0x010C
      UART4,  // 0x0110
      UART5,  // 0x0114
#if defined VALUE_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX
      TIM6_DAC,  // 0x0118
#else
      TIM6,  // 0x0118
#endif
      TIM7,  // 0x011C
#if defined STM32F2XX || \
    defined STM32F4XX
      DMA2_Stream0,  // 0x0120
      DMA2_Stream1,  // 0x0124
      DMA2_Stream2,  // 0x0128
#else
      DMA2_Channel1,  // 0x0120
      DMA2_Channel2,// 0x0124
      DMA2_Channel3,// 0x0128
#endif
#if defined STM32F2XX || \
    defined STM32F4XX
      DMA2_Stream3,  // 0x012C
#elif defined CONNECTIVITY_LINE
      DMA2_Channel4,  // 0x012C
#else
      DMA2_Channel4_5,  // 0x012C
#endif
#if defined STM32F2XX || \
    defined STM32F4XX
      DMA2_Stream4,  // 0x0130
#elif defined VALUE_LINE || \
      defined CONNECTIVITY_LINE
      DMA2_Channel5,  // 0x0130
#else
      0,  // 0x0130
#endif
#if defined CONNECTIVITY_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX
      ETH,  // 0x0134
      ETH_WKUP,  // 0x0138
      CAN2_TX,  // 0x013C
      CAN2_RX0,  // 0x0140
      CAN2_RX1,  // 0x0144
      CAN2_SCE,  // 0x0148
      OTG_FS,  // 0x014C
#endif
#if defined STM32F2XX || \
    defined STM32F4XX
      DMA2_Stream5,  // 0x0150
      DMA2_Stream6,  // 0x0154
      DMA2_Stream7,  // 0x0158
      USART6,  // 0x015C
      I2C3_EV,  // 0x0160
      I2C3_ER,  // 0x0164
      OTG_HS_EP1_OUT,  // 0x0168
      OTG_HS_EP1_IN,  // 0x016C
      OTG_HS_WKUP,  // 0x0170
      OTG_HS,  // 0x0174
      DCMI,  // 0x0178
      CRYP,  // 0x017C
      HASH_RNG,  // 0x0180
#endif
#ifdef STM32F4XX
      FPU,  // 0x0184
#endif
      };  // interruptVector
}  // namespace interrupt

extern "C" void defaultInterruptHandler()
{
  while (true) {
  }
}

#pragma weak WWDG               = defaultInterruptHandler
#pragma weak PVD                = defaultInterruptHandler

#if defined VALUE_LINE || \
defined STM32F2XX || \
defined STM32F4XX
#pragma weak TAMP_STAMP         = defaultInterruptHandler
#pragma weak RTC_WKUP           = defaultInterruptHandler
#else
#pragma weak TAMPER             = defaultInterruptHandler
#pragma weak RTC                = defaultInterruptHandler
#endif

#pragma weak FLASH              = defaultInterruptHandler
#pragma weak RCC                = defaultInterruptHandler
#pragma weak EXTI0              = defaultInterruptHandler
#pragma weak EXTI1              = defaultInterruptHandler
#pragma weak EXTI2              = defaultInterruptHandler
#pragma weak EXTI3              = defaultInterruptHandler
#pragma weak EXTI4              = defaultInterruptHandler

#ifdef STM32F1XX
#pragma weak DMA1_Channel1      = defaultInterruptHandler
#pragma weak DMA1_Channel2      = defaultInterruptHandler
#pragma weak DMA1_Channel3      = defaultInterruptHandler
#pragma weak DMA1_Channel4      = defaultInterruptHandler
#pragma weak DMA1_Channel5      = defaultInterruptHandler
#pragma weak DMA1_Channel6      = defaultInterruptHandler
#pragma weak DMA1_Channel7      = defaultInterruptHandler
#else
#pragma weak DMA1_Stream0       = defaultInterruptHandler
#pragma weak DMA1_Stream1       = defaultInterruptHandler
#pragma weak DMA1_Stream2       = defaultInterruptHandler
#pragma weak DMA1_Stream3       = defaultInterruptHandler
#pragma weak DMA1_Stream4       = defaultInterruptHandler
#pragma weak DMA1_Stream5       = defaultInterruptHandler
#pragma weak DMA1_Stream6       = defaultInterruptHandler
#endif

#ifdef VALUE_LINE
#pragma weak ADC1               = defaultInterruptHandler
#elif defined STM32F2XX || \
      defined STM32F4XX
#pragma weak ADC                = defaultInterruptHandler
#else
#pragma weak ADC1_2             = defaultInterruptHandler
#endif

#ifdef VALUE_LINE

#elif defined CONNECTIVITY_LINE || \
      defined STM32F2XX || \
      defined STM32F4XX
#pragma weak CAN1_TX            = defaultInterruptHandler
#pragma weak CAN1_RX0           = defaultInterruptHandler
#else
#pragma weak USB_HP_CAN1_TX     = defaultInterruptHandler
#pragma weak USB_LP_CAN1_RX0    = defaultInterruptHandler
#endif

#ifndef VALUE_LINE
#pragma weak CAN1_RX1           = defaultInterruptHandler
#pragma weak CAN1_SCE           = defaultInterruptHandler
#endif

#pragma weak EXTI9_5            = defaultInterruptHandler

#ifdef VALUE_LINE
#pragma weak TIM1_BRK_TIM15     = defaultInterruptHandler
#pragma weak TIM1_UP_TIM6       = defaultInterruptHandler
#pragma weak TIM1_TRG_COM_TIM17 = defaultInterruptHandler
#elif defined XL_DENSITY || \
      defined STM32F2XX || \
      defined STM32F4XX
#pragma weak TIM1_BRK_TIM9      = defaultInterruptHandler
#pragma weak TIM1_UP_TIM10      = defaultInterruptHandler
#pragma weak TIM1_TRG_COM_TIM11 = defaultInterruptHandler
#else
#pragma weak TIM1_BRK           = defaultInterruptHandler
#pragma weak TIM1_UP            = defaultInterruptHandler
#pragma weak TIM1_TRG_COM       = defaultInterruptHandler
#endif

#pragma weak TIM1_CC            = defaultInterruptHandler
#pragma weak TIM2               = defaultInterruptHandler
#pragma weak TIM3               = defaultInterruptHandler
#pragma weak TIM4               = defaultInterruptHandler
#pragma weak I2C1_EV            = defaultInterruptHandler
#pragma weak I2C1_ER            = defaultInterruptHandler
#pragma weak I2C2_EV            = defaultInterruptHandler
#pragma weak I2C2_ER            = defaultInterruptHandler
#pragma weak SPI1               = defaultInterruptHandler
#pragma weak SPI2               = defaultInterruptHandler
#pragma weak USART1             = defaultInterruptHandler
#pragma weak USART2             = defaultInterruptHandler
#pragma weak USART3             = defaultInterruptHandler
#pragma weak EXTI15_10          = defaultInterruptHandler
#pragma weak RTCAlarm           = defaultInterruptHandler

#ifdef VALUE_LINE
#pragma weak CEC                = defaultInterruptHandler
#elif defined CONNECTIVITY_LINE || \
      defined STM32F2XX || \
      defined STM32F4XX
#pragma weak OTG_FS_WKUP        = defaultInterruptHandler
#else
#pragma weak USB_WakeUp         = defaultInterruptHandler
#endif

#ifdef VALUE_LINE
#pragma weak TIM12              = defaultInterruptHandler
#pragma weak TIM13              = defaultInterruptHandler
#pragma weak TIM14              = defaultInterruptHandler
#elif defined CONNECTIVITY_LINE

#elif defined XL_DENSITY || \
      defined STM32F2XX || \
      defined STM32F4XX
#pragma weak TIM8_BRK_TIM12     = defaultInterruptHandler
#pragma weak TIM8_UP_TIM13      = defaultInterruptHandler
#pragma weak TIM8_TRG_COM_TIM14 = defaultInterruptHandler
#else
#pragma weak TIM8_BRK           = defaultInterruptHandler
#pragma weak TIM8_UP            = defaultInterruptHandler
#pragma weak TIM8_TRG_COM       = defaultInterruptHandler
#endif

#if not defined VALUE_LINE && \
    not defined CONNECTIVITY_LINE
#pragma weak TIM8_CC            = defaultInterruptHandler
#endif

#if defined VALUE_LINE || \
    defined CONNECTIVITY_LINE

#elif defined STM32F2XX || \
      defined STM32F4XX
#pragma weak ADC3               = defaultInterruptHandler
#else
#pragma weak DMA1_Stream7       = defaultInterruptHandler
#endif

#ifndef CONNECTIVITY_LINE
#pragma weak FSMC               = defaultInterruptHandler
#endif

#if not defined VALUE_LINE && \
    not defined CONNECTIVITY_LINE
#pragma weak SDIO               = defaultInterruptHandler
#endif

#pragma weak TIM5               = defaultInterruptHandler
#pragma weak SPI3               = defaultInterruptHandler
#pragma weak UART4              = defaultInterruptHandler
#pragma weak UART5              = defaultInterruptHandler

#if defined VALUE_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX
#pragma weak TIM6_DAC           = defaultInterruptHandler
#else
#pragma weak TIM6               = defaultInterruptHandler
#endif

#pragma weak TIM7               = defaultInterruptHandler

#if defined STM32F2XX || \
    defined STM32F4XX
#pragma weak DMA2_Stream0       = defaultInterruptHandler
#pragma weak DMA2_Stream1       = defaultInterruptHandler
#pragma weak DMA2_Stream2       = defaultInterruptHandler
#else
#pragma weak DMA2_Channel1      = defaultInterruptHandler
#pragma weak DMA2_Channel2      = defaultInterruptHandler
#pragma weak DMA2_Channel3      = defaultInterruptHandler
#endif

#if defined STM32F2XX || \
    defined STM32F4XX
#pragma weak DMA2_Stream3       = defaultInterruptHandler
#elif defined CONNECTIVITY_LINE
#pragma weak DMA2_Channel4      = defaultInterruptHandler
#else
#pragma weak DMA2_Channel4_5    = defaultInterruptHandler
#endif

#if defined STM32F2XX || \
    defined STM32F4XX
#pragma weak DMA2_Stream4       = defaultInterruptHandler
#elif defined VALUE_LINE || \
      defined CONNECTIVITY_LINE
#pragma weak DMA2_Channel5      = defaultInterruptHandler
#endif

#if defined CONNECTIVITY_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX
#pragma weak ETH                = defaultInterruptHandler
#pragma weak ETH_WKUP           = defaultInterruptHandler
#pragma weak CAN2_TX            = defaultInterruptHandler
#pragma weak CAN2_RX0           = defaultInterruptHandler
#pragma weak CAN2_RX1           = defaultInterruptHandler
#pragma weak CAN2_SCE           = defaultInterruptHandler
#pragma weak OTG_FS             = defaultInterruptHandler
#endif

#if defined STM32F2XX || \
    defined STM32F4XX
#pragma weak DMA2_Stream5       = defaultInterruptHandler
#pragma weak DMA2_Stream6       = defaultInterruptHandler
#pragma weak DMA2_Stream7       = defaultInterruptHandler
#pragma weak USART6             = defaultInterruptHandler
#pragma weak I2C3_EV            = defaultInterruptHandler
#pragma weak I2C3_ER            = defaultInterruptHandler
#pragma weak OTG_HS_EP1_OUT     = defaultInterruptHandler
#pragma weak OTG_HS_EP1_IN      = defaultInterruptHandler
#pragma weak OTG_HS_WKUP        = defaultInterruptHandler
#pragma weak OTG_HS             = defaultInterruptHandler
#pragma weak DCMI               = defaultInterruptHandler
#pragma weak CRYP               = defaultInterruptHandler
#pragma weak HASH_RNG           = defaultInterruptHandler
#endif

#ifdef STM32F4XX
#pragma weak FPU                = defaultInterruptHandler
#endif
