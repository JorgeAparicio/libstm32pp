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

#pragma once

#include "common.hpp"

namespace nvic {
  struct Registers
  {
      __RW
      u32 ISER[3];  // 0x000: Set enable
      u32 _RESERVED0[29];
      __RW
      u32 ICER[3];  // 0x080: Clear enable
      u32 RSERVED1[29];
      __RW
      u32 ISPR[3];  // 0x100: Set pending
      u32 _RESERVED2[29];
      __RW
      u32 ICPR[3];  // 0x180: Clear pending
      u32 _RESERVED3[29];
      __R
      u32 IABR[3];  // 0x200: Active bit
      u32 _RESERVED4[61];
      __RW
      u32 IPR[21];  // 0x300: Priority
      u32 _RESERVED5[683];
      __W
      u32 STIR;  // 0xE00: Software trigger
  };

  enum {
    ADDRESS = alias::PPB + 0x100
  };

  namespace irqn {
    enum {
      MASK = 0b1111
    };

    enum E {
      WWDG = 0,
      PVD = 1,
  #if defined VALUE_LINE || \
      defined STM32F2XX || \
      defined STM32F4XX
      TAMP_STAMP = 2,
      RTC_WKUP = 3,
  #else
      TAMPER = 2,
      RTC = 3,
  #endif
      FLASH = 4,
      RCC = 5,
      EXTI0 = 6,
      EXTI1 = 7,
      EXTI2 = 8,
      EXTI3 = 9,
      EXTI4 = 10,
  #ifdef STM32F1XX
      DMA1_Channel1 = 11,
      DMA1_Channel2 = 12,
      DMA1_Channel3 = 13,
      DMA1_Channel4 = 14,
      DMA1_Channel5 = 15,
      DMA1_Channel6 = 16,
      DMA1_Channel7 = 17,
  #else
      DMA1_Stream0 = 11,
      DMA1_Stream1 = 12,
      DMA1_Stream2 = 13,
      DMA1_Stream3 = 14,
      DMA1_Stream4 = 15,
      DMA1_Stream5 = 16,
      DMA1_Stream6 = 17,
  #endif
  #ifdef VALUE_LINE
      ADC1 = 18,
  #elif defined STM32F2XX || \
        defined STM32F4XX
      ADC = 18,
  #else
      ADC1_2 = 18,
  #endif
  #ifdef VALUE_LINE

  #elif defined CONNECTIVITY_LINE || \
        defined STM32F2XX || \
        defined STM32F4XX
      CAN1_TX = 19,
      CAN1_RX0 = 20,
  #else
      USB_HP_CAN1_TX = 19,
      USB_LP_CAN1_RX0 = 20,
  #endif
  #ifndef VALUE_LINE
      CAN1_RX1 = 21,
      CAN1_SCE = 22,
  #endif
      EXTI9_5 = 23,
  #ifdef VALUE_LINE
      TIM1_BRK_TIM15 = 24,
      TIM1_UP_TIM6 = 25,
      TIM1_TRG_COM_TIM17 = 26,
  #elif defined XL_DENSITY || \
        defined STM32F2XX || \
        defined STM32F4XX
      TIM1_BRK_TIM9 = 24,
      TIM1_UP_TIM10 = 25,
      TIM1_TRG_COM_TIM11 = 26,
  #else
      TIM1_BRK = 24,
      TIM1_UP = 25,
      TIM1_TRG_COM = 26,
  #endif
      TIM1_CC = 27,
      TIM2 = 28,
      TIM3 = 29,
      TIM4 = 30,
      I2C1_EV = 31,
      I2C1_ER = 32,
      I2C2_EV = 33,
      I2C2_ER = 34,
      SPI1 = 35,
      SPI2 = 36,
      USART1 = 37,
      USART2 = 38,
      USART3 = 39,
      EXTI15_10 = 40,
      RTCAlarm = 41,
  #ifdef VALUE_LINE
      CEC = 42,
  #elif defined CONNECTIVITY_LINE || \
        defined STM32F2XX || \
        defined STM32F4XX
      OTG_FS_WKUP = 42,
  #else
      USB_WakeUp = 42,
  #endif
  #ifdef VALUE_LINE
      TIM12 = 43,
      TIM13 = 44,
      TIM14 = 45,
  #elif defined CONNECTIVITY_LINE

  #elif defined XL_DENSITY || \
        defined STM32F2XX || \
        defined STM32F4XX
      TIM8_BRK_TIM12 = 43,
      TIM8_UP_TIM13 = 44,
      TIM8_TRG_COM_TIM14 = 45,
  #else
      TIM8_BRK = 43,
      TIM8_UP = 44,
      TIM8_TRG_COM = 45,
  #endif
  #if not defined VALUE_LINE && \
      not defined CONNECTIVITY_LINE
      TIM8_CC = 46,
  #endif
  #if defined VALUE_LINE || \
      defined CONNECTIVITY_LINE

  #elif defined STM32F2XX || \
        defined STM32F4XX
      ADC3 = 47,
  #else
      DMA1_Stream7 = 47,
  #endif
  #ifndef CONNECTIVITY_LINE
      FSMC = 48,
  #endif
  #if not defined VALUE_LINE && \
      not defined CONNECTIVITY_LINE
      SDIO = 49,
  #endif
      TIM5 = 50,
      SPI3 = 51,
      UART4 = 52,
      UART5 = 53,
  #if defined VALUE_LINE || \
      defined STM32F2XX || \
      defined STM32F4XX
      TIM6_DAC = 54,
  #else
      TIM6 = 54,
  #endif
      TIM7 = 55,
  #if defined STM32F2XX || \
      defined STM32F4XX
      DMA2_Stream0 = 56,
      DMA2_Stream1 = 57,
      DMA2_Stream2 = 58,
  #else
      DMA2_Channel1 = 56,
      DMA2_Channel2 = 57,
      DMA2_Channel3 = 58,
  #endif
  #if defined STM32F2XX || \
      defined STM32F4XX
      DMA2_Stream3 = 59,
  #elif defined CONNECTIVITY_LINE
      DMA2_Channel4 = 59,
  #else
      DMA2_Channel4_5 = 59,
  #endif
  #if defined STM32F2XX || \
      defined STM32F4XX
      DMA2_Stream4 = 60,
  #elif defined VALUE_LINE || \
        defined CONNECTIVITY_LINE
      DMA2_Channel5 = 60,
  #endif
  #if defined CONNECTIVITY_LINE || \
      defined STM32F2XX || \
      defined STM32F4XX
      ETH = 61,
      ETH_WKUP = 62,
      CAN2_TX = 63,
      CAN2_RX0 = 64,
      CAN2_RX1 = 65,
      CAN2_SCE = 66,
      OTG_FS = 67,
  #endif
  #if defined STM32F2XX || \
      defined STM32F4XX
      DMA2_Stream5 = 68,
      DMA2_Stream6 = 69,
      DMA2_Stream7 = 70,
      USART6 = 71,
      I2C3_EV = 72,
      I2C3_ER = 73,
      OTG_HS_EP1_OUT = 74,
      OTG_HS_EP1_IN = 75,
      OTG_HS_WKUP = 76,
      OTG_HS = 77,
      DCMI = 78,
      CRYP = 79,
      HASH_RNG = 80,
  #endif
  #ifdef STM32F4XX
      FPU = 81
  #endif
    };
  }  // namespace irqn
}  // namespace nvic
