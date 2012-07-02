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

#include "device_select.hpp"

namespace interrupt {
  extern "C" {
    void WWDG();
    void PVD();
#if defined VALUE_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX
    void TAMP_STAMP();
    void RTC_WKUP();
#else
    void TAMPER();
    void RTC();
#endif
    void FLASH();
    void RCC();
    void EXTI0();
    void EXTI1();
    void EXTI2();
    void EXTI3();
    void EXTI4();
#ifdef STM32F1XX
    void DMA1_Channel1();
    void DMA1_Channel2();
    void DMA1_Channel3();
    void DMA1_Channel4();
    void DMA1_Channel5();
    void DMA1_Channel6();
    void DMA1_Channel7();
#else
    void DMA1_Stream0();
    void DMA1_Stream1();
    void DMA1_Stream2();
    void DMA1_Stream3();
    void DMA1_Stream4();
    void DMA1_Stream5();
    void DMA1_Stream6();
#endif
#ifdef VALUE_LINE
    void ADC1();
#elif defined STM32F2XX || \
      defined STM32F4XX
    void ADC();
#else
    void ADC1_2();
#endif
#ifdef VALUE_LINE

#elif defined CONNECTIVITY_LINE || \
      defined STM32F2XX || \
      defined STM32F4XX
    void CAN1_TX();
    void CAN1_RX0();
#else
    void USB_HP_CAN1_TX();
    void USB_LP_CAN1_RX0();
#endif
#ifndef VALUE_LINE
    void CAN1_RX1();
    void CAN1_SCE();
#endif
    void EXTI9_5();
#ifdef VALUE_LINE
    void TIM1_BRK_TIM15();
    void TIM1_UP_TIM6();
    void TIM1_TRG_COM_TIM17();
#elif defined XL_DENSITY || \
      defined STM32F2XX || \
      defined STM32F4XX
    void TIM1_BRK_TIM9();
    void TIM1_UP_TIM10();
    void TIM1_TRG_COM_TIM11();
#else
    void TIM1_BRK();
    void TIM1_UP();
    void TIM1_TRG_COM();
#endif
    void TIM1_CC();
    void TIM2();
    void TIM3();
    void TIM4();
    void I2C1_EV();
    void I2C1_ER();
    void I2C2_EV();
    void I2C2_ER();
    void SPI1();
    void SPI2();
    void USART1();
    void USART2();
    void USART3();
    void EXTI15_10();
    void RTCAlarm();
#ifdef VALUE_LINE
    void CEC();
#elif defined CONNECTIVITY_LINE || \
      defined STM32F2XX || \
      defined STM32F4XX
    void OTG_FS_WKUP();
#else
    void USB_WakeUp();
#endif
#ifdef VALUE_LINE
    void TIM12();
    void TIM13();
    void TIM14();
#elif defined CONNECTIVITY_LINE

#elif defined XL_DENSITY || \
      defined STM32F2XX || \
      defined STM32F4XX
    void TIM8_BRK_TIM12();
    void TIM8_UP_TIM13();
    void TIM8_TRG_COM_TIM14();
#else
    void TIM8_BRK();
    void TIM8_UP();
    void TIM8_TRG_COM();
#endif
#if not defined VALUE_LINE && \
    not defined CONNECTIVITY_LINE
    void TIM8_CC();
#endif
#if defined VALUE_LINE || \
    defined CONNECTIVITY_LINE

#elif defined STM32F2XX || \
      defined STM32F4XX
    void ADC3();
#else
    void DMA1_Stream7();
#endif
#ifndef CONNECTIVITY_LINE
    void FSMC();
#endif
#if not defined VALUE_LINE && \
    not defined CONNECTIVITY_LINE
    void SDIO();
#endif
    void TIM5();
    void SPI3();
    void UART4();
    void UART5();
#if defined VALUE_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX
    void TIM6_DAC();
#else
    void TIM6();
#endif
    void TIM7();
#if defined STM32F2XX || \
    defined STM32F4XX
    void DMA2_Stream0();
    void DMA2_Stream1();
    void DMA2_Stream2();
#else
    void DMA2_Channel1();
    void DMA2_Channel2();
    void DMA2_Channel3();
#endif
#if defined STM32F2XX || \
    defined STM32F4XX
    void DMA2_Stream3();
#elif defined CONNECTIVITY_LINE
    void DMA2_Channel4();
#else
    void DMA2_Channel4_5();
#endif
#if defined STM32F2XX || \
    defined STM32F4XX
    void DMA2_Stream4();
#elif defined VALUE_LINE || \
      defined CONNECTIVITY_LINE
    void DMA2_Channel5();
#endif
#if defined CONNECTIVITY_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX
    void ETH();
    void ETH_WKUP();
    void CAN2_TX();
    void CAN2_RX0();
    void CAN2_RX1();
    void CAN2_SCE();
    void OTG_FS();
#endif
#if defined STM32F2XX || \
    defined STM32F4XX
    void DMA2_Stream5();
    void DMA2_Stream6();
    void DMA2_Stream7();
    void USART6();
    void I2C3_EV();
    void I2C3_ER();
    void OTG_HS_EP1_OUT();
    void OTG_HS_EP1_IN();
    void OTG_HS_WKUP();
    void OTG_HS();
    void DCMI();
    void CRYP();
    void HASH_RNG();
#endif
#ifdef STM32F4XX
    void FPU();
#endif
  }
}  // namespace interrupt
