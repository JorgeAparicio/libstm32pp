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

#include "peripheral/pwr.hpp"
#include "cfunctions.hpp"

namespace clk {
  /****************************************************************************
   *                                                                          *
   *                               CLOCK SOURCES                              *
   *                                                                          *
   ****************************************************************************/
#if defined USING_HSE_CRYSTAL || \
    defined USING_HSE_CLOCK
#if defined STM32F1XX
#ifdef VALUE_LINE
  static_assert((HSE >= 4000000) && (HSE <= 24000000),
      " The external clock frequency must be between 4 MHz and 24 MHz");
#elif defined CONNECTIVITY_LINE
  static_assert((HSE >= 3000000) && (HSE <= 25000000),
      " The external clock frequency must be between 3 MHz and 25 MHz");
#else // VALUE_LINE
  static_assert((HSE >= 4000000) && (HSE <= 16000000),
      " The external clock frequency must be between 4 MHz and 16 MHz");
#endif // VALUE_LINE
#else // STM32F1XX
  static_assert((HSE >= 4000000) && (HSE <= 26000000),
      " The external clock frequency must be between 4 MHz and 26 MHz");
#endif // STM32F1XX
#else // USING_HSE_CRYSTAL || USING_HSE_CLOCK
  enum {
    HSE = 0
  };
#endif // USING_HSE_CRYSTAL || USING_HSE_CLOCK
#ifdef USING_LSE_CRYSTAL
  static_assert(LSE == 32768, "The LSE crystal frequency must be 32.768 KHz");
#elif defined USING_LSE_CLOCK
  static_assert(LSE < 1000000,
      "The LSE clock frequency must be lower than 1MHz");
#else
  enum {
    LSE = 0
  };
#endif

#ifdef STM32F1XX
  enum {
    HSI = 8000000,
#ifdef USING_LSI
    LSI = 40000
#else // USING_LSI
    LSI = 0
#endif // USING_LSI
  };
#else // STM32F1XX
  enum {
    HSI = 16000000,
    #ifdef USING_LSI
    LSI = 32000
#else // USING_LSI
    LSI = 0
#endif // USING_LSI
  };
#endif // STM32F1XX
  /****************************************************************************
   *                                                                          *
   *                       REAL TIME CLOCK CONFIGURATION                      *
   *                                                                          *
   ****************************************************************************/
#ifdef USING_RTC

#ifdef STM32F1XX
  enum {
    __RTCPRE = 128
  };
#else // STM32F1XX
  static_assert((__RTCPRE >= 2) && (__RTCPRE <= 31),
      "RTCPRE must be between 2 and 31 (inclusive).");
#endif // STM32F1XX
  enum {
    RTC =
    __RTCSEL ==
    rcc::bdcr::rtcsel::
    HSE_CLOCK_AS_RTC_SOURCE ?
    HSE / __RTCPRE :
    (__RTCSEL ==
        rcc::bdcr::rtcsel::
        LSE_CLOCK_AS_RTC_SOURCE ?
        LSE :
        (__RTCSEL ==
            rcc::bdcr::rtcsel::
            LSI_CLOCK_AS_RTC_SOURCE ?
            LSI :
            0))
  };
#else // USING_RTC
  enum {
    RTC = 0
  };
#endif // USING_RTC
  /****************************************************************************
   *                                                                          *
   *                             PLL CONFIGURATION                            *
   *                                                                          *
   ****************************************************************************/
#ifdef USING_PLL
#ifdef STM32F1XX
#ifndef CONNECTIVITY_LINE
#ifdef VALUE_LINE
  static_assert((__PREDIV1 >= 1) && (__PREDIV1 <= 16),
      "PREDIV1 must be between 1 and 16 (inclusive)");
  enum {
    _PLLSRC = __PLLSRC ==
    rcc::cfgr::pllsrc::
    USE_HSI_CLOCK_OVER_2_AS_PLL_SOURCE ?
    HSI:
    (__PLLSRC ==
        rcc::cfgr::pllsrc::
        USE_PREDIV1_OUTPUT_AS_PLL_SOURCE ?
        HSE / __PREDIV1:
        0)
  };
#else // VALUE_LINE
  static_assert((__PLLXTPRE >= 0) && (__PLLXTPRE <= 1),
      "PLLXTPRE can only take two values: 0 or 1");
  enum {
    _PLLSRC = __PLLSRC ==
    rcc::cfgr::pllsrc::
    USE_HSI_CLOCK_OVER_2_AS_PLL_SOURCE ?
    (HSI / 2):
    ( __PLLSRC ==
        rcc::cfgr::pllsrc::
        USE_PREDIV1_OUTPUT_AS_PLL_SOURCE ?
        HSE / (1 + __PLLXTPRE):
        0)
  };
#endif // VALUE_LINE
#else // !CONNECTIVITY_LINE
  static_assert((__PREDIV2 >= 1) && (__PREDIV2 <= 16),
      "PREDIV2 must be between 1 and 16 (inclusive).");

  static_assert((__PLL2MUL >= 6) && (__PLL2MUL <= 15) && (__PLL2MUL != 13),
      "PLL2MUL must be between 6 and 15 (inclusive, without 13).");
  enum {
    __PLL2 = HSE * (
        __PLL2MUL == 15?
        __PLL2MUL + 5:
        __PLL2MUL + 2
    ) / __PREDIV2,
  };

  enum {
    _PLLSRC =
    __PLLSRC ==
    rcc::cfgr::pllsrc::
    USE_HSI_CLOCK_OVER_2_AS_PLL_SOURCE?
    HSI / 2:
    (__PLLSRC !=
        rcc::cfgr::pllsrc::
        USE_PREDIV1_OUTPUT_AS_PLL_SOURCE?
        0:
        (__PREDIV1SRC ==
            rcc::cfgr2::prediv1src::
            USE_HSE_OSCILLATOR_AS_PREDIV1_INPUT?
            HSE / __PREDIV1:
            (__PREDIV1SRC ==
                rcc::cfgr2::prediv1src::
                USE_PLL2_AS_PREDIV1_INPUT?
                __PLL2 / __PREDIV1:
                0)))
  };

#endif // !CONNECTIVITY_LINE
  static_assert((__PLLMUL >= 2) && (__PLLMUL <= 16),
      "PLLMUL must be between 2 and 16 (inclusive)");
  enum {
    PLL = _PLLSRC * __PLLMUL
  };
#ifdef VALUE_LINE
  static_assert(PLL <= 24000000,
      "The PLL clock can't exceed 24 MHz");
#else // VALUE_LINE
  static_assert(PLL <= 72000000,
      "The PLL clock can't exceed 72 MHz");
#endif // VALUE_LINE
#else // STM32F1XX
  enum {
    _PLLSRC =
    __PLLSRC ==
    rcc::pllcfgr::pllsrc::
    USE_HSE_CLOCK_AS_PLL_CLOCK_SOURCE ?
    HSE :
    (__PLLSRC ==
        rcc::pllcfgr::pllsrc::
        USE_HSI_CLOCK_AS_PLL_CLOCK_SOURCE ?
        HSI :
        0)
  };
  static_assert((__PLLM >= 2) && (__PLLM <= 63),
      "PLLM must be between 2 and 63 (inclusive).");
  static_assert((__PLLN >= 64) && (__PLLN <= 432),
      "PLLM must be between 64 and 432 (inclusive).");
  static_assert((__PLLP % 2 == 0) && (__PLLP >= 2) && (__PLLP <= 8),
      "PLLP can only take the following values: 2, 4, 6 or 8.");
  static_assert((__PLLQ >= 2) && (__PLLQ <= 15),
      "PLLQ must be between 2 and 15 (inclusive).");

  enum {
    __VCO_IN = _PLLSRC / __PLLM,
    __VCO_OUT = __VCO_IN * __PLLN,
    PLL = __VCO_OUT / __PLLP,
    USB = __VCO_OUT / __PLLQ,
    SDIO = USB,
    RNG = SDIO
  };

  static_assert((__VCO_IN >= 1000000) && (__VCO_IN <= 2000000),
      "VCO_IN must be between 1MHz and 2MHZ (inclusive).");

  static_assert((__VCO_OUT >= 64000000) && (__VCO_OUT <= 432000000),
      "VCO_OUT must be between 64MHz and 432MHZ (inclusive).");

#ifdef STM32F2XX
  static_assert((PLL <= 120000000), "The PLL clock can't exceed 120MHz.");
#else // STM32F2XX
  static_assert((PLL <= 168000000), "The PLL clock can't exceed 168MHz.");
#endif // STM32F2XX
  static_assert((USB <= 48000000),
      "The USB/SDIO/RNG clock can't exceed 48MHz.");
#endif // STM32F1XX
#else // USING_PLL
  enum {
    PLL = 0,
#ifdef CONNECTIVITY_LINE
  _PLL2 = 0
#endif
};
#endif // USING_PLL
/****************************************************************************
 *                                                                          *
 *                          SYSTEM CLOCK SELECTION                          *
 *                                                                          *
 ****************************************************************************/
enum {
  SYSTEM =
  __SW ==
      rcc::cfgr::sw::
      HSE_OSCILLATOR_SELECTED_AS_SYSTEM_CLOCK ?
      u32(HSE) :
      (__SW ==
          rcc::cfgr::sw::
          HSI_OSCILLATOR_SELECTED_AS_SYSTEM_CLOCK ?
          HSI :
          (__SW ==
              rcc::cfgr::sw::
              PLL_SELECTED_AS_SYSTEM_CLOCK ?
                                             PLL :
                                             0))
};

/****************************************************************************
 *                                                                          *
 *                              BUS PRESCALERS                              *
 *                                                                          *
 ****************************************************************************/
#ifdef STM32F1XX
enum {
  AHB = SYSTEM / cPow<2, __HPRE>::value,
  CORE = AHB,
  APB1 = AHB / cPow<2, __PPRE1>::value,
  APB2 = AHB / cPow<2, __PPRE2>::value,
  APB1_TIMERS = APB1 * ((__PPRE1 == 0) ? 1 : 2),
  APB2_TIMERS = APB2 * ((__PPRE2 == 0) ? 1 : 2),
  ADC = APB2 / (2 * __ADCPRE + 2),
  SDIO = AHB
};
#ifndef VALUE_LINE
static_assert(APB1 <= 36000000,
    "The APB1 clock can't exceed 36 MHz");
static_assert(ADC <= 14000000,
    "The ADC clock can't exceed 14 MHz");
#endif // !VALUE_LINE
#else // STM32F1XX
enum {
  AHB = SYSTEM / cPow<2, __HPRE>::value,
  CORE = AHB,
  SYSTICK = AHB / 8,
  APB1 = AHB / cPow<2, __PPRE1>::value,
  APB2 = AHB / cPow<2, __PPRE2>::value,
  APB1_TIMERS = APB1 * ((__PPRE1 == 0) ? 1 : 2),
  APB2_TIMERS = APB2 * ((__PPRE2 == 0) ? 1 : 2),
};

static_assert(APB1 <= 42000000,
    "The APB1 clock can't exceed 42 MHz");
static_assert(APB2 <= 84000000,
    "The APB2 clock can't exceed 84 MHz");
#endif // STM32F1XX
/****************************************************************************
 *                                                                          *
 *                             PERIPHERAL CLOCKS                            *
 *                                                                          *
 ****************************************************************************/
#ifdef USING_USB
#if not defined STM32F2XX && \
    not defined STM32F4XX
enum {
  USB = PLL * 2 / (2 + __USBPRE)
};
#endif
static_assert((USB == 48000000),
    "The USB clock must be 48MHz, otherwise the module won't work.");
#else
enum {
  __USBPRE = 0
};
#endif // USING_USB
#ifdef USING_ETHERNET
static_assert((AHB >= 25000000),
    "The AHB clock must be at least 25MHz, "
    "otherwise the Ethernet module won't work.");
#endif // USING_ETHERNET
#ifdef USING_RTC
static_assert((RTC <= 1000000), "The RTC clock can't exceed 1MHz.");
#endif // USING_RTC
#ifdef USING_I2S
#ifdef STM32F1XX
#ifdef CONNECTIVITY_LINE
#ifdef USING_I2S_PLL
#ifndef USING_PLL
static_assert((__PREDIV2 >= 1) && (__PREDIV2 <= 16),
    "PREDIV2 must be between 1 and 16 (inclusive).");
#endif // !USING_PLL
static_assert((__PLL3MUL >= 6) && (__PLL3MUL <= 15) && (__PLL3MUL != 13),
    "PLL3MUL must be between 6 and 15 (inclusive, without 13).");

enum {
  __PLL3 = HSE * (
      __PLL3MUL == 15?
      __PLL3MUL + 5:
      __PLL3MUL + 2
  ) / __PREDIV2,
};
#else // USING_I2S_PLL
enum {
  __PLL3 = 0
};
#endif // USING_I2S_PLL
enum {
  I2S2 = __I2S2SRC ==
  rcc::cfgr2::i2s2src::
  USE_PLL3_CLOCK_AS_I2S2_CLOCK?
  2 * __PLL3:
  (__I2S2SRC ==
      rcc::cfgr2::i2s2src::
      USE_SYSTEM_CLOCK_AS_I2S2_CLOCK?
      SYSTEM:
      0),
  I2S3 = __I2S3SRC ==
  rcc::cfgr2::i2s3src::
  USE_PLL3_CLOCK_AS_I2S3_CLOCK?
  2 * __PLL3:
  (__I2S2SRC ==
      rcc::cfgr2::i2s3src::
      USE_SYSTEM_CLOCK_AS_I2S3_CLOCK?
      SYSTEM:
      0),
};
#else // CONNECTIVITY_LINE
enum {
  I2S2 = SYSTEM,
  I2S3 = SYSTEM
};
#endif // CONNECTIVITY_LINE
#else // STM32F1XX
#ifndef USING_PLL
enum {
  _PLLSRC =
  __PLLSRC ==
  rcc::pllcfgr::pllsrc::
  USE_HSE_CLOCK_AS_PLL_CLOCK_SOURCE ?
  HSE :
  (__PLLSRC ==
      rcc::pllcfgr::pllsrc::
      USE_HSI_CLOCK_AS_PLL_CLOCK_SOURCE ?
      HSI :
      0)
};

static_assert((__PLLM >= 2) && (__PLLM <= 63),
    "PLLM must be between 2 and 63 (inclusive).");
#endif // !USING_PLL
#ifdef USING_I2S_PLL
static_assert((__PLLI2SN >= 192) && (__PLLI2SN <= 432),
    "PLLI2SN must be between 192 and 432 (inclusive).");

static_assert((__PLLI2SR >= 2) && (__PLLI2SR <= 7),
    "PLLI2SR must be between 2 and 7 (inclusive).");

enum {
  __VCO_I2S_IN = __PLLSRC / __PLLM,
  __VCO_I2S_OUT = __VCO_I2S_IN * __PLLI2SN,
  __PLLI2S = __VCO_I2S_OUT / __PLLI2SR,
  I2S = __PLLI2S
};

static_assert((__VCO_I2S_IN >= 1000000) && (__VCO_I2S_IN <= 2000000),
    "VCO_I2S_IN must be between 1MHz and 2MHZ (inclusive).");

static_assert((__VCO_I2S_OUT >= 192000000) && (__VCO_I2S_OUT <= 432000000),
    "VCO_I2S_OUT must be between 192MHz and 432MHZ (inclusive).");

static_assert(__PLLI2S <= 192000000,
    "PLLI2S can't exceed 192MHz");
#else // USING_I2S_PLL
enum {
  I2S = 0
};
#endif // USING_I2S_PLL
#endif // STM32F1XX
#else // USING_I2S
#ifdef CONNECTIVITY_LINE
enum {
  __PLL3 = 0
};
#endif // CONNECTIVITY_LINE
#endif // USING_I2S
/****************************************************************************
 *                                                                          *
 *                       MICROCONTROLLER CLOCK OUTPUT                       *
 *                                                                          *
 ****************************************************************************/
#ifdef USING_MCO
#ifdef STM32F1XX
#ifndef CONNECTIVITY_LINE
enum {
  MCO =
  __MCO ==
  rcc::cfgr::mco::
  NO_CLOCK_OUTPUT ?
  0 :
  (__MCO ==
      rcc::cfgr::mco::
      OUTPUT_HSE_CLOCK ?
      HSE :
      (__MCO ==
          rcc::cfgr::mco::
          OUTPUT_HSI_CLOCK ?
          HSI :
          (__MCO ==
              rcc::cfgr::mco::
              OUTPUT_PLL_CLOCK_OVER_2 ?
              PLL / 2 :
              (__MCO ==
                  rcc::cfgr::mco::
                  OUTPUT_SYSTEM_CLOCK ?
                  SYSTEM :
                  0))))
};
#else // !CONNECTIVITY_LINE
enum {
  MCO =
  __MCO ==
  rcc::cfgr::mco::
  NO_CLOCK_OUTPUT ?
  0 :
  (__MCO ==
      rcc::cfgr::mco::
      OUTPUT_HSE_CLOCK ?
      HSE :
      (__MCO ==
          rcc::cfgr::mco::
          OUTPUT_HSI_CLOCK ?
          HSI :
          (__MCO ==
              rcc::cfgr::mco::
              OUTPUT_PLL2_CLOCK ?
              __PLL2 :
              (__MCO ==
                  rcc::cfgr::mco::
                  OUTPUT_PLL3_CLOCK ?
                  __PLL3 :
                  (__MCO ==
                      rcc::cfgr::mco::
                      OUTPUT_PLL_CLOCK_OVER_2 ?
                      PLL / 2 :
                      (__MCO ==
                          rcc::cfgr::mco::
                          OUTPUT_SYSTEM_CLOCK ?
                          SYSTEM :
                          (__MCO ==
                              rcc::cfgr::mco::
                              OUTPUT_XT1_CLOCK ?
                              HSE :
                              0)))))))
};
#endif // !CONNECTIVITY_LINE
static_assert(MCO <= 50000000,
    "MCO can't exceed 50 MHz");
#else // STM32F1XX
static_assert((__MCO1PRE >= 1) && (__MCO1PRE <= 5),
    "MCOPRE1 must be between 1 and 5 (inclusive).");

static_assert((__MCO2PRE >= 1) && (__MCO2PRE <= 5),
    "MCOPRE2 must be between 1 and 5 (inclusive).");

enum {
  MCO1 = (
      __MCO1 ==
      rcc::cfgr::mco1::
      OUTPUT_HSE_CLOCK ?
      HSE :
      (__MCO1 ==
          rcc::cfgr::mco1::
          OUTPUT_HSI_CLOCK ?
          HSI :
          (__MCO1 ==
              rcc::cfgr::mco1::
              OUTPUT_LSE_CLOCK ?
              LSE :
              (__MCO1 ==
                  rcc::cfgr::mco1::
                  OUTPUT_PLL_CLOCK ?
                  PLL :
                  0)))
  ) / __MCO1PRE,

  MCO2 = (
      __MCO2 ==
      rcc::cfgr::mco2::
      OUTPUT_HSE_CLOCK ?
      HSE :
      (__MCO2 ==
          rcc::cfgr::mco2::
          OUTPUT_PLLI2S_CLOCK ?
          /*PLLI2S*/0 :
          (__MCO2 ==
              rcc::cfgr::mco2::
              OUTPUT_PLL_CLOCK ?
              PLL :
              (__MCO2 ==
                  rcc::cfgr::mco2::
                  OUTPUT_SYSTEM_CLOCK ?
                  SYSTEM :
                  0)))
  ) / __MCO2PRE
};
static_assert(MCO1 <= 100000000,
    "MCO can't exceed 100 MHz");
static_assert(MCO2 <= 100000000,
    "MCO can't exceed 100 MHz");
#endif // STM32F1XX
#endif // USING_MCO
/****************************************************************************
 *                                                                          *
 *                        FLASH MEMORY ACCESS LATENCY                       *
 *                                                                          *
 ****************************************************************************/
#if defined STM32F1XX && \
    not defined VALUE_LINE
static_assert((SYSTEM < 24000000) ||
    (__LATENCY >=
        flash::acr::latency::ONE_WAIT_STATE),
    "For this system clock frequency, the latency must be 1 or higher.");
static_assert((SYSTEM < 48000000) ||
    (__LATENCY >=
        flash::acr::latency::TWO_WAIT_STATES),
    "For this system clock frequency, the latency must be 2 or higher.");
#endif // STM32F1XX && !VALUE_LINE
/****************************************************************************
 *                                                                          *
 *                           CLOCK INITIALIZATION                           *
 *                                                                          *
 ****************************************************************************/
#if defined USING_HSE_CLOCK || \
    defined USING_HSE_CRYSTAL
static inline void initializeHse()
{
#ifdef USING_HSE_CRYSTAL
  RCC::useHseOscillator();
#endif // USING_HSE_CRYSTAL
#ifdef USING_HSE_CLOCK
  RCC::bypassHseOscillator();
#endif // USING_HSE_CLOCK
  RCC::enableHse();

  u16 HseTimeoutCounter = 0;
  do {
    HseTimeoutCounter++;
  }while ((HseTimeoutCounter != clk::__HSE_TIMEOUT) &&
      (!RCC::isHseStable()));
}
#endif // USING_HSE_CLOCK || USING_HSE_CRYSTAL
#if defined USING_LSE_CLOCK || \
    defined USING_LSE_CRYSTAL
static inline void initializeLse()
{
#ifdef USING_HSE_CRYSTAL
  RCC::useLseOscillator();
#endif // USING_HSE_CRYSTAL
#ifdef USING_HSE_CLOCK
  RCC::bypassLseOscillator();
#endif // USING_HSE_CLOCK
  RCC::enableLse();

  u16 LseTimeoutCounter = 0;
  do {
    LseTimeoutCounter++;
  }while ((LseTimeoutCounter != clk::__LSE_TIMEOUT) &&
      (!RCC::isLseStable()));
}
#endif // USING_HSE_CLOCK || USING_HSE_CRYSTAL
void initialize()
{
#if defined USING_LSE_CRYSTAL || \
    defined USING_LSE_CLOCK || \
    defined USING_RTC
  RCC::enableClocks<
  rcc::apb1enr::PWR
  >();
  PWR::enableBackupDomainWriteProtection();
#endif // USING_LSE_CRYSTAL || USING_LSE_CLOCK || USING_RTC
  /* Low Speed Internal Clock ***********************************************/
#ifdef USING_LSI
  RCC::enableLsi();

  while (!RCC::isLsiStable()) {}
#endif // USING_LSI
  /* Low Speed External Clock ***********************************************/
#if defined USING_LSE_CLOCK || \
    defined USING_LSE_CRYSTAL
  initializeLse();

  if (!RCC::isLseStable()) {
    lseFailureHandler();
  }
#endif // USING_LSE_CLOCK || USING_LSE_CRYSTAL
  /* High Speed External Clock **********************************************/
#if defined USING_HSE_CLOCK || \
    defined USING_HSE_CRYSTAL
  initializeHse();
#endif // USING_HSE_CLOCK || USING_HSE_CRYSTAL
#if defined USING_HSE_CLOCK || \
    defined USING_HSE_CRYSTAL
  if (RCC::isHseStable()) {
#endif // USING_HSE_CLOCK || USING_HSE_CRYSTAL
  /* PLL configuration ******************************************************/
#ifdef USING_PLL
#ifdef STM32F1XX
#ifdef VALUE_LINE
  RCC::configurePll<
  __PLLSRC,
  __PLLMUL,
  __PREDIV1
  >();
#else // VALUE_LINE
#ifndef CONNECTIVITY_LINE
  RCC::configurePll<
  __PLLSRC,
  __PLLXTPRE,
  __PLLMUL
  >();
#else // !CONNECTIVITY_LINE
  RCC::configurePll<
  __PLLSRC,
  __PLLMUL,
  __PREDIV1,
  __PREDIV2,
  __PLL2MUL,
#ifdef USING_I2S_PLL
  __PLL3MUL,
#else // USING_I2S_PLL
  6, /* Any value */
#endif
  __PREDIV1SRC,
#ifdef USING_I2S
  __I2S2SRC,
  __I2S3SRC
#else // USING_I2S
  rcc::cfgr2::i2s2src::
  USE_SYSTEM_CLOCK_AS_I2S2_CLOCK, /* Any value */
  rcc::cfgr2::i2s3src::
  USE_PLL3_CLOCK_AS_I2S3_CLOCK /* Any value */
#endif // USING_I2S
  >();
#endif // !CONNECTIVITY_LINE
#endif // VALUE_LINE
#else // STM32F1XX
  RCC::configurePll<
  __PLLSRC,
  __PLLM,
  __PLLN,
  __PLLP,
  __PLLQ
  >();
#ifdef USING_I2S_PLL
  RCC::configureI2sPll<
  __PLLI2SN,
  __PLLI2SR
  >();
#endif // USING_I2S_PLL
#endif // STM32F1XX
#else // USING_PLL
#ifdef USING_I2S_PLL
  RCC::configurePll<
  rcc::cfgr::pllsrc::
  USE_HSI_CLOCK_OVER_2_AS_PLL_SOURCE, /* Any value */
  2, /* Any value */
  1, /* Any value */
  __PREDIV2,
  6, /* Any value */
  __PLL3MUL,
  rcc::cfgr2::prediv1src::
  USE_PLL2_AS_PREDIV1_INPUT, /* Any value */
  __I2S2SRC,
  __I2S3SRC
  >();
  RCC::selectI2sSource<
  rcc::cfgr::i2ssrc::PLLI2S_USED_AS_I2S_CLOCK_SOURCE
  >();
#else // USING_I2S_PLL
#ifndef STM32F1XX
  RCC::selectI2sSource(
      rcc::cfgr::i2ssrc::
      I2S_CKIN_USED_AS_I2S_CLOCK_SOURCE);
#endif // !STM32F1XX
#endif // USING_I2S_PLL
#endif // USING_PLL
  /* Microcontroller Clock Output *******************************************/
#ifdef USING_MCO
#ifdef STM32F1XX
  RCC::configureClockOutput<
  __MCO
  >();
#else // STM32F1XX
  RCC::configureClockOutput<
  __MCO1,
  __MCO2,
  __MCO1PRE + 0b10,
  __MCO2PRE + 0b10
  >();
#endif // STM32F1XX
#endif // USING_MCO
  /* Flash latency **********************************************************/
#ifdef STM32F1XX
#ifdef VALUE_LINE
  FLASH::configure(flash::acr::hlfcya::
      FLASH_HALF_CYCLE_ACCESS_ENABLED);
#else // VALUE_LINE
  FLASH::configure(
      __LATENCY,
      flash::acr::hlfcya::
      FLASH_HALF_CYCLE_ACCESS_ENABLED,
      flash::acr::prftbe::
      PREFETCH_ENABLED);
#endif // VALUE_LINE
#else // STM32F1XX
  FLASH::configure(
      __LATENCY,
      flash::acr::prften::PREFETCH_ENABLED,
      flash::acr::dcen::DATA_CACHE_ENABLED,
      flash::acr::icen::INSTRUCTION_CACHE_ENABLED);
#endif // STM32F1XX
  /* Bus prescalers *********************************************************/
#ifdef STM32F1XX
#ifdef VALUE_LINE
  RCC::configureBusPrescalers<
  __HPRE + 0b111,
  __PPRE1 + 0b11,
  __PPRE2 + 0b11,
  __ADCPRE
  >();
#else // VALUE_LINE
#ifndef CONNECTIVITY_LINE
  RCC::configureBusPrescalers<
  __HPRE + 0b111,
  __PPRE1 + 0b11,
  __PPRE2 + 0b11,
  __ADCPRE,
  __USBPRE
  >();
#else // !CONNECTIVITY_LINE
#endif // !CONNECTIVITY_LINE
#endif // VALUE_LINE
#else // STM32F1XX
  RCC::configurePrescalers<
      __HPRE + 0b111,
      __PPRE1 + 0b11,
      __PPRE2 + 0b11,
      #ifdef USING_RTC
      __RTCPRE
#else // USING_RTC
      0
  #endif // USING_RTC
  >();
#endif // STM32F1XX
  /* RTC configuration ******************************************************/
#ifdef USING_RTC
  RCC::setRtcClockSource(__RTCSEL);

  RCC::enableRtc();
#endif // USING_RTC
  /* Enable PLLs ************************************************************/
#ifdef USING_PLL
#ifdef STM32F1XX
  RCC::enablePll();

  while (!RCC::isPllStable()) {}
#ifdef CONNECTIVITY_LINE
  RCC::enablePll2();

  while (!RCC::isPll2Stable()) {}
#endif // !CONNECTIVITY_LINE
#else // STM32F1XX
  RCC::enablePll();

  while (!RCC::isPllStable()) {}
#endif // STM32F1XX
#endif // USING_PLL
#ifdef CONNETIVITY_LINE
#ifdef USING_I2S_PLL
  RCC::enablePll3();

  while (!RCC::isPll3Stable()) {}
#endif // USING_I2S_PLL
#endif // CONNECTIVITY_LINE
  /* Select system clock ****************************************************/
  RCC::setSystemClockSource(__SW);

  while (!RCC::isSystemClockSourceStable()) {
  }
#if defined USING_HSE_CLOCK || \
    defined USING_HSE_CRYSTAL
} else {
  hseFailureHandler();
}
#endif // USING_HSE_CLOCK || USING_HSE_CRYSTAL
}
}  // namespace clk
