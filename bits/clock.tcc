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
#include "../include/peripheral/rcc.hpp"
#include "../include/peripheral/flash.hpp"

namespace clock {

  /*****************************************************************************
   *
   *             Verify the correctness of the clock configuration
   *
   ****************************************************************************/

  // Checking the clock source correctness
#if defined USING_EXTERNAL_CRYSTAL || \
    defined USING_EXTERNAL_CLOCK
  enum {
    _HSE_TIMEOUT = 0x0500
  };
#if defined STM32F1XX
#ifdef VALUE_LINE
  static_assert((_SOURCE >= 4000000) && (_SOURCE <= 24000000),
      " The external clock frequency must be between 4 MHz and 16 MHz");
#elif defined CONNECTIVITY_LINE
  static_assert((_SOURCE >= 3000000) && (_SOURCE <= 25000000),
      " The external clock frequency must be between 3 MHz and 25 MHz");
#else
  static_assert((_SOURCE >= 4000000) && (_SOURCE <= 16000000),
      " The external clock frequency must be between 4 MHz and 16 MHz");
#endif
#else // STM32F1XX
  static_assert((_SOURCE >= 4000000) && (_SOURCE <= 26000000),
      " The external clock frequency must be between 4 MHz and 26 MHz");
#endif // STM32F1XX
#else // USING_EXTERNAL_CRYSTAL || USING_EXTERNAL_CLOCK
#ifdef STM32F1XX
  enum {_SOURCE = 8000000};
#else // STM32F1XX
  enum {
    _SOURCE = 16000000
  };
  //!< _SOURCE
#endif // STM32F1XX
#endif // USING_EXTERNAL_CRYSTAL || USING_EXTERNAL_CLOCK
  // Checking the PLL correctness
#ifdef USING_PLL
#ifdef STM32F1XX
#if defined USING_EXTERNAL_CLOCK || \
    defined USING_EXTERNAL_CRYSTAL
#ifdef VALUE_LINE
  enum {
    SYSTEM = _SOURCE * (2 + _PLLMUL) / (_PREDIV1 + 1)
  };
#elif defined CONNECTIVITY_LINE
  enum {
    SYSTEM = _SOURCE *
    ((_PLLMUL == 13)? (_PLLMUL / 2):(_PLLMUL + 2)) *
    ((_PLL2MUL == 15)? (_PLL2MUL + 5):(_PLL2MUL + 2)) /
    ((_PREDIV1 + 1) * (_PREDIV2 + 1))
  };
#ifdef USING_I2S_PLL
  enum {
    I2S = _SOURCE *
    ((_PLL3MUL == 15? (_PLL3MUL + 5):(_PLL3MUL + 2))) /
    (_PREDIV2 + 1),
  };
#else // USING_I2S_PLL
  enum {
    I2S = SYSTEM
  };
#endif // USING_I2S_PLL
#else // VALUE_LINE
  enum {
    SYSTEM = _SOURCE * (_PLLMUL + 2) / (1 + PLLXTPRE),
    I2S = SYSTEM
  };
#endif // VALUE_LINE
#ifdef USING_USB
  enum {
    USB = SYSTEM * 2 / (3 - _USBPRE)
  };
#endif // USING_USB
#else // EXTERNAL_CLOCK || EXTERNAL_CRYSTAL
  enum {
    SYSTEM = (_SOURCE / 2) * (_PLLMUL + 2),
    I2S = SYSTEM
  };
#endif // EXTERNAL_CLOCK || EXTERNAL_CRYSTAL
#else // STM32F1XX
  enum {
    _VCO_IN = _SOURCE / _PLLM,
    _VCO_OUT = _VCO_IN * _PLLN,
    SYSTEM = _VCO_OUT / _PLLP,
    USB = _VCO_OUT / _PLLQ,
    SDIO = _VCO_OUT / _PLLQ,
    RNG = _VCO_OUT / _PLLQ,
  };

  static_assert((_VCO_IN >= 1000000) && (_VCO_IN <= 2000000),
      "VCO_IN must be between 1MHz and 2MHZ (inclusive)");

  static_assert((_VCO_OUT >= 64000000) && (_VCO_OUT <= 432000000),
      "VCO_OUT must be between 64MHz and 432MHZ (inclusive)");

#if defined STM32F2XX
  static_assert((SYSTEM <= 120000000),
      "The System Clock can't exceed 120MHz");
#endif // STM32F2XX
#if defined STM32F4XX
  static_assert((SYSTEM <= 168000000),
      "The System Clock can't exceed 168MHz");
#endif // STM32F4XX
  static_assert(USB <= 48000000,
      "The USB/SDIO/RNG Clock can't exceed 48 MHz");

#endif // STM32F1XX
#else // USING_PLL
  enum {
    SYSTEM = _SOURCE,
  };
#endif // USING_PLL
  // Check the I2S PLL correctness
#ifdef USING_I2S_PLL

#ifdef STM32F1XX

#else // STM32F1XX
  static_assert((_PLLI2SN >= 192) || (_PLLI2SN <= 432),
      "PLLI2SN must be between 192 and 432 (inclusive).");
  static_assert((_PLLI2SR >= 2) || (_PLLI2SR <= 7),
      "PLLI2SR must be between 2 and 7 (inclusive).");

  enum {
    _VCO_I2S_IN = _SOURCE / _PLLM,
    _VCO_I2S_OUT = _VCO_I2S_IN * _PLLI2SN,
    I2S = _VCO_I2S_OUT / _PLLI2SR,
  };

  static_assert((_VCO_I2S_OUT >= 192000000) && (_VCO_I2S_OUT <= 432000000),
      "VCO_I2S_OUT must be between 192 MHz and 432 MHz (inclusive).");
  static_assert(I2S <= 192000000,
      "I2S Clock can't exceed 192 MHz.");
#endif // STM32F1XX
#endif // USING_I2S_PLL
  // Checking the bus prescalers correctness
#ifdef STM32F1XX
  enum {
    AHB = SYSTEM / cPow<2, _HPRE>::value,
    CORE = AHB,
    APB1 = AHB / cPow<2, _PPRE1>::value,
    APB2 = AHB / cPow<2, _PPRE2>::value,
    APB1_TIMERS = APB1 * ((_PPRE1 == 0)? 1:2),
    APB2_TIMERS = APB2 * ((_PPRE2 == 0)? 1:2),
    ADC = APB2 / (2*_ADCPRE + 2),
    SDIO = AHB,
    FSMC = AHB,
  };
#else // STM32F1XX
  enum {
    AHB = SYSTEM / cPow<2, _HPRE>::value,
    CORE = AHB,
    SYSTICK = AHB / 8,
    APB1 = AHB / cPow<2, _PPRE1>::value,
    APB2 = AHB / cPow<2, _PPRE2>::value,
    APB1_TIMERS = APB1 * ((_PPRE1 == 0) ? 1 : 2),
    APB2_TIMERS = APB2 * ((_PPRE2 == 0) ? 1 : 2),
  };
#endif // STM32F1XX
#ifdef STM32F1XX
#ifdef VALUE_LINE
  static_assert(AHB <= 24000000,
      "The AHB clock can't exceed 24 MHz");
  static_assert(APB1 <= 24000000,
      "The APB1 clock can't exceed 24 MHz");
  static_assert(APB2 <= 24000000,
      "The APB2 clock can't exceed 24 MHz");
  static_assert(ADC <= 12000000,
      "The ADC clock can't exceed 12 MHz");
#else // VALUE_LINE
  static_assert(AHB <= 72000000,
      "The AHB clock can't exceed 72 MHz");
  static_assert(APB1 <= 36000000,
      "The APB1 clock can't exceed 36 MHz");
  static_assert(APB2 <= 72000000,
      "The APB2 clock can't exceed 72 MHz");
  static_assert(ADC <= 14000000,
      "The ADC clock can't exceed 14 MHz");
#endif // VALUE_LINE
#else // STM32F1XX
  static_assert(AHB <= 168000000,
      "The AHB clock can't exceed 168 MHz");
  static_assert(APB1 <= 42000000,
      "The APB1 clock can't exceed 42 MHz");
  static_assert(APB2 <= 84000000,
      "The APB2 clock can't exceed 84 MHz");
#endif // STM32F1XX
// Check USB clock correctness
#ifdef USING_USB
  static_assert(USB == 48000000,
      "The USB clock must be 48 MHz, or the USB module won't work");
#endif // USING_USB
// Check the RTC prescaler correctness
#ifdef USING_HSE_FOR_RTC
  static_assert(_RTCPRE >= 2,
      "The minimum value for the RTC prescaler (RTCPRE) is 2");
  static_assert((_SOURCE / _RTCPRE) == 1000000,
      "1 MHz clock must be fed to the RTC.");
#endif /* USING_HSE_FOR_RTC */

// Check the flash access latency correctness
#ifdef STM32F1XX
  static_assert((SYSTEM < 24000000) ||
      (_LATENCY >= 1),
      "For this system clock frequency, the latency must be 1 or higher.");
  static_assert((SYSTEM < 48000000) ||
      (_LATENCY >= 2),
      "For this system clock frequency, the latency must be 2 or higher.");
#endif

#if defined USING_EXTERNAL_CRYSTAL || \
    defined USING_EXTERNAL_CLOCK
  static INLINE void initializeHse(void)
  {
#ifdef USING_EXTERNAL_CRYSTAL
    RCC::enableOscillator();
#elif defined USING_EXTERNAL_CLOCK
    RCC::disableOscillator();
#endif

    RCC::enableHse();

    u16 HseTimeoutCounter = 0;
    do {
      HseTimeoutCounter++;
    }while ((HseTimeoutCounter != clock::_HSE_TIMEOUT) &&
        (!RCC::isHseStable()));
  }
#endif /* EXTERNAL CRYSTAL || EXTERNAL CLOCK */

  /*****************************************************************************
   *
   *              Implementation of the clock::initialize function
   *
   ****************************************************************************/

#ifdef STM32F1XX
  /**
   * @brief Configures the clock, using the configuration inputted in clock.hpp
   * @note  Call this function as early as possible in you program.
   */
  void initialize()
  {
#if defined USING_EXTERNAL_CRYSTAL || \
    defined USING_EXTERNAL_CLOCK
#ifdef USING_PLL
    // External Clock + PLL
    initializeHse();

    if (RCC::isHseStable()) {
#ifdef VALUE_LINE
      RCC::configurePll<
      rcc::registers::cfgr::bits::pllsrc::states::USE_PREDIV1_OUTPUT_AS_PLL_SOURCE,
      clock::_PLLMUL,
      clock::_PREDIV1
      >();
#elif defined CONNECTIVITY_LINE
#ifdef USING_I2S_PLL
      RCC::configurePll<
      rcc::registers::cfgr::bits::pllsrc::states::USE_PREDIV1_OUTPUT_AS_PLL_SOURCE,
      clock::_PLLMUL,
      clock::_PREDIV1,
      clock::_PREDIV2,
      clock::_PLL2MUL,
      clock::_PLL3MUL,
      rcc::registers::cfgr2::bits::prediv1src::states:: USE_PLL2_AS_PREDIV1_INPUT,
      rcc::registers::cfgr2::bits::i2s2src::states:: USE_PLL3_CLOCK_AS_I2S2_CLOCK,
      rcc::registers::cfgr2::bits::i2s3src::states:: USE_PLL3_CLOCK_AS_I2S3_CLOCK
      >();

#else // USING_I2S_PLL
      RCC::configurePll<
      rcc::registers::cfgr::bits::pllsrc::states::USE_PREDIV1_OUTPUT_AS_PLL_SOURCE,
      clock::_PLLMUL,
      clock::_PREDIV1,
      clock::_PREDIV2,
      clock::_PLL2MUL,
      0,
      rcc::registers::cfgr2::bits::prediv1src::states:: USE_HSE_AS_PREDIV1_INPUT,
      rcc::registers::cfgr2::bits::i2s2src::states:: USE_SYSTEM_CLOCK_AS_I2S2_CLOCK,
      rcc::registers::cfgr2::bits::i2s3src::states:: USE_SYSTEM_CLOCK_AS_I2S3_CLOCK
      >();
#endif // USING_I2S_PLL
#else // VALUE_LINE
      RCC::configurePll<
      registers::cfgr::bits::pllsrc::states::USE_PREDIV1_OUTPUT_AS_PLL_SOURCE,
      clock::_PLLXTPRE,
      clock::_PLLMUL
      >();
#endif // VALUE_LINE
#ifdef USING_USB
      RCC::configureBusPrescalers<
      clock::_HPRE + 0b111,
      clock::_PPRE1 + 0b11,
      clock::_PPRE2 + 0b11,
      clock::_ADCPRE,
      clock::_USBPRE
      >();
#elif defined VALUE_LINE
      RCC::configureBusPrescalers<
      clock::_HPRE + 0b111,
      clock::_PPRE1 + 0b11,
      clock::_PPRE2 + 0b11,
      clock::_ADCPRE
      >();
#else  // USING_USB
      RCC::configureBusPrescalers<
      clock::_HPRE + 0b111,
      clock::_PPRE1 + 0b11,
      clock::_PPRE2 + 0b11,
      clock::_ADCPRE,
      0
      >();
#endif  // USING_USB
#ifndef VALUE_LINE
      FLASH::configure<
      clock::_LATENCY,
      flash::registers::acr::bits::hlfcya::states::FLASH_HALF_CYCLE_ACCESS_DISABLED,
      flash::registers::acr::bits::prftbe::states::PREFETCH_ENABLED
      >();
#else // VALUE_LINE
      FLASH::configure<
      flash::registers::acr::bits::hlfcya::states::FLASH_HALF_CYCLE_ACCESS_DISABLED
      >();
#endif // VALUE_LINE
#ifdef CONNECTIVITY_LINE
      RCC::enablePll2();

      while (!RCC::isPll2Stable()) {}
#ifdef USING_I2S_PLL
      RCC::enablePll3();

      while (!RCC::isPll3Stable()) {}
#endif // USING_I2S_PLL
#endif // CONNECTIVITY_LINE
      RCC::enablePll();

      while (!RCC::isPllStable()) {}

      RCC::selectSystemClockSource<
      rcc::registers::cfgr::bits::sw::states::PLL
      >();

      while (!RCC::matchSystemClockSource<
          rcc::registers::cfgr::bits::sws::states::PLL
          >()) {}
    }
    else
    clock::hseFailureHandler();

#else // USING_PLL
    // External Clock
    initializeHse();

    if (RCC::isHseStable()) {

      RCC::configureBusPrescalers<
      clock::_HPRE + 0b111,
      clock::_PPRE1 + 0b11,
      clock::_PPRE2 + 0b11,
      clock::_ADCPRE,
#ifdef USING_USB
      clock::_USBPRE
#else /* USING_USB */
      0
#endif /* USING_USB */
      >();

#ifndef VALUE_LINE
      FLASH::configure<
      clock::_LATENCY,
      flash::registers::acr::bits::hlfcya::states::FLASH_HALF_CYCLE_ACCESS_DISABLED,
      flash::registers::acr::bits::prftbe::states::PREFETCH_ENABLED
      >();
#else // VALUE_LINE
      FLASH::configure<
      flash::registers::acr::bits::hlfcya::states::FLASH_HALF_CYCLE_ACCESS_DISABLED
      >();
#endif // VALUE_LINE
      RCC::selectSystemClockSource<
      rcc::registers::cfgr::bits::sw::HSE
      >();

      while (!RCC::matchSystemClockSource<
          rcc::registers::cfgr::bits::sws::HSE
          >()) {}

    }
    else
    clock::hseFailureHandler();

#endif // USING_PLL
#else // USING_EXTERNAL_CRYSTAL || USING_EXTERNAL_CLOCK
#ifdef USING_PLL
    // Internal Clock + PLL
#ifdef VALUE_LINE
    RCC::configurePll<
    rcc::registers::cfgr::bits::pllsrc::states::USE_HSI_CLOCK_OVER_2_AS_PLL_SOURCE,
    0,
    clock::_PLLMUL
    >();
#elif defined CONNECTIVITY_LINE
    RCC::configurePll<
    rcc::registers::cfgr::bits::pllsrc::states::USE_HSI_CLOCK_OVER_2_AS_PLL_SOURCE,
    clock::_PLLMUL,
    0,
    0,
    6,
    6,
    rcc::registers::cfgr2::bits::prediv1src::states:: USE_HSE_AS_PREDIV1_INPUT,
    rcc::registers::cfgr2::bits::i2s2src::states:: USE_SYSTEM_CLOCK_AS_I2S2_CLOCK,
    rcc::registers::cfgr2::bits::i2s3src::states:: USE_SYSTEM_CLOCK_AS_I2S3_CLOCK
    >();
#else // VALUE_LINE
    RCC::configurePll<
    rcc::registers::cfgr::bits::pllsrc::states::USE_HSI_CLOCK_OVER_2_AS_PLL_SOURCE,
    0,
    clock::_PLLMUL
    >();0
#endif // VALUE_LINE
#ifdef VALUE_LINE
    RCC::configureBusPrescalers<
    clock::_HPRE + 0b111,
    clock::_PPRE1 + 0b11,
    clock::_PPRE2 + 0b11,
    clock::_ADCPRE
    >();
#else // VALUE_LINE
    RCC::configureBusPrescalers<
    clock::_HPRE + 0b111,
    clock::_PPRE1 + 0b11,
    clock::_PPRE2 + 0b11,
    clock::_ADCPRE,
    0
    >();
#endif // VALUE_LINE
#ifndef VALUE_LINE
    FLASH::configure<
    clock::_LATENCY,
    flash::registers::acr::bits::hlfcya::states::FLASH_HALF_CYCLE_ACCESS_DISABLED,
    flash::registers::acr::bits::prftbe::states::PREFETCH_ENABLED
    >();
#else // VALUE_LINE
    FLASH::configure<
    flash::registers::acr::bits::hlfcya::states::FLASH_HALF_CYCLE_ACCESS_DISABLED
    >();
#endif // VALUE_LINE
    RCC::enablePll();

    while (!RCC::isPllStable()) {}

    RCC::selectSystemClockSource<
    rcc::registers::cfgr::bits::sw::PLL
    >();

    while (!RCC::matchSystemClockSource<
        rcc::registers::cfgr::bits::sws::PLL
        >()) {}

#else // USING_PLL
    // Internal Clock

#ifdef VALUE_LINE
    RCC::configureBusPrescalers<
    clock::_HPRE + 0b111,
    clock::_PPRE1 + 0b11,
    clock::_PPRE2 + 0b11,
    clock::_ADCPRE
    >();
#else // VALUE_LINE
    RCC::configureBusPrescalers<
    clock::_HPRE + 0b111,
    clock::_PPRE1 + 0b11,
    clock::_PPRE2 + 0b11,
    clock::_ADCPRE,
    0
    >();
#endif // VALUE_LINE
#ifndef VALUE_LINE
    FLASH::configure<
    clock::_LATENCY,
    flash::registers::acr::bits::hlfcya::states::FLASH_HALF_CYCLE_ACCESS_DISABLED,
    flash::registers::acr::bits::prftbe::states::PREFETCH_ENABLED
    >();
#else // VALUE_LINE
    FLASH::configure<
    flash::registers::acr::bits::hlfcya::states::FLASH_HALF_CYCLE_ACCESS_DISABLED
    >();
#endif // VALUE_LINE
#endif // USING_PLL
#endif // USING_EXTERNAL_CRYSTAL || USING_EXTERNAL_CLOCK
  }
#else // STM32F1XX
  /**
   * @brief Configures the clock, using the configuration inputted in clock.hpp
   * @note  Call this function as early as possible in you program.
   */
  void initialize()
  {
#if defined USING_EXTERNAL_CRYSTAL || \
    defined USING_EXTERNAL_CLOCK
#ifdef USING_PLL
    // External Clock + PLL
    initializeHse();

    if (RCC::isHseStable()) {

#ifdef USING_HSE_FOR_RTC
      RCC::configureRtcPrescaler<
      clock::_RTCPRE
      >();
#endif

      RCC::configurePll<
      rcc::registers::pllcfgr::bits::pllsrc::states::HSE,
      clock::_PLLM,
      clock::_PLLN,
      clock::_PLLP,
      clock::_PLLQ
      >();

#ifdef USING_I2S_PLL
      RCC::configureI2sPll<
      clock::_PLLI2SN,
      clock::_PLLI2SR
      >();
#endif // USING_I2S_PLL
      RCC::configurePrescalers<
      clock::_HPRE + 0b111,
      clock::_PPRE1 + 0b11,
      clock::_PPRE2 + 0b11
      >();

      FLASH::configure<
      clock::_LATENCY,
      flash::registers::acr::bits::prften::states:: PREFETCH_ENABLED,
      flash::registers::acr::bits::dcen::states:: DATA_CACHE_ENABLED,
      flash::registers::acr::bits::icen::states:: INSTRUCTION_CACHE_ENABLED
      >();

      RCC::enablePll();

      while (!RCC::isPllStable()) {}

      RCC::selectSystemClockSource<
      rcc::registers::cfgr::bits::sw::states::PLL
      >();

      while (!RCC::isSystemClockSourceStable()) {}
    } else {
      clock::hseFailureHandler();
    }

#else // USING_PLL
    /* External Clock */
    initializeHse();

    if (RCC::isHseStable()) {

#ifdef USING_HSE_FOR_RTC
      RCC::configureRtcPrescaler<
      clock::_RTCPRE
      >();
#endif

      RCC::configurePrescalers<
      clock::_HPRE + 0b111,
      clock::_PPRE1 + 0b11,
      clock::_PPRE2 + 0b11
      >();

      FLASH::configure<
      clock::_LATENCY,
      flash::registers::acr::bits::prften::states:: PREFETCH_ENABLED,
      flash::registers::acr::bits::dcen::states:: DATA_CACHE_ENABLED,
      flash::registers::acr::bits::icen::states:: INSTRUCTION_CACHE_ENABLED
      >();

      RCC::selectSystemClockSource<
      rcc::registers::cfgr::bits::sw::HSE
      >();

      while (!RCC::matchSystemClockSource<
          rcc::registers::cfgr::bits::sws::HSE
          >()) {}
    }
    else
    clock::hseFailureHandler();

#endif // USING_PLL
#else // USING_EXTERNAL_CRYSTAL || USING_EXTERNAL_CLOCK
#ifdef USING_PLL
    // Internal Clock + PLL

    RCC::configurePll<
    rcc::registers::pllcfgr::bits::pllsrc::HSI,
    clock::_PLLM,
    clock::_PLLN,
    clock::_PLLP,
    clock::_PLLQ
    >();

#ifdef USING_I2S_PLL
    RCC::configureI2sPll<
    clock::_PLLI2SN,
    clock::_PLLI2SR
    >();
#endif // USING_I2S_PLL
    RCC::configurePrescalers<
    clock::_HPRE + 0b111,
    clock::_PPRE1 + 0b11,
    clock::_PPRE2 + 0b11
    >();

    FLASH::configure<
    clock::_LATENCY,
    flash::registers::acr::bits::prften::states:: PREFETCH_ENABLED,
    flash::registers::acr::bits::dcen::states:: DATA_CACHE_ENABLED,
    flash::registers::acr::bits::icen::states:: INSTRUCTION_CACHE_ENABLED
    >();

    RCC::enablePll();

    while (!RCC::isPllStable()) {}

    RCC::selectSystemClockSource<
    rcc::registers::cfgr::bits::sw::PLL
    >();

    while (!RCC::matchSystemClockSource<
        rcc::registers::cfgr::bits::sws::PLL
        >()) {}

#else // USING_PLL
    // Internal Clock

    RCC::configurePrescalers<
        clock::_HPRE + 0b111,
        clock::_PPRE1 + 0b11,
        clock::_PPRE2 + 0b11
    >();

    FLASH::configure<
        clock::_LATENCY,
        flash::registers::acr::bits::prften::states::PREFETCH_ENABLED,
        flash::registers::acr::bits::dcen::states::DATA_CACHE_ENABLED,
        flash::registers::acr::bits::icen::states::INSTRUCTION_CACHE_ENABLED
    >();

#endif // USING_PLL
#endif // USING_EXTERNAL_CRYSTAL || USING_EXTERNAL_CLOCK
  }

#endif // STM32F1XX
}  // namespace clock
