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

#include "bitband.hpp"
#include "cfunctions.hpp"

namespace rcc {

  /**
   * @brief Enables the external oscillator circuitry.
   */
  void Functions::enableHse()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::hseon::POSITION
    >()) = 1;
  }

  /**
   * @brief Disables the external oscillator circuitry.
   */
  void Functions::disableHse()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::hseon::POSITION
    >()) = 0;
  }

  /**
   * @brief Returns true if the HSE oscillator is stable.
   */
  bool Functions::isHseStable()
  {
    return *(bool volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::hserdy::POSITION
    >());
  }

  /**
   * @brief Uses the HSE oscillator circuitry.
   */
  void Functions::useHseOscillator()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::hsebyp::POSITION
    >()) = 0;
  }

  /**
   * @brief Bypasses the HSE oscillator circuitry with an external clock.
   */
  void Functions::bypassHseOscillator()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::hsebyp::POSITION
    >()) = 1;
  }

  /**
   * @brief Enables the external oscillator circuitry.
   */
  void Functions::enableLse()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + bdcr::OFFSET,
        bdcr::lseon::POSITION
    >()) = 1;
  }

  /**
   * @brief Disables the external oscillator circuitry.
   */
  void Functions::disableLse()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + bdcr::OFFSET,
        bdcr::lseon::POSITION
    >()) = 0;
  }

  /**
   * @brief Returns true if the HSE oscillator is stable.
   */
  bool Functions::isLseStable()
  {
    return *(bool volatile*) (bitband::peripheral<
        ADDRESS + bdcr::OFFSET,
        bdcr::lserdy::POSITION
    >());
  }

  /**
   * @brief Uses the LSE oscillator circuitry.
   */
  void Functions::useLseOscillator()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + bdcr::OFFSET,
        bdcr::lsebyp::POSITION
    >()) = 0;
  }

  /**
   * @brief Bypasses the LSE oscillator circuitry with an external clock.
   */
  void Functions::bypassLseOscillator()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + bdcr::OFFSET,
        bdcr::lsebyp::POSITION
    >()) = 1;
  }

  /**
   * @brief Enables the HSI oscillator.
   */
  void Functions::enableHsi()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::hsion::POSITION
    >()) = 1;
  }

  /**
   * @brief Disables the HSI oscillator.
   */
  void Functions::disableHsi()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::hsion::POSITION
    >()) = 0;
  }

  /**
   * @brief Returns true if the HSI oscillator is stable.
   */
  bool Functions::isHsiStable()
  {
    return *(bool volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::hsirdy::POSITION
    >());
  }

  /**
   * @brief Enables the HSI oscillator.
   */
  void Functions::enableLsi()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + csr::OFFSET,
        csr::lsion::POSITION
    >()) = 1;
  }

  /**
   * @brief Enables the HSI oscillator.
   */
  void Functions::disableLsi()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + csr::OFFSET,
        csr::lsion::POSITION
    >()) = 0;
  }

  /**
   * @brief Enables the HSI oscillator.
   */
  bool Functions::isLsiStable()
  {
    return *(bool volatile*) (bitband::peripheral<
        ADDRESS + csr::OFFSET,
        csr::lsirdy::POSITION
    >());
  }

  /**
   * @brief Enables the Real Time Clock.
   */
  void Functions::enableRtc()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + bdcr::OFFSET,
        bdcr::rtcen::POSITION
    >()) = 1;
  }

  /**
   * @brief Disables the Real Time Clock.
   */
  void Functions::disableRtc()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + bdcr::OFFSET,
        bdcr::rtcen::POSITION
    >()) = 0;
  }

  /**
   * @brief Enables the PLL circuitry.
   */
  void Functions::enablePll()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::pllon::POSITION
    >()) = 1;
  }

  /**
   * @brief Disables the PLL circuitry.
   */
  void Functions::disablePll()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::pllon::POSITION
    >()) = 0;
  }

  /**
   * @brief Returns true if the PLL is stable.
   */
  bool Functions::isPllStable()
  {
    return *(bool volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::pllrdy::POSITION
    >());
  }

#ifdef CONNECTIVITY_LINE
  /**
   * @brief Enables the PLL2 circuitry.
   */
  void Functions::enablePll2()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::pll2on::POSITION
        >()) = 1;
  }

  /**
   * @brief Disables the PLL2 circuitry.
   */
  void Functions::disablePll2()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::pll2on::POSITION
        >()) = 0;
  }

  /**
   * @brief Returns true if the PLL2 is stable.
   */
  bool Functions::isPll2Stable()
  {
    return *(bool volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::pll2rdy::POSITION
        >());
  }

  /**
   * @brief Enables the PLL3 circuitry.
   */
  void Functions::enablePll3()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::pllon::POSITION
        >()) = 1;
  }

  /**
   * @brief Disables the PLL3 circuitry.
   */
  void Functions::disablePll3()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::pllon::POSITION
        >()) = 0;
  }

  /**
   * @brief Returns true if the PLL3 is stable.
   */
  bool Functions::isPll3Stable()
  {
    return *(bool volatile*) (bitband::peripheral<
        ADDRESS + cr::OFFSET,
        cr::pll3rdy::POSITION
        >());
  }
#endif

  /**
   * @brief Tests if the system clock source is stable.
   */
  bool Functions::isSystemClockSourceStable()
  {
    return (((RCC_REGS->CFGR & cfgr::sw::MASK)
        >> cfgr::sw::POSITION)
        ==
        ((RCC_REGS->CFGR & cfgr::sws::MASK)
            >> cfgr::sws::POSITION));
  }

  /**
   * @brief Selects the system clock source.
   */
  void Functions::setSystemClockSource(cfgr::sw::States SW)
  {
    RCC_REGS->CFGR &= ~cfgr::sw::MASK;
    RCC_REGS->CFGR |= SW;
  }

  /**
   * @brief Selects the RTC clock source.
   */
  void Functions::setRtcClockSource(bdcr::rtcsel::States RTCSEL)
  {
    RCC_REGS->BDCR &= ~bdcr::rtcsel::MASK;
    RCC_REGS->BDCR |= RTCSEL;
  }

  /**
   * @brief Enables these peripherals. (APB1)
   */
  template<
      apb1enr::Bits ... APB1ENR
  >
  void Functions::enableClocks()
  {
    RCC_REGS->APB1ENR |= cSum<APB1ENR...>::value;
  }

  /**
   * @brief Disables these peripherals. (APB1)
   */
  template<
      apb1enr::Bits ... APB1ENR
  >
  void Functions::disableClocks()
  {
    RCC_REGS->APB1ENR &= ~cSum<APB1ENR...>::value;
  }

  /**
   * @brief Resets these peripherals. (APB1)
   */
  template<
      apb1rstr::Bits ... APB1RSTR
  >
  void Functions::resetPeripherals()
  {
    RCC_REGS->APB1RSTR |= cSum<APB1RSTR...>::value;
    RCC_REGS->APB1RSTR &= ~cSum<APB1RSTR...>::value;
  }

  /**
   * @brief Enables these peripherals. (APB2)
   */
  template<
      apb2enr::Bits ... APB2ENR
  >
  void Functions::enableClocks()
  {
    RCC_REGS->APB2ENR |= cSum<APB2ENR...>::value;
  }

  /**
   * @brief Disables these peripherals. (APB2)
   */
  template<
      apb2enr::Bits ... APB2ENR
  >
  void Functions::disableClocks()
  {
    RCC_REGS->APB2ENR &= ~cSum<APB2ENR...>::value;
  }

  /**
   * @brief Resets these peripherals. (APB2)
   */
  template<
      apb2rstr::Bits ... APB2RSTR
  >
  void Functions::resetPeripherals()
  {
    RCC_REGS->APB2RSTR |= cSum<APB2RSTR...>::value;
    RCC_REGS->APB2RSTR &= ~cSum<APB2RSTR...>::value;
  }

#ifdef STM32F1XX

  /**
   * @brief Enables these peripherals. (AHB)
   */
  template<
  ahbenr::Bits ... AHBENR
  >
  void Functions::enableClocks()
  {
    RCC_REGS->AHBENR |= cSum<AHBENR...>::value;
  }

  /**
   * @brief Disables these peripherals. (AHB)
   */
  template<
  ahbenr::Bits ... AHBENR
  >
  void Functions::disableClocks()
  {
    RCC_REGS->AHBENR &= ~cSum<AHBENR...>::value;
  }

  void Functions::configureClockOutput(cfgr::mco::States MCO)
  {
    RCC_REGS->CFGR &= ~(cfgr::mco::MASK);

    RCC_REGS->CFGR |= MCO;
  }

#ifdef CONNECTIVITY_LINE
  /**
   * @brief Resets these peripherals. (AHB)
   */
  template<
  ahbrstr::Bits ... AHBRSTR
  >
  void Functions::resetPeripherals()
  {
    RCC_REGS->AHBRSTR |= cSum<AHBRSTR...>::value;
    RCC_REGS->AHBRSTR &= ~cSum<AHBRSTR...>::value;
  }
#endif

#ifdef VALUE_LINE

  /**
   * @brief Configures the various bus prescalers.
   * @note  Overrides the old configuration.
   */
  template<
  u8 HPRE,
  u8 PPRE1,
  u8 PPRE2,
  u8 ADCPRE
  >
  void Functions::configureBusPrescalers()
  {
    static_assert(HPRE < 16,
        "The AHB prescaler (HPRE) must be lower than 16. (inclusive)");

    static_assert(PPRE1 < 8,
        "The APB1 prescaler (PPRE1) must be lower than 8. (inclusive)");

    static_assert(PPRE2 < 8,
        "The APB2 prescaler (PPRE2) must be lower than 8. (inclusive)");

    static_assert(ADCPRE < 4,
        "The ADC prescaler (ADCPRE) must be lower than 3. (inclusive)");

    RCC_REGS->CFGR &=
    cfgr::sw::MASK +
    cfgr::pllsrc::MASK +
    cfgr::pllxtpre::MASK +
    cfgr::pllmul::MASK +
    cfgr::mco::MASK;

    RCC_REGS->CFGR |=
    (HPRE << cfgr::hpre::POSITION) +
    (PPRE1 << cfgr::ppre1::POSITION) +
    (PPRE2 << cfgr::ppre2::POSITION) +
    (ADCPRE << cfgr::adcpre::POSITION);
  }

#else // VALUE_LINE
  /**
   * @brief Configures the various bus prescalers.
   * @note  Overrides the old configuration.
   */
  template<
  u8 HPRE,
  u8 PPRE1,
  u8 PPRE2,
  u8 ADCPRE,
  u8 USBPRE
  >
  void Functions::configureBusPrescalers()
  {
    static_assert(HPRE < 16,
        "The AHB prescaler (HPRE) must be lower than 16. (inclusive)");

    static_assert(PPRE1 < 8,
        "The APB1 prescaler (PPRE1) must be lower than 8. (inclusive)");

    static_assert(PPRE2 < 8,
        "The APB2 prescaler (PPRE2) must be lower than 8. (inclusive)");

    static_assert(ADCPRE < 4,
        "The ADC prescaler (ADCPRE) must be lower than 3. (inclusive)");

    static_assert(USBPRE < 2,
        "The USB prescaler can only be 0 or 1.");

    RCC_REGS->CFGR &=
    cfgr::sw::MASK +
    cfgr::pllsrc::MASK +
    cfgr::pllxtpre::MASK +
    cfgr::pllmul::MASK +
    cfgr::mco::MASK;
    RCC_REGS->CFGR |=
    (HPRE << cfgr::hpre::POSITION) +
    (PPRE1 << cfgr::ppre1::POSITION) +
    (PPRE2 << cfgr::ppre2::POSITION) +
    (ADCPRE << cfgr::adcpre::POSITION) +
    (USBPRE << cfgr::usbpre::POSITION);
  }

#endif // VALUE_LINE
#ifdef VALUE_LINE

  /**
   * @brief Configures the various PLL prescalers and multipliers.
   * @note  The PLL circuitry must be off during configuration
   * @note  Overrides the old configuration.
   */
  template<
  cfgr::pllsrc::States PLLSRC,
  u8 PLLMUL,
  u8 PREDIV1
  >
  void Functions::configurePll()
  {
    static_assert(PLLMUL < 16,
        "PLLMUL must be between 0 and 15. (inclusive)");
    static_assert(PREDIV1 < 16,
        "PREDIV1 must be between 0 and 15. (inclusive)");

    RCC_REGS->CFGR &=
    ~(cfgr::pllsrc::MASK +
        cfgr::pllmul::MASK +
        cfgr::pllxtpre::MASK);

    RCC_REGS->CFGR |=
    PLLSRC +
    (PLLMUL << cfgr::pllmul::POSITION);

    RCC_REGS->CFGR2 =
    (PREDIV1 << cfgr2::prediv1::POSITION);
  }
#else // VALUE_LINE
#ifdef CONNECTIVITY_LINE

  /**
   * @brief Configures the various PLL prescalers and multipliers.
   * @note  The PLL circuitry must be off during configuration!
   * @note  Overrides the old configuration.
   */
  template<
  cfgr::pllsrc::States PLLSRC,
  u8 PLLMUL,
  u8 PREDIV1,
  u8 PREDIV2,
  u8 PLL2MUL,
  u8 PLL3MUL,
  cfgr2::prediv1src::States PREDIV1SRC,
  cfgr2::i2s2src::States I2S2SRC,
  cfgr2::i2s3src::States I2S3SRC
  >
  void Functions::configurePll()
  {
    static_assert(((PLLMUL >= 2) && (PLLMUL <= 7)) || (PLLMUL == 13),
        "PLLMUL value can only take these values: 2, 3, 4, 5, 6, 7, 13");
    static_assert((PREDIV1 <= 16),
        "PREDIV must be between 0 and 15. (inclusive)");
    static_assert((PREDIV2 <= 16),
        "PREDIV must be between 0 and 15 (inclusive)");
    static_assert(PLL2MUL >= 6 && PLL2MUL <= 15 && (PLL2MUL != 13),
        "PLL2MUL must be between 6 and 15 (inclusive) and not 13.");
    static_assert(PLL3MUL >= 6 && PLL3MUL <= 15 && (PLL3MUL != 13),
        "PLL3MUL must be between 6 and 15 (inclusive) and not 13.");

    RCC_REGS->CFGR &=
    ~(cfgr::pllsrc::MASK +
        cfgr::pllmul::MASK +
        cfgr::pllxtpre::MASK);
    RCC_REGS->CFGR |=
    PLLSRC +
    (PLLMUL << cfgr::pllmul::POSITION);
    RCC_REGS->CFGR2 =
    (PREDIV1 << cfgr2::prediv1::POSITION) +
    (PREDIV2 << cfgr2::prediv2::POSITION) +
    (PLL2MUL << cfgr2::pll2mul::POSITION) +
    (PLL3MUL << cfgr2::pll3mul::POSITION) +
    PREDIV1SRC + I2S2SRC + I2S3SRC;
  }

#else // CONNECTIVITY_LINE
  /**
   * @brief Configures the various PLL prescalers and multipliers.
   * @note  The PLL circuitry must be off during the configuration
   * @note  Overrides the old configuration.
   */
  template<
  cfgr::pllsrc::States PLLSRC,
  u8 PLLXTPRE,
  u8 PLLMUL
  >
  void Functions::configurePll()
  {
    static_assert(PLLXTPRE < 2,
        "PLLXTPRE can only take these values: 0 or 1");
    static_assert(PLLMUL < 16,
        "PLLMUL must be between 0 and 15. (inclusive)");

    RCC_REGS->CFGR &=
    ~(cfgr::pllsrc::MASK +
        cfgr::pllxtpre::MASK +
        cfgr::pllmul::MASK);
    RCC_REGS->CFGR |=
    PLLSRC +
    (PLLXTPRE << cfgr::pllxtpre::POSITION) +
    (PLLMUL << cfgr::pllmul::POSITION);
  }
#endif // CONNECTIVITY_LINE
#endif // VALUE_LINE
#else // STM32F1XX
  /**
   * @brief Enables these peripherals. (AHB1)
   */
  template<
      ahb1enr::Bits ... AHB1ENR
  >
  void Functions::enableClocks()
  {
    RCC_REGS->AHB1ENR |= cSum<AHB1ENR...>::value;
  }

  /**
   * @brief Disables these peripherals. (AHB1)
   */
  template<
      ahb1enr::Bits ... AHB1ENR
  >
  void Functions::disableClocks()
  {
    RCC_REGS->AHB1ENR &= ~cSum<AHB1ENR...>::value;
  }

  /**
   * @brief Resets these peripherals. (AHB1)
   */
  template<
      ahb1rstr::Bits ... AHB1RSTR
  >
  void Functions::resetPeripherals()
  {
    RCC_REGS->AHB1RSTR |= cSum<AHB1RSTR...>::value;
    RCC_REGS->AHB1RSTR &= ~cSum<AHB1RSTR...>::value;
  }

  /**
   * @brief Enables these peripherals. (AHB2)
   */
  template<
      ahb2enr::Bits ... AHB2ENR
  >
  void Functions::enableClocks()
  {
    RCC_REGS->AHB2ENR |= cSum<AHB2ENR...>::value;
  }

  /**
   * @brief Disables these peripherals. (AHB2)
   */
  template<
      ahb2enr::Bits ... AHB2ENR
  >
  void Functions::disableClocks()
  {
    RCC_REGS->AHB2ENR &= ~cSum<AHB2ENR...>::value;
  }

  /**
   * @brief Resets these peripherals. (AHB2)
   */
  template<
      ahb2rstr::Bits ... AHB2RSTR
  >
  void Functions::resetPeripherals()
  {
    RCC_REGS->AHB2RSTR |= cSum<AHB2RSTR...>::value;
    RCC_REGS->AHB2RSTR &= ~cSum<AHB2RSTR...>::value;
  }

  /**
   * @brief Enables these peripherals. (AHB3)
   */
  template<
      ahb3enr::Bits ... AHB3ENR
  >
  void Functions::enableClocks()
  {
    RCC_REGS->AHB3ENR |= cSum<AHB3ENR...>::value;
  }

  /**
   * @brief Disables these peripherals. (AHB3)
   */
  template<
      ahb3enr::Bits ... AHB3ENR
  >
  void Functions::disableClocks()
  {
    RCC_REGS->AHB3ENR &= ~cSum<AHB3ENR...>::value;
  }

  /**
   * @brief Resets these peripherals. (AHB3)
   */
  template<
      ahb3rstr::Bits ... AHB3RSTR
  >
  void Functions::resetPeripherals()
  {
    RCC_REGS->AHB3RSTR |= cSum<AHB3RSTR...>::value;
    RCC_REGS->AHB3RSTR &= ~cSum<AHB3RSTR...>::value;
  }

  /**
   * @brief Configures the various PLL prescalers and multipliers.
   * @note  The PLL circuitry must be off during configuration!
   * @note  Overrides the old configuration.
   */
  template<
      pllcfgr::pllsrc::States PLLSRC,
      u8 PLLM,
      u16 PLLN,
      u8 PLLP,
      u8 PLLQ
  >
  void Functions::configurePll()
  {
    static_assert((PLLM >= 2) && (PLLM <= 63),
        "PLLM must be between 2 and 63 (inclusive).");
    static_assert((PLLN >= 64) && (PLLN <= 432),
        "PLLN must be between 64 and 432 (inclusive).");
    static_assert((PLLP % 2 == 0) && (PLLP < 10),
        "PLLP can only take the following values: 2, 4, 6 or 8.");
    static_assert((PLLQ >= 2) && (PLLQ <= 15),
        "PLLQ must be between 2 and 15 (inclusive).");

    RCC_REGS->PLLCFGR =
        (PLLM << pllcfgr::pllm::POSITION) +
            (PLLN << pllcfgr::plln::POSITION) +
            (((PLLP / 2) - 1) << pllcfgr::pllp::POSITION) +
            (PLLQ << pllcfgr::pllq::POSITION) +
            PLLSRC
            ;
  }

  /**
   * @brief Configures the various I2S PLL prescalers and multipliers.
   * @note  The PLL circuitry must be off during configuration!
   * @note  Overrides the old configuration.
   */
  template<
      u16 PLLI2SN,
      u8 PLLI2SR
  >
  void Functions::configureI2sPll()
  {
    RCC_REGS->PLLCFGR |= cfgr::i2ssrc::PLLI2S_USED_AS_I2S_CLOCK_SOURCE;

    RCC_REGS->PLLI2SCFGR =
        (PLLI2SN << plli2scfgr::plli2sn::POSITION) +
            (PLLI2SR << plli2scfgr::plli2sr::POSITION);
  }

  /**
   * @brief Configures the various bus prescalers.
   * @note  Overrides the old configuration.
   */
  template<
      u8 HPRE,
      u8 PPRE1,
      u8 PPRE2,
      u8 RTCPRE
  >
  void Functions::configurePrescalers()
  {
    static_assert(HPRE < 16,
        "The AHB prescaler (HPRE) must be lower than 16. (inclusive)");

    static_assert(PPRE1 < 8,
        "The APB1 prescaler (PPRE1) must be lower than 8. (inclusive)");

    static_assert(PPRE2 < 8,
        "The APB2 prescaler (PPRE2) must be lower than 8. (inclusive)");

    static_assert(RTCPRE < 32,
        "The RTC prescaler (RTCPRE) must be lower than 32. (inclusive)");

    RCC_REGS->CFGR &=
        ~(cfgr::hpre::MASK +
            cfgr::ppre1::MASK +
            cfgr::ppre2::MASK +
            cfgr::rtcpre::MASK)
        ;
    RCC_REGS->CFGR |=
        (HPRE << cfgr::hpre::POSITION) +
            (PPRE1 << cfgr::ppre1::POSITION) +
            (PPRE2 << cfgr::ppre2::POSITION) +
            (RTCPRE << cfgr::rtcpre::POSITION)
            ;
  }

  template<
      cfgr::mco1::States MCO1,
      cfgr::mco2::States MCO2,
      u8 MCOPRE1,
      u8 MCOPRE2
  >
  void Functions::configureClockOutput()
  {
    static_assert(MCOPRE1 < 0b1000,
        "MCOPRE1 can't exceed 0b1000");

    static_assert(MCOPRE2 < 0b1000,
        "MCOPRE2 can't exceed 0b1000");

    RCC_REGS->CFGR &= ~(cfgr::mco1::MASK +
        cfgr::mco2::MASK +
        cfgr::mco1pre::MASK +
        cfgr::mco2pre::MASK);

    RCC_REGS->CFGR |= MCO1 + MCO2 +
        (MCOPRE1 << cfgr::mco1pre::POSITION) +
        (MCOPRE2 << cfgr::mco2pre::POSITION);
  }

  void Functions::selectI2sSource(cfgr::i2ssrc::States I2SSRC)
  {
    RCC_REGS->CFGR &= cfgr::i2ssrc::MASK;

    RCC_REGS->CFGR |= I2SSRC;
  }

#endif // STM32F1XX
}  // namespace rcc

