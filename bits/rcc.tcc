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
   * @brief Enables the HSI oscillator.
   */
  void Functions::enableHsi()
  {
    *(u32*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::hsion::POSITION
    >::address) = 1;
  }

  /**
   * @brief Disables the HSI oscillator.
   */
  void Functions::disableHsi()
  {
    *(u32*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::hsion::POSITION
    >::address) = 0;
  }

  /**
   * @brief Returns true if the HSI oscillator is stable.
   */
  bool Functions::isHsiStable()
  {
    return *(bool*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::hsirdy::POSITION
    >::address);
  }

  /**
   * @brief Enables the external oscillator circuitry.
   */
  void Functions::enableHse()
  {
    *(u32*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::hseon::POSITION
    >::address) = 1;
  }

  /**
   * @brief Disables the external oscillator circuitry.
   */
  void Functions::disableHse()
  {
    *(u32*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::hseon::POSITION
    >::address) = 0;
  }

  /**
   * @brief Returns true if the HSE oscillator is stable.
   */
  bool Functions::isHseStable()
  {
    return *(bool*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::hserdy::POSITION
    >::address);
  }

  /**
   * @brief Enables the PLL circuitry.
   */
  void Functions::enablePll()
  {
    *(u32*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::pllon::POSITION
    >::address) = 1;
  }

  /**
   * @brief Disables the PLL circuitry.
   */
  void Functions::disablePll()
  {
    *(u32*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::pllon::POSITION
    >::address) = 0;
  }

  /**
   * @brief Returns true if the PLL is stable.
   */
  bool Functions::isPllStable()
  {
    return *(bool*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::pllrdy::POSITION
    >::address);
  }

#ifdef CONNECTIVITY_LINE
  /**
   * @brief Enables the PLL2 circuitry.
   */
  void Functions::enablePll2()
  {
    *(u32*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::pll2on::POSITION
        >::address) = 1;
  }

  /**
   * @brief Disables the PLL2 circuitry.
   */
  void Functions::disablePll2()
  {
    *(u32*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::pll2on::POSITION
        >::address) = 0;
  }

  /**
   * @brief Returns true if the PLL2 is stable.
   */
  bool Functions::isPll2Stable()
  {
    return *(bool*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::pll2rdy::POSITION
        >::address);
  }

  /**
   * @brief Enables the PLL3 circuitry.
   */
  void Functions::enablePll3()
  {
    *(u32*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::pllon::POSITION
        >::address) = 1;
  }

  /**
   * @brief Disables the PLL3 circuitry.
   */
  void Functions::disablePll3()
  {
    *(u32*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::pllon::POSITION
        >::address) = 0;
  }

  /**
   * @brief Returns true if the PLL3 is stable.
   */
  bool Functions::isPll3Stable()
  {
    return *(bool*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::pll3rdy::POSITION
        >::address);
  }
#endif

  /**
   * @brief Selects the system clock source.
   */
  template<
      registers::cfgr::bits::sw::states::E SW
  >
  void Functions::selectSystemClockSource()
  {
    _RCC->CFGR &= ~registers::cfgr::bits::sw::MASK;
    _RCC->CFGR |= SW;
  }

  /**
   * @brief Tests if the system clock source is stable.
   */
  bool Functions::isSystemClockSourceStable()
  {
    return (((_RCC->CFGR & registers::cfgr::bits::sw::MASK)
        >> registers::cfgr::bits::sw::POSITION)
        ==
        ((_RCC->CFGR & registers::cfgr::bits::sws::MASK)
            >> registers::cfgr::bits::sws::POSITION));
  }

  /**
   * @brief Enables oscillator circuitry for external crystal/resonator.
   */
  void Functions::enableOscillator()
  {
    *(u32*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::hsebyp::POSITION
    >::address) = 0;
  }

  /**
   * @brief Bypasses the oscillator with an external clock.
   */
  void Functions::disableOscillator()
  {
    *(u32*) (bitband::Peripheral<
        ADDRESS + registers::cr::OFFSET,
        registers::cr::bits::hsebyp::POSITION
    >::address) = 1;
  }

  /**
   * @brief Enables these peripherals. (APB1)
   */
  template<
      registers::apb1enr::bits::E ... APB1ENR
  >
  void Functions::enableClocks()
  {
    _RCC->APB1ENR |= cSum<APB1ENR...>::value;
  }

  /**
   * @brief Disables these peripherals. (APB1)
   */
  template<
      registers::apb1enr::bits::E ... APB1ENR
  >
  void Functions::disableClocks()
  {
    _RCC->APB1ENR &= ~cSum<APB1ENR...>::value;
  }

  /**
   * @brief Resets these peripherals. (APB1)
   */
  template<
      registers::apb1rstr::bits::E ... APB1RSTR
  >
  void Functions::resetPeripherals()
  {
    _RCC->APB1RSTR |= cSum<APB1RSTR...>::value;
  }

  /**
   * @brief Enables these peripherals. (APB2)
   */
  template<
      registers::apb2enr::bits::E ... APB2ENR
  >
  void Functions::enableClocks()
  {
    _RCC->APB2ENR |= cSum<APB2ENR...>::value;
  }

  /**
   * @brief Disables these peripherals. (APB2)
   */
  template<
      registers::apb2enr::bits::E ... APB2ENR
  >
  void Functions::disableClocks()
  {
    _RCC->APB2ENR &= ~cSum<APB2ENR...>::value;
  }

  /**
   * @brief Resets these peripherals. (APB2)
   */
  template<
      registers::apb2rstr::bits::E ... APB2RSTR
  >
  void Functions::resetPeripherals()
  {
    _RCC->APB2RSTR = cSum<APB2RSTR...>::value;
  }

#ifdef STM32F1XX

  /**
   * @brief Enables these peripherals. (AHB)
   */
  template<
  registers::ahbenr::bits::E ... AHBENR
  >
  void Functions::enableClocks()
  {
    _RCC->AHBENR |= cSum<AHBENR...>::value;
  }

  /**
   * @brief Disables these peripherals. (AHB)
   */
  template<
  rcc::registers::ahbenr::bits::E ... AHBENR
  >
  void Functions::disableClocks()
  {
    _RCC->AHBENR &= ~cSum<AHBENR...>::value;
  }

  /**
   * @brief Resets these peripherals. (AHB)
   */
  template<
  rcc::registers::ahbrstr::bits::E ... AHBRSTR
  >
  void Functions::resetPeripherals()
  {
    _RCC->AHBRSTR = cSum<AHBRSTR...>::value;
  }

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

    _RCC->CFGR &=
    registers::cfgr::bits::sw::MASK +
    registers::cfgr::bits::pllsrc::MASK +
    registers::cfgr::bits::pllxtpre::MASK +
    registers::cfgr::bits::pllmul::MASK +
    registers::cfgr::bits::mco::MASK;

    _RCC->CFGR |=
    (HPRE << registers::cfgr::bits::hpre::POSITION) +
    (PPRE1 << registers::cfgr::bits::ppre1::POSITION) +
    (PPRE2 << registers::cfgr::bits::ppre2::POSITION) +
    (ADCPRE << registers::cfgr::bits::adcpre::POSITION);
  }

#else // VALUE_LINE
  /**
   * @brief Configures the various bus prescalers.
   * @note: Overrides the old configuration.
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

    _RCC->CFGR &=
    registers::cfgr::bits::sw::MASK +
    registers::cfgr::bits::pllsrc::MASK +
    registers::cfgr::bits::pllxtpre::MASK +
    registers::cfgr::bits::pllmul::MASK +
    registers::cfgr::bits::mco::MASK;
    _RCC->CFGR |=
    (HPRE << registers::cfgr::bits::hpre::POSITION) +
    (PPRE1 << registers::cfgr::bits::ppre1::POSITION) +
    (PPRE2 << registers::cfgr::bits::ppre2::POSITION) +
    (ADCPRE << registers::cfgr::bits::adcpre::POSITION) +
    (USBPRE << registers::cfgr::bits::usbpre::POSITION);
  }

#endif // VALUE_LINE
#ifdef VALUE_LINE

  /**
   * @brief Configures the various PLL prescalers and multipliers.
   * @note  The PLL circuitry must be off during configuration
   * @note: Overrides the old configuration.
   */
  template<
  rcc::registers::cfgr::bits::pllsrc::states::E PLLSRC,
  u8 PLLMUL,
  u8 PREDIV1
  >
  static INLINE void configurePll()
  {
    static_assert(PLLMUL < 16,
        "PLLMUL must be between 0 and 15. (inclusive)");
    static_assert(PREDIV1 < 16,
        "PREDIV1 must be between 0 and 15. (inclusive)");

    _RCC->CFGR &=
    ~(registers::cfgr::bits::pllsrc::MASK +
        registers::cfgr::bits::pllmul::MASK +
        registers::cfgr::bits::pllxtpre::MASK);

    _RCC->CFGR |=
    PLLSRC +
    (PLLMUL << registers::cfgr::bits::pllmul::POSITION);

    _RCC->CFGR2 =
    (PREDIV1 << registers::cfgr2::bits::prediv1::POSITION);
  }

#elif defined CONNECTIVITY_LINE

  /**
   * @brief Configures the various PLL prescalers and multipliers.
   * @note  The PLL circuitry must be off during configuration!
   * @note  Overrides the old configuration.
   */
  template<
  rcc::registers::cfgr::bits::pllsrc::states::E PLLSRC,
  u8 PLLMUL,
  u8 PREDIV1,
  u8 PREDIV2,
  u8 PLL2MUL,
  u8 PLL3MUL,
  rcc::registers::cfgr2::bits::prediv1src::states::E PREDIV1SRC,
  rcc::registers::cfgr2::bits::i2s2src::states::E I2S2SRC,
  rcc::registers::cfgr2::bits::i2s3src::states::E I2S3SRC
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

    _RCC->CFGR &=
    ~(registers::cfgr::bits::pllsrc::MASK +
        registers::cfgr::bits::pllmul::MASK +
        registers::cfgr::bits::pllxtpre::MASK);
    _RCC->CFGR |=
    PLLSRC +
    (PLLMUL << registers::cfgr::bits::pllmul::POSITION);
    _RCC->CFGR2 =
    (PREDIV1 << registers::cfgr2::bits::prediv1::POSITION) +
    (PREDIV2 << registers::cfgr2::bits::prediv2::POSITION) +
    (PLL2MUL << registers::cfgr2::bits::pll2mul::POSITION) +
    (PLL3MUL << registers::cfgr2::bits::pll3mul::POSITION) +
    PREDIV1SRC + I2S2SRC + I2S3SRC;
  }

#else

  /**
   * @brief Configures the various PLL prescalers and multipliers.
   * @note  The PLL circuitry must be off during the configuration
   * @note  Overrides the old configuration.
   */
  template <
  rcc::registers::cfgr::bits::pllsrc::states::E PLLSRC,
  u8 PLLXTPRE,
  u8 PLLMUL
  >
  void Functions::configurePll()
  {
    static_assert(PLLXTPRE < 2,
        "PLLXTPRE can only take these values: 0 or 1");
    static_assert(PLLMUL < 16,
        "PLLMUL must be between 0 and 15. (inclusive)");

    _RCC->CFGR &=
    ~(registers::cfgr::bits::pllsrc::MASK +
        registers::cfgr::bits::pllxtpre::MASK +
        registers::cfgr::bits::pllmul::MASK);
    _RCC->CFGR |=
    PLLSRC +
    (PLLXTPRE << registers::cfgr::bits::pllxtpre::POSITION) +
    (PLLMUL << registers::cfgr::bits::pllmul::POSITION);
  }

#endif // VALUE_LINE
#else // STM32F1XX
  /**
   * @brief Enables these peripherals. (AHB1)
   */
  template<
      rcc::registers::ahb1enr::bits::E ... AHB1ENR
  >
  void Functions::enableClocks()
  {
    _RCC->AHB1ENR |= cSum<AHB1ENR...>::value;
  }

  /**
   * @brief Disables these peripherals. (AHB1)
   */
  template<
      rcc::registers::ahb1enr::bits::E ... AHB1ENR
  >
  void Functions::disableClocks()
  {
    _RCC->AHB1ENR &= ~cSum<AHB1ENR...>::value;
  }

  /**
   * @brief Resets these peripherals. (AHB1)
   */
  template<
      rcc::registers::ahb1rstr::bits::E ... AHB1RSTR
  >
  void Functions::resetPeripherals()
  {
    _RCC->AHB1RSTR = cSum<AHB1RSTR...>::value;
  }

  /**
   * @brief Enables these peripherals. (AHB2)
   */
  template<
      rcc::registers::ahb2enr::bits::E ... AHB2ENR
  >
  void Functions::enableClocks()
  {
    _RCC->AHB2ENR |= cSum<AHB2ENR...>::value;
  }

  /**
   * @brief Disables these peripherals. (AHB2)
   */
  template<
      rcc::registers::ahb2enr::bits::E ... AHB2ENR
  >
  void Functions::disableClocks()
  {
    _RCC->AHB2ENR &= ~cSum<AHB2ENR...>::value;
  }

  /**
   * @brief Resets these peripherals. (AHB2)
   */
  template<
      rcc::registers::ahb2rstr::bits::E ... AHB2RSTR
  >
  void Functions::resetPeripherals()
  {
    _RCC->AHB2RSTR = cSum<AHB2RSTR...>::value;
  }

  /**
   * @brief Configures the various PLL prescalers and multipliers.
   * @note  The PLL circuitry must be off during configuration!
   * @note  Overrides the old configuration.
   */
  template<
      rcc::registers::pllcfgr::bits::pllsrc::states::E PLLSRC,
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

    _RCC->PLLCFGR =
        (PLLM << registers::pllcfgr::bits::pllm::POSITION) +
            (PLLN << registers::pllcfgr::bits::plln::POSITION) +
            (((PLLP / 2) - 1) << registers::pllcfgr::bits::pllp::POSITION) +
            (PLLQ << registers::pllcfgr::bits::pllq::POSITION) +
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
    _RCC->PLLCFGR |= registers::cfgr::bits::i2ssrc::states::PLLI2S;

    _RCC->PLLI2SCFGR =
        (PLLI2SN << registers::plli2scfgr::bits::plli2sn::POSITION) +
            (PLLI2SR << registers::plli2scfgr::bits::plli2sr::POSITION);
  }

  /**
   * @brief Configures the various bus prescalers.
   * @note: Overrides the old configuration.
   */
  template<
      u8 HPRE,
      u8 PPRE1,
      u8 PPRE2
  >
  void Functions::configurePrescalers()
  {
    static_assert(HPRE < 16,
        "The AHB prescaler (HPRE) must be lower than 16. (inclusive)");

    static_assert(PPRE1 < 8,
        "The APB1 prescaler (PPRE1) must be lower than 8. (inclusive)");

    static_assert(PPRE2 < 8,
        "The APB2 prescaler (PPRE2) must be lower than 8. (inclusive)");

    _RCC->CFGR &=
        ~(registers::cfgr::bits::hpre::MASK +
            registers::cfgr::bits::ppre1::MASK +
            registers::cfgr::bits::ppre2::MASK)
        ;
    _RCC->CFGR |=
        (HPRE << registers::cfgr::bits::hpre::POSITION) +
            (PPRE1 << registers::cfgr::bits::ppre1::POSITION) +
            (PPRE2 << registers::cfgr::bits::ppre2::POSITION)
            ;
  }

  /**
   * @brief Configures the HSE prescaler for the RTC.
   * @note  Overrides the old configuration.
   */
  template<
      u8 RTCPRE
  >
  void Functions::configureRtcPrescaler()
  {
    static_assert(RTCPRE < 32,
        "The RTC prescaler (RTCPRE) must be lower than 32. (inclusive)");
    _RCC->CFGR &= ~registers::cfgr::bits::rtcpre::MASK;
    _RCC->CFGR |= (RTCPRE << registers::cfgr::bits::rtcpre::POSITION);
  }

#endif // STM32F1XX
}  // namespace rcc

