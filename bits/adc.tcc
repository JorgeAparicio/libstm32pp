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
#include "../include/peripheral/rcc.hpp"

namespace adc {
  /**
   * @brief Enables the ADC's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  template<Address A>
  void Functions<A>::enableClock()
  {
    switch (A) {
      case ADC1:
        RCC::enableClocks<rcc::apb2enr::ADC1>();
        break;
      case ADC2:
        RCC::enableClocks<rcc::apb2enr::ADC2>();
        break;
      case ADC3:
        RCC::enableClocks<rcc::apb2enr::ADC3>();
        break;
    }
  }

  /**
   * @brief Disables the ADC's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  template<Address A>
  void Functions<A>::disableClock()
  {
    switch (A) {
      case ADC1:
        RCC::disableClocks<rcc::apb2enr::ADC1>();
        break;
      case ADC2:
        RCC::disableClocks<rcc::apb2enr::ADC2>();
        break;
      case ADC3:
        RCC::disableClocks<rcc::apb2enr::ADC3>();
        break;
    }
  }

  /**
   * @brief Turns on the ADC peripheral.
   */
  template<Address A>
  void Functions<A>::enablePeripheral()
  {
    *(u32 volatile*) (bitband::peripheral<
        A + cr2::OFFSET,
        cr2::adon::POSITION
    >()) = 1;
  }

  /**
   * @brief Turns off the ADC peripheral.
   */
  template<Address A>
  void Functions<A>::disablePeripheral()
  {
    *(u32 volatile*) (bitband::peripheral<
        A + cr2::OFFSET,
        cr2::adon::POSITION
    >()) = 0;
  }

  /**
   * @brief Starts conversion of regular channels
   */
  template<Address A>
  void Functions<A>::startRegularConversions()
  {
    *(u32 volatile*) (bitband::peripheral<
        A + cr2::OFFSET,
        cr2::swstart::POSITION
    >()) = 1;
  }

  /**
   * @brief Starts conversion of injected channels
   */
  template<Address A>
  void Functions<A>::startInjectedConversions()
  {
    *(u32 volatile*) (bitband::peripheral<
        A + cr2::OFFSET,
        cr2::jswstart::POSITION
    >()) = 1;
  }

  /**
   * @brief Enables continuous conversion mode.
   */
  template<Address A>
  void Functions<A>::enableContinuousConversion()
  {
    *(u32 volatile*) (bitband::peripheral<
        A + cr2::OFFSET,
        cr2::cont::POSITION
    >()) = 1;
  }

  /**
   * @brief Disables continuous conversion mode.
   */
  template<Address A>
  void Functions<A>::disableContinuousConversion()
  {
    *(u32 volatile*) (bitband::peripheral<
        A + cr2::OFFSET,
        cr2::cont::POSITION
    >()) = 0;
  }

  /**
   * @brief Sets the lower threshold of the analog watchdog.
   */
  template<Address A>
  void Functions<A>::setWatchdogLowerThreshold(u16 const Threshold)
  {
    reinterpret_cast<Registers*>(A)->LTR = Threshold;
  }

  /**
   * @brief Returns the lower threshold of the analog watchdog.
   */
  template<Address A>
  u16 Functions<A>::getWatchdogLowerThreshold()
  {
    return reinterpret_cast<Registers*>(A)->LTR;
  }

  /**
   * @brief Sets the higher threshold of the analog watchdog.
   */
  template<Address A>
  void Functions<A>::setWatchdogHigherThreshold(u16 const Threshold)
  {
    reinterpret_cast<Registers*>(A)->HTR = Threshold;
  }

  /**
   * @brief Returns the higher threshold of the analog watchdog.
   */
  template<Address A>
  u16 Functions<A>::getWatchdogHigherThreshold()
  {
    return reinterpret_cast<Registers*>(A)->HTR;
  }

  /**
   * @brief Configures the conversion time.
   */
  template<Address A>
  template<
      u32 C,
      adc::smp::States SMP
  >
  void Functions<A>::setConversionTime()
  {
    static_assert(C <= 18, "There are only channels from 0 to 18.");

    reinterpret_cast<Registers*>(A)->SMPR[(C > 9) ? 0 : 1] &=
        ~(smp::MASK << smp::POSITION *
            ((C > 9) ?
                       (C - 10) :
                       C));

    reinterpret_cast<Registers*>(A)->SMPR[(C > 9) ? 0 : 1] |=
        (SMP << smp::POSITION *
            ((C > 9) ?
                       (C - 10) :
                       C));
  }

  /**
   * @brief Returns true if the regular conversions have ended.
   */
  template<Address A>
  bool Functions<A>::hasRegularConversionEnded()
  {
    return *(bool volatile*) (bitband::peripheral<
        A + sr::OFFSET,
        sr::eoc::POSITION
    >());
  }

  /**
   * @brief Returns true if the injected conversions have ended.
   */
  template<Address A>
  bool Functions<A>::hasInjectedConversionEnded()
  {
    return *(bool volatile*) (bitband::peripheral<
        A + sr::OFFSET,
        sr::jeoc::POSITION
    >());
  }

  /**
   * @brief Returns the result of the conversion.
   */
  template<Address A>
  u16 Functions<A>::getConversionResult()
  {
    return reinterpret_cast<Registers*>(A)->DR;
  }

  /**
   * @brief Sets the number of regular conversions.
   */
  template<Address A>
  template<u32 N>
  void Functions<A>::setNumberOfRegularChannels()
  {
    static_assert(N > 0, "There must be at least one conversion.");
    static_assert(N <= 16, "The maximum number of regular conversions is 16.");

    reinterpret_cast<Registers*>(A)->SQR[0] &=
        ~sqr1::l::MASK;

    reinterpret_cast<Registers*>(A)->SQR[0] |=
        (N - 1) << sqr1::l::POSITION;
  }

  /**
   * @brief Sets the number of injected conversions.
   */
  template<Address A>
  template<u32 N>
  void Functions<A>::setNumberOfInjectedChannels()
  {
    static_assert(N > 0, "There must be at least one conversion.");
    static_assert(N <= 4, "The maximum number of injected conversions is 4.");

    reinterpret_cast<Registers*>(A)->JSQR &=
        ~jsqr::jl::MASK;

    reinterpret_cast<Registers*>(A)->JSQR |=
        N << jsqr::jl::POSITION;
  }

  /**
   * @brief Configures the order of the regular conversions.
   */
  template<Address A>
  template<u32 O, u32 C>
  void Functions<A>::setRegularSequenceOrder()
  {
    static_assert((O >= 1) && (O <= 16), "Order range goes from 1 to 16");
    static_assert((C >= 0) && (C <= 18), "Conversion range goes from 0 to 18");

    reinterpret_cast<Registers*>(A)->SQR[(O > 12) ? 0 : ((O > 6) ? 1 : 2)] &=
        ~(sqr::MASK << sqr::POSITION *
            ((O > 12) ?
                        (O - 13) :
                        ((O > 6) ?
                                   (O - 7) :
                                   (O - 1))));

    reinterpret_cast<Registers*>(A)->SQR[(O > 12) ? 0 : ((O > 6) ? 1 : 2)] |=
        (C << sqr::POSITION *
            ((O > 12) ?
                        (O - 13) :
                        ((O > 6) ?
                                   (O - 7) :
                                   (O - 1))));
  }

  /**
   * @brief Configures the order of the injected conversions.
   */
  template<Address A>
  template<u32 O, u32 C>
  void Functions<A>::setInjectedSequenceOrder()
  {
    static_assert((O >= 1) && (O <= 4), "Order range goes from 1 to 4");
    static_assert((C >= 0) && (C <= 18), "Conversion range goes from 0 to 18");

    reinterpret_cast<Registers*>(A)->JSQR &=
        ~jsq::MASK << jsq::POSITION * (O - 1);

    reinterpret_cast<Registers*>(A)->JSQR |=
        C << jsq::POSITION * (O - 1);
  }

  /**
   * @brief Configures the ADC.
   * @note Overrides the old configuration.
   */
  template<Address A>
  void Functions<A>::configure(
      cr1::awdch::States AWDCH,
      cr1::eocie::States EOCIE,
      cr1::awdie::States AWDIE,
      cr1::jeocie::States JEOCIE,
      cr1::scan::States SCAN,
      cr1::awdsgl::States AWDSGL,
      cr1::jauto::States JAUTO,
      cr1::discen::States DISCEN,
      cr1::jdiscen::States JDISCEN,
      cr1::discnum::States DISCNUM,
      cr1::jawden::States JAWDEN,
      cr1::awden::States AWDEN,
      cr1::res::States RES,
      cr1::ovrie::States OVRIE,
      cr2::adon::States ADON,
      cr2::cont::States CONT,
      cr2::dma::States DMA,
      cr2::dds::States DDS,
      cr2::eocs::States EOCS,
      cr2::align::States ALIGN,
      cr2::jextsel::States JEXTSEL,
      cr2::jexten::States JEXTEN,
      cr2::jswstart::States JSWSTART,
      cr2::extsel::States EXTSEL,
      cr2::exten::States EXTEN,
      cr2::swstart::States SWSTART)
  {
    reinterpret_cast<Registers*>(A)->CR1 =
        AWDCH + EOCIE + AWDIE + JEOCIE + SCAN + AWDSGL + JAUTO + DISCEN +
            JDISCEN + DISCNUM + JAWDEN + AWDEN + RES + OVRIE;

    reinterpret_cast<Registers*>(A)->CR2 =
        ADON + CONT + DMA + DDS + EOCS + ALIGN + JEXTSEL + JEXTEN + JSWSTART +
            EXTSEL + EXTEN + SWSTART;
  }

  void CommonFunctions::enableTemperatureSensor()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADC + ccr::OFFSET,
        ccr::tsvrefe::POSITION
    >()) = 1;
  }

  void CommonFunctions::disableTemperatureSensor()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADC + ccr::OFFSET,
        ccr::tsvrefe::POSITION
    >()) = 0;
  }

  void CommonFunctions::setPrescaler(ccr::adcpre::States ADCPRE)
  {
    ADC_COMMON_REGS->CCR &= ccr::adcpre::MASK;
    ADC_COMMON_REGS->CCR |= ADCPRE;
  }
}  // namespace adc
