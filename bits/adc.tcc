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
  template<address::E A>
  void Functions<A>::enableClock()
  {
    RCC::enableClocks<
        A == address::ADC1 ?
            rcc::registers::apb2enr::bits::ADC1 :
            (A == address::ADC2 ?
                rcc::registers::apb2enr::bits::ADC2 :
                (A == address::ADC3 ?
                                      rcc::registers::apb2enr::bits::ADC3 :
                                      0))
    >();
  }

  /**
   * @brief Disables the ADC's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  template<address::E A>
  void Functions<A>::disableClock()
  {
    RCC::disableClocks<
        A == address::ADC1 ?
            rcc::registers::apb2enr::bits::ADC1 :
            (A == address::ADC2 ?
                rcc::registers::apb2enr::bits::ADC2 :
                (A == address::ADC3 ?
                                      rcc::registers::apb2enr::bits::ADC3 :
                                      0))
    >();
  }

  /**
   * @brief Turns on the ADC peripheral.
   */
  template<address::E A>
  void Functions<A>::enablePeripheral()
  {
    *(u32*) (bitband::Peripheral<
        A + registers::cr1::OFFSET,
        registers::cr2::bits::adon::POSITION
    >::address) = 1;
  }

  /**
   * @brief Turns off the ADC peripheral.
   */
  template<address::E A>
  void Functions<A>::disablePeripheral()
  {
    *(u32*) (bitband::Peripheral<
        A + registers::cr1::OFFSET,
        registers::cr2::bits::adon::POSITION
    >::address) = 0;
  }

  /**
   * @brief Starts conversion of regular channels
   */
  template<address::E A>
  void Functions<A>::startRegularConversions()
  {
    *(u32*) (bitband::Peripheral<
        A + registers::cr2::OFFSET,
        registers::cr2::bits::swstart::POSITION
    >::address) = 1;
  }

  /**
   * @brief Starts conversion of injected channels
   */
  template<address::E A>
  void Functions<A>::startInjectedConversions()
  {
    *(u32*) (bitband::Peripheral<
        A + registers::cr2::OFFSET,
        registers::cr2::bits::jswstart::POSITION
    >::address) = 1;
  }

  /**
   * @brief Enables continuous conversion mode.
   */
  template<address::E A>
  void Functions<A>::enableContinuousConversion()
  {
    *(u32*) (bitband::Peripheral<
        A + registers::cr2::OFFSET,
        registers::cr2::bits::cont::POSITION
    >::address) = 1;
  }

  /**
   * @brief Disables continuous conversion mode.
   */
  template<address::E A>
  void Functions<A>::disableContinuousConversion()
  {
    *(u32*) (bitband::Peripheral<
        A + registers::cr2::OFFSET,
        registers::cr2::bits::cont::POSITION
    >::address) = 0;
  }

  /**
   * @brief Sets the lower threshold of the analog watchdog.
   */
  template<address::E A>
  void Functions<A>::setWatchdogLowerThreshold(u16 const Threshold)
  {
    reinterpret_cast<Registers*>(A)->LTR = Threshold;
  }

  /**
   * @brief Returns the lower threshold of the analog watchdog.
   */
  template<address::E A>
  u16 Functions<A>::getWatchdogLowerThreshold()
  {
    return reinterpret_cast<Registers*>(A)->LTR;
  }

  /**
   * @brief Sets the higher threshold of the analog watchdog.
   */
  template<address::E A>
  void Functions<A>::setWatchdogHigherThreshold(u16 const Threshold)
  {
    reinterpret_cast<Registers*>(A)->HTR = Threshold;
  }

  /**
   * @brief Returns the higher threshold of the analog watchdog.
   */
  template<address::E A>
  u16 Functions<A>::getWatchdogHigherThreshold()
  {
    return reinterpret_cast<Registers*>(A)->HTR;
  }

  /**
   * @brief Configures the conversion time.
   */
  template<address::E A>
  template<
      u32 C,
      adc::registers::smp::states::E SMP
  >
  void Functions<A>::setConversionTime()
  {
    static_assert(C <= 18, "There are only channels from 0 to 18.");

    reinterpret_cast<Registers*>(A)->SMPR[(C > 9) ? 0 : 1] &=
        ~(registers::smp::MASK << registers::smp::POSITION *
            ((C > 9) ?
                       (C - 10) :
                       C));

    reinterpret_cast<Registers*>(A)->SMPR[(C > 9) ? 0 : 1] |=
        (SMP << registers::smp::POSITION *
            ((C > 9) ?
                       (C - 10) :
                       C));
  }

  /**
   * @brief Returns true if the regular conversions have ended.
   */
  template<address::E A>
  bool Functions<A>::hasRegularConversionEnded()
  {
    return *(bool*) (bitband::Peripheral<
        A + registers::sr::OFFSET,
        registers::sr::bits::eoc::POSITION
    >::address);
  }

  /**
   * @brief Returns true if the injected conversions have ended.
   */
  template<address::E A>
  bool Functions<A>::hasInjectedConversionEnded()
  {
    return *(bool*) (bitband::Peripheral<
        A + registers::sr::OFFSET,
        registers::sr::bits::jeoc::POSITION
    >::address);
  }

  /**
   * @brief Returns the result of the conversion.
   */
  template<address::E A>
  u16 Functions<A>::getConversionResult()
  {
    return reinterpret_cast<Registers*>(A)->DR;
  }

  /**
   * @brief Sets the number of regular conversions.
   */
  template<address::E A>
  template<u32 N>
  void Functions<A>::setNumberOfRegularChannels()
  {
    static_assert(N > 0, "There must be at least one conversion.");
    static_assert(N <= 16, "The maximum number of regular conversions is 16.");

    reinterpret_cast<Registers*>(A)->SQR[0] &=
        ~registers::sqr1::bits::l::MASK;

    reinterpret_cast<Registers*>(A)->SQR[0] |=
        (N - 1) << registers::sqr1::bits::l::POSITION;
  }

  /**
   * @brief Sets the number of injected conversions.
   */
  template<address::E A>
  template<u32 N>
  void Functions<A>::setNumberOfInjectedChannels()
  {
    static_assert(N > 0, "There must be at least one conversion.");
    static_assert(N <= 4, "The maximum number of injected conversions is 4.");

    reinterpret_cast<Registers*>(A)->JSQR &=
        ~registers::jsqr::bits::jl::MASK;

    reinterpret_cast<Registers*>(A)->JSQR |=
        N << registers::jsqr::bits::jl::POSITION;
  }

  /**
   * @brief Configures the order of the regular conversions.
   */
  template<address::E A>
  template<u32 O, u32 C>
  void Functions<A>::setRegularSequenceOrder()
  {
    static_assert((O >= 1) && (O <= 16), "Order range goes from 1 to 16");
    static_assert((C >= 0) && (C <= 18), "Conversion range goes from 0 to 18");

    reinterpret_cast<Registers*>(A)->SQR[(O > 12) ? 0 : ((O > 6) ? 1 : 2)] &=
        ~(registers::sqr::MASK << registers::sqr::POSITION *
            ((O > 12) ?
                        (O - 13) :
                        ((O > 6) ?
                                   (O - 7) :
                                   (O - 1))));

    reinterpret_cast<Registers*>(A)->SQR[(O > 12) ? 0 : ((O > 6) ? 1 : 2)] |=
        (C << registers::sqr::POSITION *
            ((O > 12) ?
                        (O - 13) :
                        ((O > 6) ?
                                   (O - 7) :
                                   (O - 1))));
  }

  /**
   * @brief Configures the order of the injected conversions.
   */
  template<address::E A>
  template<u32 O, u32 C>
  void Functions<A>::setInjectedSequenceOrder()
  {
    static_assert((O >= 1) && (O <= 4), "Order range goes from 1 to 4");
    static_assert((C >= 0) && (C <= 18), "Conversion range goes from 0 to 18");

    reinterpret_cast<Registers*>(A)->JSQR &=
        ~registers::jsq::MASK << registers::jsq::POSITION * (O - 1);

    reinterpret_cast<Registers*>(A)->JSQR |=
        C << registers::jsq::POSITION * (O - 1);
  }

  /**
   * @brief Configures the ADC.
   * @note Overrides the old configuration.
   */
  template<address::E A>
  template<
      registers::cr1::bits::awdch::states::E AWDCH,
      registers::cr1::bits::eocie::states::E EOCIE,
      registers::cr1::bits::awdie::states::E AWDIE,
      registers::cr1::bits::jeocie::states::E JEOCIE,
      registers::cr1::bits::scan::states::E SCAN,
      registers::cr1::bits::awdsgl::states::E AWDSGL,
      registers::cr1::bits::jauto::states::E JAUTO,
      registers::cr1::bits::discen::states::E DISCEN,
      registers::cr1::bits::jdiscen::states::E JDISCEN,
      registers::cr1::bits::discnum::states::E DISCNUM,
      registers::cr1::bits::jawden::states::E JAWDEN,
      registers::cr1::bits::awden::states::E AWDEN,
      registers::cr1::bits::res::states::E RES,
      registers::cr1::bits::ovrie::states::E OVRIE,
      registers::cr2::bits::adon::states::E ADON,
      registers::cr2::bits::cont::states::E CONT,
      registers::cr2::bits::dma::states::E DMA,
      registers::cr2::bits::dds::states::E DDS,
      registers::cr2::bits::eocs::states::E EOCS,
      registers::cr2::bits::align::states::E ALIGN,
      registers::cr2::bits::jextsel::states::E JEXTSEL,
      registers::cr2::bits::jexten::states::E JEXTEN,
      registers::cr2::bits::jswstart::states::E JSWSTART,
      registers::cr2::bits::extsel::states::E EXTSEL,
      registers::cr2::bits::exten::states::E EXTEN,
      registers::cr2::bits::swstart::states::E SWSTART
  >
  void Functions<A>::configure()
  {
    reinterpret_cast<Registers*>(A)->CR1 =
        AWDCH + EOCIE + AWDIE + JEOCIE + SCAN + AWDSGL + JAUTO + DISCEN +
            JDISCEN + DISCNUM + JAWDEN + AWDEN + RES + OVRIE;

    reinterpret_cast<Registers*>(A)->CR2 =
        ADON + CONT + DMA + DDS + EOCS + ALIGN + JEXTSEL + JEXTEN + JSWSTART +
            EXTSEL + EXTEN + SWSTART;
  }
}  // namespace adc
