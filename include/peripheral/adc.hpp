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

/*******************************************************************************
 *
 *                        Analog to Digital Converter
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"
#include "../defs.hpp"

#include "../../memorymap/adc.hpp"

// Low-level access to the registers
#define _ADC1 reinterpret_cast<adc::Registers *>(adc::address::E::ADC1)
#define _ADC2 reinterpret_cast<adc::Registers *>(adc::address::E::ADC2)
#ifndef STM32F1XX
#define _ADC3 reinterpret_cast<adc::Registers *>(adc::address::E::ADC3)
#define _ADC  reinterpret_cast<adc::CommonRegisters *>(adc::address::E::ADC)
#endif

// High-level functions
namespace adc {
  template<address::E>
  class Functions {
    public:
      static inline void enableClock();
      static inline void disableClock();
      static inline void enablePeripheral();
      static inline void disablePeripheral();
      static inline void startRegularConversions();
      static inline void startInjectedConversions();
      static inline void enableContinuousConversion();
      static inline void disableContinuousConversion();
      static inline void setWatchdogLowerThreshold(u16 const);
      static inline u16 getWatchdogLowerThreshold();
      static inline void setWatchdogHigherThreshold(u16 const);
      static inline u16 getWatchdogHigherThreshold();
      static inline bool hasRegularConversionEnded();
      static inline bool hasInjectedConversionEnded();
      static inline u16 getConversionResult();

      template<u32>
      static inline void setNumberOfRegularChannels();

      template<u32>
      static inline void setNumberOfInjectedChannels();

      template<
          u32 CHANNEL,
          adc::registers::smp::states::E
      >
      static inline void setConversionTime();

      template<
          u32 ORDER,
          u32 CONVERSION
      >
      static inline void setRegularSequenceOrder();

      template<
          u32 ORDER,
          u32 CONVERSION
      >
      static inline void setInjectedSequenceOrder();

      template<
          adc::registers::cr1::bits::awdch::states::E,
          adc::registers::cr1::bits::eocie::states::E,
          adc::registers::cr1::bits::awdie::states::E,
          adc::registers::cr1::bits::jeocie::states::E,
          adc::registers::cr1::bits::scan::states::E,
          adc::registers::cr1::bits::awdsgl::states::E,
          adc::registers::cr1::bits::jauto::states::E,
          adc::registers::cr1::bits::discen::states::E,
          adc::registers::cr1::bits::jdiscen::states::E,
          adc::registers::cr1::bits::discnum::states::E,
          adc::registers::cr1::bits::jawden::states::E,
          adc::registers::cr1::bits::awden::states::E,
          adc::registers::cr1::bits::res::states::E,
          adc::registers::cr1::bits::ovrie::states::E,
          adc::registers::cr2::bits::adon::states::E,
          adc::registers::cr2::bits::cont::states::E,
          adc::registers::cr2::bits::dma::states::E,
          adc::registers::cr2::bits::dds::states::E,
          adc::registers::cr2::bits::eocs::states::E,
          adc::registers::cr2::bits::align::states::E,
          adc::registers::cr2::bits::jextsel::states::E,
          adc::registers::cr2::bits::jexten::states::E,
          adc::registers::cr2::bits::jswstart::states::E,
          adc::registers::cr2::bits::extsel::states::E,
          adc::registers::cr2::bits::exten::states::E,
          adc::registers::cr2::bits::swstart::states::E
      >
      static inline void configure();

    private:
      Functions();
  };
}  // namespace adc

// High-level access to the peripherals
typedef adc::Functions<adc::address::ADC1> ADC1;
typedef adc::Functions<adc::address::ADC2> ADC2;
#ifndef STM32F1XX
typedef adc::Functions<adc::address::ADC3> ADC3;
#endif

#include "../../bits/adc.tcc"
