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
#define ADC1_REGS reinterpret_cast<adc::Registers *>(adc::Address::ADC1)
#define ADC2_REGS reinterpret_cast<adc::Registers *>(adc::Address::ADC2)
#ifndef STM32F1XX
#define ADC3_REGS reinterpret_cast<adc::Registers *>(adc::Address::ADC3)
#define ADC_COMMON_REGS  reinterpret_cast<adc::CommonRegisters *>(adc::Address::ADC)
#endif

// High-level functions
namespace adc {
  template<Address>
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
          adc::smp::States
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

      static inline void configure(
          adc::cr1::awdch::States,
          adc::cr1::eocie::States,
          adc::cr1::awdie::States,
          adc::cr1::jeocie::States,
          adc::cr1::scan::States,
          adc::cr1::awdsgl::States,
          adc::cr1::jauto::States,
          adc::cr1::discen::States,
          adc::cr1::jdiscen::States,
          adc::cr1::discnum::States,
          adc::cr1::jawden::States,
          adc::cr1::awden::States,
          adc::cr1::res::States,
          adc::cr1::ovrie::States,
          adc::cr2::adon::States,
          adc::cr2::cont::States,
          adc::cr2::dma::States,
          adc::cr2::dds::States,
          adc::cr2::eocs::States,
          adc::cr2::align::States,
          adc::cr2::jextsel::States,
          adc::cr2::jexten::States,
          adc::cr2::jswstart::States,
          adc::cr2::extsel::States,
          adc::cr2::exten::States,
          adc::cr2::swstart::States);

    private:
      Functions();
  };
}  // namespace adc

// High-level access to the peripherals
typedef adc::Functions<adc::ADC1> ADC1;
typedef adc::Functions<adc::ADC2> ADC2;
#ifndef STM32F1XX
typedef adc::Functions<adc::ADC3> ADC3;
#endif

#include "../../bits/adc.tcc"
