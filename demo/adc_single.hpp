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

// DO NOT INCLUDE THIS FILE ANYWHERE. THIS DEMO IS JUST A REFERENCE TO BE USED
// IN YOUR MAIN SOURCE FILE.

#include "device_select.hpp"
#include "peripheral/gpio.hpp"
#include "peripheral/rcc.hpp"
#include "peripheral/adc.hpp"

int main()
{
  u32 debug = 0;

#ifdef STM32F1XX
  RCC::enableClocks<
      rcc::registers::apb2enr::bits::IOPA
  >();

  PA0::setMode<
      gpio::registers::cr::states::ANALOG_INPUT
  >();

#else
  RCC::enableClocks<
  rcc::registers::ahb1enr::bits::GPIOA
  >();

  PA0::setMode<
  gpio::registers::moder::states::ANALOG
  >();
#endif

  RCC::enableClocks<
      rcc::registers::apb2enr::bits::ADC1
  >();

  ADC1::configure<
      adc::registers::cr1::bits::awdch::states::SET_ANALOG_WATCHDOG_ON_CHANNEL18,
      adc::registers::cr1::bits::eocie::states::END_OF_CONVERSION_INTERRUPT_DISABLED,
      adc::registers::cr1::bits::awdie::states::ANALOG_WATCHDOG_INTERRUPT_DISABLED,
      adc::registers::cr1::bits::jeocie::states::END_OF_ALL_INJECTED_CONVERSIONS_INTERRUPT_DISABLED,
      adc::registers::cr1::bits::scan::states::SCAN_MODE_DISABLED,
      adc::registers::cr1::bits::awdsgl::states::ANALOG_WATCHDOG_ENABLED_ON_A_SINGLE_CHANNEL,
      adc::registers::cr1::bits::jauto::states::AUTOMATIC_INJECTED_CONVERSION_DISABLED,
      adc::registers::cr1::bits::discen::states::DISCONTINUOUS_MODE_ON_REGULAR_CHANNELS_DISABLED,
      adc::registers::cr1::bits::jdiscen::states::DISCONTINUOUS_MODE_ON_INJECTED_CHANNELS_DISABLED,
      adc::registers::cr1::bits::discnum::states::_1_CHANNEL_FOR_DISCONTINUOUS_MODE,
      adc::registers::cr1::bits::jawden::states::ANALOG_WATCHDOG_DISABLED_ON_INJECTED_CHANNELS,
      adc::registers::cr1::bits::awden::states::ANALOG_WATCHDOG_DISABLED_ON_REGULAR_CHANNELS,
      adc::registers::cr1::bits::res::states::_12_BITS_RESOLUTION,
      adc::registers::cr1::bits::ovrie::states::OVERRUN_INTERRUPT_DISABLED,
      adc::registers::cr2::bits::adon::states::ADC_ENABLED,
      adc::registers::cr2::bits::cont::states::SINGLE_CONVERSION_MODE,
      adc::registers::cr2::bits::dma::states::DMA_MODE_DISABLED,
      adc::registers::cr2::bits::dds::states::NO_NEW_DMA_REQUEST_IS_ISSUED_AFTER_THE_LAST_TRANSFER,
      adc::registers::cr2::bits::eocs::states::EOC_BIT_IS_SET_AFTER_EACH_REGULAR_CONVERSION,
      adc::registers::cr2::bits::align::states::LEFT_ALIGNED_DATA,
      adc::registers::cr2::bits::jextsel::states::INJECTED_GROUP_TRIGGERED_BY_EXTI15,
      adc::registers::cr2::bits::jexten::states::INJECTED_TRIGGER_DISABLED,
      adc::registers::cr2::bits::jswstart::states::INJECTED_CHANNELS_ON_RESET_STATE,
      adc::registers::cr2::bits::extsel::states::REGULAR_GROUP_TRIGGERED_BY_EXTI11,
      adc::registers::cr2::bits::exten::states::REGULAR_TRIGGER_DISABLED,
      adc::registers::cr2::bits::swstart::states::REGULAR_CHANNELS_ON_RESET_STATE
  >();

  ADC1::setRegularSequenceOrder<1, 0>();

  ADC1::setNumberOfRegularChannels<1>();

  ADC1::setConversionTime<
      0,
      adc::registers::smp::states::SAMPLING_TIME_480_CYCLES
  >();

  while (true) {
    ADC1::startRegularConversions();

    while (!ADC1::hasRegularConversionEnded()) {
    }

    debug = ADC1::getConversionResult();
  }
}
