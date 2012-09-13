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

// DO NOT INCLUDE THIS FILE ANYWHERE. THIS DEMO IS JUST A REFERENCE TO BE USED
// IN YOUR MAIN SOURCE FILE.

#include "device_select.hpp"
#include "peripheral/gpio.hpp"
#include "peripheral/rcc.hpp"
#include "peripheral/adc.hpp"

// TODO Test ADC demo

int main()
{
  volatile u32 debug = 0;

  GPIOA::enableClock();
#ifdef STM32F1XX
  PA0::setMode(gpio::cr::ANALOG_INPUT);
#else
  PA0::setMode(gpio::moder::ANALOG);
#endif

  PA0::setPullMode(gpio::pupdr::PULL_UP);

  ADC1::enableClock();

  ADC1::configure(
      adc::cr1::awdch::SET_ANALOG_WATCHDOG_ON_CHANNEL18,
      adc::cr1::eocie::END_OF_CONVERSION_INTERRUPT_DISABLED,
      adc::cr1::awdie::ANALOG_WATCHDOG_INTERRUPT_DISABLED,
      adc::cr1::jeocie::END_OF_ALL_INJECTED_CONVERSIONS_INTERRUPT_DISABLED,
      adc::cr1::scan::SCAN_MODE_DISABLED,
      adc::cr1::awdsgl::ANALOG_WATCHDOG_ENABLED_ON_A_SINGLE_CHANNEL,
      adc::cr1::jauto::AUTOMATIC_INJECTED_CONVERSION_DISABLED,
      adc::cr1::discen::DISCONTINUOUS_MODE_ON_REGULAR_CHANNELS_DISABLED,
      adc::cr1::jdiscen::DISCONTINUOUS_MODE_ON_INJECTED_CHANNELS_DISABLED,
      adc::cr1::discnum::_1_CHANNEL_FOR_DISCONTINUOUS_MODE,
      adc::cr1::jawden::ANALOG_WATCHDOG_DISABLED_ON_INJECTED_CHANNELS,
      adc::cr1::awden::ANALOG_WATCHDOG_DISABLED_ON_REGULAR_CHANNELS,
      adc::cr1::res::_12_BITS_RESOLUTION,
      adc::cr1::ovrie::OVERRUN_INTERRUPT_DISABLED,
      adc::cr2::adon::ADC_ENABLED,
      adc::cr2::cont::SINGLE_CONVERSION_MODE,
      adc::cr2::dma::DMA_MODE_DISABLED,
      adc::cr2::dds::NO_NEW_DMA_REQUEST_IS_ISSUED_AFTER_THE_LAST_TRANSFER,
      adc::cr2::eocs::EOC_BIT_IS_SET_AFTER_EACH_REGULAR_CONVERSION,
      adc::cr2::align::LEFT_ALIGNED_DATA,
      adc::cr2::jextsel::INJECTED_GROUP_TRIGGERED_BY_EXTI15,
      adc::cr2::jexten::INJECTED_TRIGGER_DISABLED,
      adc::cr2::jswstart::INJECTED_CHANNELS_ON_RESET_STATE,
      adc::cr2::extsel::REGULAR_GROUP_TRIGGERED_BY_EXTI11,
      adc::cr2::exten::REGULAR_TRIGGER_DISABLED,
      adc::cr2::swstart::REGULAR_CHANNELS_ON_RESET_STATE);

  ADC1::setRegularSequenceOrder<1, 0>();

  ADC1::setNumberOfRegularChannels<1>();

  ADC1::setConversionTime<
      0,
      adc::smp::SAMPLING_TIME_480_CYCLES
  >();

  while (true) {
    ADC1::startRegularConversions();

    while (!ADC1::hasRegularConversionEnded()) {
    }

    debug = ADC1::getConversionResult();
  }
}

