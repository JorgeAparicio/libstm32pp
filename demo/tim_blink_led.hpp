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

#include "interrupt.hpp"
#include "peripheral/gpio.hpp"
#include "peripheral/rcc.hpp"
#include "peripheral/tim.hpp"
#include "core/nvic.hpp"

// TODO Test TIM demo on STM32F1XX

int main()
{
#ifdef STM32F1XX
  RCC::enableClocks<
      rcc::apb2enr::IOPA
  >();

  PA0::setMode<
      gpio::registers::cr::states::GP_PUSH_PULL_2MHZ
  >();

#else
  RCC::enableClocks<
  rcc::ahb1enr::GPIOA
  >();

  PA0::setMode<
  gpio::registers::moder::states::OUTPUT
  >();
#endif

  RCC::enableClocks<
      rcc::apb1enr::TIM6
  >();

  TIM6::configureBasicCounter<
      tim::registers::cr1::bits::cen::states::COUNTER_DISABLED,
      tim::registers::cr1::bits::udis::states::UPDATE_EVENT_ENABLED,
      tim::registers::cr1::bits::urs::states::UPDATE_REQUEST_SOURCE_OVERFLOW_UNDERFLOW,
      tim::registers::cr1::bits::opm::states::DONT_STOP_COUNTER_AT_NEXT_UPDATE_EVENT,
      tim::registers::cr1::bits::arpe::states::AUTO_RELOAD_UNBUFFERED
  >();

  TIM6::enableUpdateInterrupt();

  TIM6::setPrescaler(3999);

  TIM6::setAutoReload(1999);

  TIM6::generateUpdate();

#if defined VALUE_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX
  NVIC::enableInterrupt<
  nvic::irqn::TIM6_DAC
  >();
#else
  NVIC::enableInterrupt<
      nvic::irqn::TIM6
  >();
#endif

  TIM6::startCounter();

  while (true) {
  }

}

#if defined VALUE_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX
void interrupt::TIM6_DAC()
#else
void interrupt::TIM6()
#endif
{
  static u32 debug = 0;

  TIM6::clearUpdateFlag();

  debug++;

  if (debug % 2)
    PA0::setLow();
  else
    PA0::setHigh();
}
