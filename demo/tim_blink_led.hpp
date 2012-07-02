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

int main()
{
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

  TIM6::configureCounter<
      tim::registers::cr1::bits::cen::states::COUNTER_DISABLED,
      tim::registers::cr1::bits::udis::states::UPDATE_EVENT_ENABLED,
      tim::registers::cr1::bits::urs::states::UPDATE_REQUEST_SOURCE_OVERFLOW_UNDERFLOW,
      tim::registers::cr1::bits::opm::states::DONT_STOP_COUNTER_AT_NEXT_UPDATE_EVENT,
      tim::registers::cr1::bits::arpe::states::AUTO_RELOAD_UNBUFFERED
  >();

  TIM6::enableInterrupt();

  TIM6::setPrescaler(3999);

  TIM6::setAutoReload(1999);

  TIM6::generateUpdate();

  NVIC::enableInterrupt<
      nvic::irqn::TIM6_DAC
  >();

  TIM6::startCounter();

  while (true) {
  }

}

void interrupt::TIM6_DAC()
{
  static u32 debug = 0;

  TIM6::clearFlag();

  debug++;

  if (debug % 2)
    PA0::setLow();
  else
    PA0::setHigh();
}
