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

#include "interrupt.hpp"
#include "peripheral/gpio.hpp"
#include "peripheral/rcc.hpp"
#include "peripheral/tim.hpp"
#include "core/nvic.hpp"

// TODO Test TIM demo on STM32F1XX
typedef PA0 LED;

int main()
{
  clk::initialize();

  LED::enableClock();

#ifdef STM32F1XX
  LED::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
#else
  LED::setMode(gpio::moder::OUTPUT);
#endif

  TIM6::enableClock();

  TIM6::configurePeriodicInterrupt<
      1 /* Hz */
  >();

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
  static u32 counter = 0;

  TIM6::clearUpdateFlag();

  counter++;

  if (counter % 2)
    LED::setLow();
  else
    LED::setHigh();
}
