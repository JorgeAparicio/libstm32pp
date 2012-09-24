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
////////////////////////////////////////////////////////////////////////////////
// Tested on STM32VLDISCOVERY
// Tested on STM32F4DISCOVERY
// Tested on F4Dev
#include "clock.hpp"

#include "interrupt.hpp"

#include "peripheral/gpio.hpp"

typedef PA0 LED;

#include "peripheral/tim.hpp"

void initializeGpio()
{
  LED::enableClock();

#ifdef STM32F1XX
  LED::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
#else
  LED::setMode(gpio::moder::OUTPUT);
#endif
}

void initializeTimer()
{
  TIM6::enableClock();
  TIM6::configurePeriodicInterrupt<
      4 /* Hz */
  >();
}

void initializePeripherals()
{
  initializeGpio();
  initializeTimer();

  TIM6::startCounter();
}

void loop()
{

}

int main()
{
  clk::initialize();

  initializePeripherals();

  while (true) {
    loop();
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
  static u8 counter = 0;

  TIM6::clearUpdateFlag();

  if (counter++ % 2)
    LED::setHigh();
  else
    LED::setLow();
}
