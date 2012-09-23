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
#include "clock.hpp"

#include "interrupt.hpp"

#include "peripheral/gpio.hpp"

typedef PD13 LD3; // ORANGE
typedef PD12 LD4; // GREEN
typedef PD14 LD5; // RED
typedef PD15 LD6; // BLUE

#include "peripheral/tim.hpp"

void initializeGpio()
{
  GPIOD::enableClock();

  LD3::setMode(gpio::moder::OUTPUT);
  LD4::setMode(gpio::moder::OUTPUT);
  LD6::setMode(gpio::moder::OUTPUT);
  LD5::setMode(gpio::moder::OUTPUT);
}

void loop()
{
  LD3::setLow();
  LD4::setLow();
  LD5::setLow();
  LD6::setLow();

  LD3::setHigh();
  LD4::setHigh();
  LD5::setHigh();
  LD6::setHigh();
}

int main()
{
  clk::initialize();

  initializeGpio();

  while (true) {
    loop();
  }
}
