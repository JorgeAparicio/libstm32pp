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

#include "peripheral/gpio.hpp"
#include "peripheral/rcc.hpp"

// TODO Test GPIO demo on STM32F1XX

int main()
{
  PA0::enableClock();
#ifdef STM32F1XX
  PA0::setMode(gpio::cr::GP_PUSH_PULL_10MHZ);
#else // STM32F1XX
  PA0::setMode(gpio::moder::OUTPUT);
#endif // STM32F1XX

  while (true) {
    PA0::setLow();
    PA0::setHigh();
  }
}
