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

#include "core/fpu.hpp"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327
#endif

int main()
{
  float debug = 16;

  FPU::enableFullAccess();

  debug = sqrtf(debug);  // 4.0
  debug = 1 / debug;  // .25
  debug = powf(debug, .5);  // .5
  debug *= M_PI;  // PI / 2
  debug = sinf(debug);  // 1.0
  debug = 0.0;

}
