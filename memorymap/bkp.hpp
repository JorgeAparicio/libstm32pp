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

#include "common.hpp"

namespace bkp {
  namespace address {
    enum E {
      BKP = alias::address::APB1 + 0x6C00,
    };
  }  // namespace address

  struct Registers
  {
      u32 _RESERVED0;
      __RW
      u32 DR1[10];  // 0x04-0x28: Data 1
      __RW
      u32 RTCCR;    // 0x2C: RTC clock calibration
      __RW
      u32 CR;       // 0x30: Control
      __RW
      u32 CSR;      // 0x34: Control/status
      u32 _RESERVED1[2];
      __RW
      u32 DR2[32];  // 0x40-0xBC: Data 2
  };

  namespace registers {
  // TODO BKP register bits
  }// namespace registers
}  // namespace bkp
