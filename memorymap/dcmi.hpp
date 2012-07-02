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

namespace dcmi {
  struct Registers {
      __RW
      u32 CR;      // 0x00: Control 1
      __RW
      u32 SR;      // 0x00: Status
      __RW
      u32 RISR;    // 0x00: Raw interrupt status
      __RW
      u32 IER;     // 0x00: Interrupt enable
      __RW
      u32 MISR;    // 0x00: Masked interrupt status
      __RW
      u32 ICR;     // 0x00: Interrupt clear
      __RW
      u32 ESCR;    // 0x00: Embedded synchronization code
      __RW
      u32 ESUR;    // 0x00: Embedded synchronization unmask
      __RW
      u32 CWSTRTR;  // 0x00: Crop window start
      __RW
      u32 CWSIZER;  // 0x00: Crop window size
      __RW
      u32 DR;      // 0x00: Data
  };

  enum {
    ADDRESS = alias::address::AHB2 + 0x50000
  };

  namespace registers {
  // TODO DCMI register bits
  }// namespace registers
}  // namespace dcmi
