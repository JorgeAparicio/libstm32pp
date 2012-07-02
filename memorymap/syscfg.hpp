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

namespace syscfg {
  struct Registers {
      __RW
      u32 MEMRMP;  // 0x00: Memory remap
      __RW
      u32 PMC;  // 0x04: Peripheral mode configuration
      __RW
      u32 EXTICR[4];  // 0x08: External interrupt configuration
      u32 _RESERVED[2];
      __RW
      u32 CMPCR;  // 0x20: Compensation cell control
  };

  enum {
    ADDRESS = alias::address::APB2 + 0x3800
  };

  namespace registers {
  // TODO SYSCFG register bits
  }// namespace registers
}  // namespace syscfg
