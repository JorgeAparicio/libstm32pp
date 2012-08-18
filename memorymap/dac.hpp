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

namespace dac {
  enum {
    ADDRESS = alias::APB1 + 0x7400
  };

  struct Registers {
      __RW
      u32 CR;       // 0x00: Control
      __RW
      u32 SWTRIGR;  // 0x04: Software trigger
      struct {
          __RW
          u32 R12;  // 0x08, 0x14: 12-bit right-aligned
          __RW
          u32 L12;  // 0x0C, 0x18: 12-bit left-aligned
          __RW
          u32 R8;   // 0x10, 0x1C: 8-bit right-aligned
      } DHR[2];     // Channel data holding
      __RW
      u32 DHR12RD;  // 0x20: Dual 12-bit right-aligned data holding
      __RW
      u32 DHR12LD;  // 0x24: Dual 12-bit left-aligned data holding
      __RW
      u32 DHR8RD;   // 0x28: Dual 8-bit right-aligned data holding
      __RW
      u32 DOR[2];   // 0x2C: Channel data output
#if defined VALUE_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX
      __RW
      u32 SR;      // 0x34: Status
#endif
  };

  namespace cr {
    enum {
      OFFSET = 0x00
    };
  }  // namespace cr

  namespace swtrigr {
    enum {
      OFFSET = 0x04
    };
  }  // namespace swtrigr

  namespace sr {
    enum {
      OFFSET = 0x34
    };
  }  // namespace sr
}  // namespace dac
