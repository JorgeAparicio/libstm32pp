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

#include "defs.hpp"
#include "../memorymap/common.hpp"

namespace bitband {
  template<u32 Address, u8 Bit>
  struct Peripheral {
      static_assert(
          ((Address - alias::PERIPH) >= 0) &&
          ((Address - alias::PERIPH) < 0x100000),
          "This is not a valid Peripheral address for bit-banding."
      );

      static_assert(Bit < 32, "Only 32 bits can be bit-banded.");

      enum {
        address = PERIPH + ((Address - alias::PERIPH) << 5) + (Bit << 2),
      };
  };

  template<u32 Address, u8 Bit>
  struct Ram {
      static_assert(
          ((Address - alias::SRAM) >= 0) &&
          ((Address - alias::SRAM) < 0x100000),
          "This is not a valid RAM address for bit-banding."
      );

      static_assert(Bit < 32, "Only 32 bits can be bit-banded.");

      enum {
        address = SRAM + ((Address - alias::SRAM) << 5) + (Bit << 2),
      };
  };
}
