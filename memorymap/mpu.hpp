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

namespace mpu {
  enum {
    ADDRESS = alias::PPB + 0xD90
  };

  struct Registers {
      __RW
      u32 TYPER;  // 0x00: Type
      __RW
      u32 CR;  // 0x04: Control
      __RW
      u32 RNR;  // 0x08: Region number
      __RW
      u32 RBAR;  // 0x0C: Region base address
      __RW
      u32 RASR;  // 0x10: Region attribute and size
  };

  namespace typer {
    enum {
      OFFSET = 0x00
    };
  }  // namespace typer

  namespace control {
      enum {
        OFFSET = 0x04
      };
    }  // namespace control

  namespace rnr {
      enum {
        OFFSET = 0x08
      };
    }  // namespace rnr

  namespace rbar {
      enum {
        OFFSET = 0x0C
      };
    }  // namespace rbar

  namespace rasr {
      enum {
        OFFSET = 0x10
      };
    }  // namespace rasr
}  // namespace mpu
