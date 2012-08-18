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

namespace cryp {
  enum {
    ADDRESS = alias::AHB2 + 0x60000
  };

  struct Registers {
      __RW
      u32 CR;     // 0x00: Control
      __RW
      u32 SR;     // 0x04: Status
      __RW
      u32 DR;     // 0x08: Data input
      __RW
      u32 DOUT;   // 0x0C: Data output
      __RW
      u32 DMACR;  // 0x10: DMA control
      __RW
      u32 IMSCR;  // 0x14: Interrupt mask set/clear
      __RW
      u32 RISR;   // 0x18: Raw interrupt status
      __RW
      u32 MISR;   // 0x1C: Masked interrupt status
      struct {
          __RW
          u32 L;  // 0x20, 0x28, 0x30, 0x38: Left word
          __RW
          u32 R;  // 0x24, 0x2C, 0x34, 0x3C: Right word
      } KR[4];    // Key
      struct {
          __RW
          u32 L;  // 0x40, 0x48: Left word
          __RW
          u32 R;  // 0x44, 0x4C: Right word
      } IVR[2];   // Initialization vector
  };

  namespace cr {
    enum {
      OFFSET = 0x00
    };
  }  // namespace cr

  namespace sr {
    enum {
      OFFSET = 0x04
    };
  }  // namespace sr

  namespace dr {
    enum {
      OFFSET = 0x08
    };
  }  // namespace dr

  namespace dout {
    enum {
      OFFSET = 0x0C
    };
  }  // namespace dout

  namespace dmacr {
    enum {
      OFFSET = 0x10
    };
  }  // namespace dmacr

  namespace imscr {
    enum {
      OFFSET = 0x14
    };
  }  // namespace imscr

  namespace risr {
    enum {
      OFFSET = 0x18
    };
  }  // namespace risr

  namespace misr {
    enum {
      OFFSET = 0x1C
    };
  }  // namespace misr
}  // namespace cryp
