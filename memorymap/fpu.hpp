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

namespace fpu {

  struct Registers {
      __RW
      u32 CPACR;  // 0x000: Coprocessor access control
      u32 _RESERVED[106];
      __RW
      u32 FPCCR;  // 0x1AC: Floating point context control
      __RW
      u32 FPCAR;  // 0x1B0: Floating point context address
      __RW
      u32 FPDSCR;  // 0x1B4: Floating point default status control
  };

  enum {
    ADDRESS = alias::PPB + 0xD88
  };

  namespace registers {
    namespace cpacr {
      enum {
        OFFSET = 0x00
      };
      namespace bits {
        namespace cp10 {
          enum {
            POSITION = 20
          };
          enum {
            MASK = 0b11 << POSITION
          };
          namespace states {
            enum E {
              ACCESS_DENIED = 0 << POSITION,
              PRIVILEGED_ACCESS = 1 << POSITION,
              FULL_ACCESS = 3 << POSITION
            };
          }
        }
        namespace cp11 {
          enum {
            POSITION = 22
          };
          enum {
            MASK = 0b11 << POSITION
          };
          namespace states {
            enum E {
              ACCESS_DENIED = 0 << POSITION,
              PRIVILEGED_ACCESS = 1 << POSITION,
              FULL_ACCESS = 3 << POSITION
            };
          }
        }
      }
    }
  }  // namespace registers

}  // namespace fpu

