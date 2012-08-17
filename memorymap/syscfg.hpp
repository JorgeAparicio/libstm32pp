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
    ADDRESS = alias::APB2 + 0x3800
  };

  namespace registers {
    namespace memrmp {
      enum {
        OFFSET = 0x00
      };
    // TODO SYSCFG MEMRMP bits
    }// namespace memrmp

    namespace pmc {
      enum {
        OFFSET = 0x04
      };
    // TODO SYSCFG PMC bits
    }// namespace pmc

    namespace exticr1 {
      enum {
        OFFSET = 0x08
      };
    }  // namespace exticr1

    namespace exticr2 {
      enum {
        OFFSET = 0x0C
      };
    }  // namespace exticr2

    namespace exticr3 {
      enum {
        OFFSET = 0x10
      };
    }  // namespace exticr3

    namespace exticr4 {
      enum {
        OFFSET = 0x14
      };
    }  // namespace exticr4

    namespace exticr {
      enum {
        MASK = 0b1111
      };

      namespace states {
        enum E {
          PA = 0,
          PB = 1,
          PC = 2,
          PD = 3,
          PE = 4,
          PF = 5,
          PG = 6,
          PH = 7,
          PI = 8,
        };
      }  // namespace states

    }  // namespace exticr

    namespace cmpcr {
      enum {
        OFFSET = 0x20
      };
    }  // namespace cmpcr
  // TODO SYSCFG CMPCR bits
  }// namespace registers
}  // namespace syscfg
