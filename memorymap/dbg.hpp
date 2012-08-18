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

namespace dbg {
  enum {
    ADDRESS = alias::DBG
  };

  struct Registers {
      __RW
      u32 IDCODE;  // 0x00: MCU device ID code
      __RW
      u32 CR;      // 0x04: Configuration
#ifndef STM32F1XX
      __RW
      u32 APB1FZ;  // 0x08: APB1 freeze register
      __RW
      u32 APB2FZ;  // 0x0C: APB2 freeze register
#endif
  };

  namespace idcode {
    enum {
      OFFSET = 0x00
    };
  }  // namespace idcode

  namespace cr {
    enum {
      OFFSET = 0x04
    };
  }  // namespace cr

#ifndef STM32F1XX
  namespace apb1fz {
    enum {
      OFFSET = 0x08
    };
  }  // namespace apb1fz

  namespace apb2fz {
    enum {
      OFFSET = 0x0C
    };
  }  // namespace apb2fz
#endif
}  // namespace dbg
