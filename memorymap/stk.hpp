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

namespace stk {
  struct Registers {
      __RW
      u32 CTRL;  // 0x00: Control and status
      __RW
      u32 LOAD;  // 0x04: Reload value
      __RW
      u32 VAL;  // 0x08: Current value
      __RW
      u32 CALIB;  // 0x0C: Calibration value
  };

  enum {
    ADDRESS = alias::PPB + 0x10
  };

  namespace registers {
  // TODO STK register bits
  }// namespace registers
}  // namespace stk
