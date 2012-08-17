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

namespace iwdg {
  struct Registers {
      __RW
      u32 KR;  // 0x00: Key
      __RW
      u32 PR;  // 0x04: Prescaler
      __RW
      u32 RLR;  // 0x08: Reload
      __RW
      u32 SR;  // 0x0C: Status
  };

  enum {
    ADDRESS = alias::APB1 + 0x3000
  };

  namespace registers {
  // TODO IWDG register bits
  }// namespace registers
}  // namespace iwdg
