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

namespace wwdg {
  struct Registers {
      __RW
      u32 CR;  // 0x00: Control
      __RW
      u32 CFR;  // 0x04: Configuration
      __RW
      u32 SR;  // 0x08: Status
  };

  enum {
    ADDRESS = alias::APB1 + 0x2C00
  };

  namespace registers {
  // TODO WWDG register bits
  }// namespace registers
}  // namespace wwdg
