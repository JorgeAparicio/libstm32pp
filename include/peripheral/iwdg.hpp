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

/*******************************************************************************
 *
 *                           Independent WATCHDOG
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"

#include "../defs.hpp"
#include "../../memorymap/iwdg.hpp"

// Low-level access to the registers
#define IWDG_REGS reinterpret_cast<iwdg::Registers *>(iwdg::ADDRESS)

// High-level functions
namespace iwdg {
  class Functions {
    public:
      // TODO IWDG functions declaration
    private:
      Functions();
  };
}  // namespace iwdg

// High-level access to the peripheral
typedef iwdg::Functions IWDG;

#include "../../bits/iwdg.tcc"
