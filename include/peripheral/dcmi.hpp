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
 *                          Digital Camera Interface
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"
#include "../defs.hpp"

#ifndef STM32F1XX

#include "../../memorymap/dcmi.hpp"

// Low-level access to the registers
#define _DCMI reinterpret_cast<dcmi::Registers *>(dcmi::ADDRESS)

// High-level functions
namespace dcmi {
  class Functions {
    public:
      // TODO DCMI functions declaration
    private:
      Functions();
  };
}  // namespace dcmi

#include "../../bits/dcmi.tcc"

// High-level access to the peripheral
// TODO

#endif
