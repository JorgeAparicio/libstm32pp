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
 *                   Consumer Electronic Control Controller
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"
#include "../defs.hpp"

#ifdef VALUE_LINE

#include "../../memorymap/cec.hpp"

// Low-level access to the registers
#define CEC_REGS reinterpret_cast<cec::Registers *>(cec::ADDRESS)

// High-level functions
namespace cec {
  class Functions {
    public:
      // TODO CEC function declaration
    private:
      Functions();
  };
}  // namespace cec

#include "../../bits/cec.tcc"

// High-level access to the peripheral
// TODO CEC high-level access

#endif
