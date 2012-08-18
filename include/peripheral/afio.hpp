/*******************************************************************************
 *
 * This file is part of libstm32pp.
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
 *                           Alternate Function I/O
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"

#ifdef STM32F1XX

#include "../defs.hpp"
#include "../../memorymap/afio.hpp"

// Low-level access to the registers
#define AFIO_REGS reinterpret_cast<afio::Registers *>(afio::ADDRESS)

// High-level functions
namespace afio {
  class Functions {
    public:
      static inline void enableClock();
      static inline void disableClock();
    private:
      Functions();
  };
}  // namespace afio"

// High-level access to the peripheral
typedef afio::Functions AFIO;

#include "../../bits/afio.tcc"

#endif
