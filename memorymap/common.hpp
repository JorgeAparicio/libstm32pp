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

#include "../include/device_select.hpp"

namespace alias {
  enum Address {
    FLASH = 0x08000000,
    SRAM = 0x20000000,
    PERIPH = 0x40000000,
    PPB = 0xE000E000,
    DBG = 0xE0042000,
    FSMC = 0xA0000000,
    APB1 = PERIPH,
    APB2 = PERIPH + 0x00010000,
    #ifdef STM32F1XX
    USB = 0x50000000,
    AHB = PERIPH + 0x00020000,

#elif defined (STM32F2XX)
    AHB1 = PERIPH + 0x00020000,
    AHB2 = PERIPH + 0x10000000,

#else /* STM32F4XX */
    CCMDATARAM = 0x10000000,
    SRAM1 = SRAM,
    SRAM2 = SRAM + 0x1C000,
    BKPSRAM = 0x40024000,
    AHB1 = PERIPH + 0x00020000,
    AHB2 = PERIPH + 0x10000000,
#endif
  };
}  // namespace alias

namespace bitband {
  enum Address {
    SRAM = 0x22000000,
    PERIPH = 0x42000000,
  };
}  // namespace bitband
