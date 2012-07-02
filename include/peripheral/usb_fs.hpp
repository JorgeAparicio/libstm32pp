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
 *                       Universal Serial Bus Full Speed
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"

#ifndef VALUE_LINE

#include "../defs.hpp"

#include "../../memorymap/usb_fs.hpp"

// Low-level access to the registers
#define _USB_FS reinterpret_cast<usb_fs::Registers*>(usb_fs::ADDRESS)

// High-level functions
namespace usb_fs {
  class Functions {
    public:
      // TODO USB_FS functions declaration
    private:
      Functions();
  };
}  // namespace usb_fs

// High-level access to the peripheral
// TODO USB_FS high-level access

#include "../../bits/usb_fs.tcc"

#endif
