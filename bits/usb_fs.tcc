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

#include "../include/peripheral/rcc.hpp"

namespace usb_fs {
  void Functions::enableClock()
  {
#ifdef CONNECTIVITY_LINE
    RCC::enableClocks<
    rcc::ahbenr::USB_OTG_FS
    >();
#else
    RCC::enableClocks<
        rcc::ahb2enr::USB_OTG_FS
    >();
#endif
  }

  void Functions::disableClock()
  {
#ifdef CONNECTIVITY_LINE
    RCC::disableClocks<
    rcc::ahbenr::USB_OTG_FS
    >();
#else
    RCC::disableClocks<
        rcc::ahb2enr::USB_OTG_FS
    >();
#endif
  }
}  // namespace usb_fs
