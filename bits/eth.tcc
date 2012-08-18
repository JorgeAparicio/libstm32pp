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

namespace eth {
  void Functions::enableClocks()
  {
#ifndef STM32F1XX
    RCC::enableClocks<
    rcc::ahb1enr::ETH_MAC,
    rcc::ahb1enr::ETH_MAC_TX,
    rcc::ahb1enr::ETH_MAC_RX,
    rcc::ahb1enr::ETH_MAC_PTP
    >();
#else // !STM32F1XX
    RCC::enableClocks<
        rcc::ahbenr::ETH_MAC,
        rcc::ahbenr::ETH_MAC_TX,
        rcc::ahbenr::ETH_MAC_RX
    >();
#endif // STM32F1XX
  }

  void Functions::disableClocks()
  {
#ifndef STM32F1XX
    RCC::disableClocks<
    rcc::ahb1enr::ETH_MAC,
    rcc::ahb1enr::ETH_MAC_TX,
    rcc::ahb1enr::ETH_MAC_RX,
    rcc::ahb1enr::ETH_MAC_PTP
    >();
#else // !STM32F1XX
    RCC::disableClocks<
        rcc::ahbenr::ETH_MAC,
        rcc::ahbenr::ETH_MAC_TX,
        rcc::ahbenr::ETH_MAC_RX
    >();
#endif // STM32F1XX
  }
}  // namespace eth
