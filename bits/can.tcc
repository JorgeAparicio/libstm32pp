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

namespace can {
  template<address::E C>
  void Functions<C>::enableClocks()
  {
    switch (C) {
#if defined CONNECTIVITY_LINE || not defined STM32F1XX
      case address::CAN1:
        RCC::enableClocks<rcc::apb1enr::CAN1>();
        break;
      case address::CAN2:
        RCC::enableClocks<rcc::apb1enr::CAN2>();
        break;
#else // !CONNECTIVTY_LINE || STM32F1XX
        case address::CAN:
        RCC::enableClocks<rcc::apb1enr::CAN>();
        break;
#endif // !CONNECTIVITY_LINE
    }
  }

  template<address::E C>
  void Functions<C>::disableClocks()
  {
    switch (C) {
#if defined CONNECTIVITY_LINE || not defined STM32F1XX
      case address::CAN1:
        RCC::disableClocks<rcc::apb1enr::CAN1>();
        break;
      case address::CAN2:
        RCC::disableClocks<rcc::apb1enr::CAN2>();
        break;
#else // !CONNECTIVTY_LINE || STM32F1XX
        case address::CAN:
        RCC::disableClocks<rcc::apb1enr::CAN>();
        break;
#endif // !CONNECTIVITY_LINE
    }
  }
}  // namespace can
