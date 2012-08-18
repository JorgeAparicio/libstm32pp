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

namespace exti {
  enum {
#ifdef STM32F1XX
    ADDRESS = alias::APB2 + 0x0400
#else
    ADDRESS = alias::APB2 + 0x3C00
#endif
  };

  struct Registers {
      __RW
      u32 IMR;  // 0x00: Interrupt mask
      __RW
      u32 EMR;  // 0x04: Event mask
      __RW
      u32 RTSR;  // 0x08: Rising trigger selection
      __RW
      u32 FTSR;  // 0x0C: Falling trigger selection
      __RW
      u32 SWIER;  // 0x10: Software interrupt event
      __RW
      u32 PR;  // 0x14: Pending
  };

  namespace imr {
    enum {
      OFFSET = 0x00
    };
  }  // namespace imr

  namespace emr {
    enum {
      OFFSET = 0x04
    };
  }  // namespace imr

  namespace rtsr {
    enum {
      OFFSET = 0x08
    };
  }  // namespace rtsr

  namespace ftsr {
    enum {
      OFFSET = 0x0C
    };
  }  // namespace ftsr

  namespace swier {
    enum {
      OFFSET = 0x10
    };
  }  // namespace swier

  namespace pr {
    enum {
      OFFSET = 0x14
    };
  }  // namespace pr

}  // namespace exti
