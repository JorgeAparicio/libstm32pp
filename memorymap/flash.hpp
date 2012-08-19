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

namespace flash {
  enum {
#ifdef STM32F1XX
    ADDRESS = alias::AHB + 0x2000
#else // STM32F1XX
  ADDRESS = alias::AHB1 + 0x3C00
#endif // STM32F1XX
};

struct Registers {
    __RW
    u32 ACR;  // 0x00: Access control
};

#ifdef STM32F1XX
namespace acr {
  enum {
    OFFSET = 0x00
  };
#ifndef VALUE_LINE
  namespace latency {
    enum {
      POSITION = 0,
      MASK = 0b111 << POSITION
    };
    enum States {
      ZERO_WAIT_STATE = 0 << POSITION,
      ONE_WAIT_STATE = 1 << POSITION,
      TWO_WAIT_STATES = 2 << POSITION,
    };
  }  // namespace latency
#endif
  namespace hlfcya {
    enum {
      POSITION = 3,
      MASK = 1 << POSITION
    };
    enum States {
      FLASH_HALF_CYCLE_ACCESS_DISABLED = 0 << POSITION,
      FLASH_HALF_CYCLE_ACCESS_ENABLED = 1 << POSITION,
    };
  }  // namespace prftbe
#ifndef VALUE_LINE
  namespace prftbe {
    enum {
      POSITION = 4,
      MASK = 1 << POSITION
    };
    enum States {
      PREFETCH_DISABLED = 0 << POSITION,
      PREFETCH_ENABLED = 1 << POSITION,
    };
  }  // namespace prftbe
#endif
}  // namespace acr
#else /* STM32F2XX || STM32F4XX */
namespace acr {
  enum {
    OFFSET = 0x00
  };
  namespace latency {
    enum {
      POSITION = 0,
      MASK = 0b111 << POSITION
    };
    enum States {
      ZERO_WAIT_STATE = 0 << POSITION,
      ONE_WAIT_STATE = 1 << POSITION,
      TWO_WAIT_STATES = 2 << POSITION,
      THREE_WAIT_STATES = 3 << POSITION,
      FOUR_WAIT_STATES = 4 << POSITION,
      FIVE_WAIT_STATES = 5 << POSITION,
      SIX_WAIT_STATES = 6 << POSITION,
      SEVEN_WAIT_STATES = 7 << POSITION,
    };
  }  // namespace latency

  namespace prften {
    enum {
      POSITION = 8,
      MASK = 1 << POSITION
    };
    enum States {
      PREFETCH_DISABLED = 0 << POSITION,
      PREFETCH_ENABLED = 1 << POSITION,
    };
  }  // namespace prften

  namespace icen {
    enum {
      POSITION = 9,
      MASK = 1 << POSITION
    };
    enum States {
      INSTRUCTION_CACHE_DISABLED = 0 << POSITION,
      INSTRUCTION_CACHE_ENABLED = 1 << POSITION,
    };
  }  // namespace icen

  namespace dcen {
    enum {
      POSITION = 10,
      MASK = 1 << POSITION
    };
    enum States {
      DATA_CACHE_DISABLED = 0 << POSITION,
      DATA_CACHE_ENABLED = 1 << POSITION,
    };
  }  // namespace dcen
}  // namespace acr
#endif /* STM32F1XX */
}  // namespace flash
