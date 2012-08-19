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
 *                               FLASH Registers
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"

#include "../defs.hpp"
#include "../../memorymap/flash.hpp"

// Low-level access to the registers
#define FLASH_REGS reinterpret_cast<flash::Registers*>(flash::ADDRESS)

// High-level functions
namespace flash {
  class Functions {
    public:
#ifndef VALUE_LINE
      static inline void setLatency(flash::acr::latency::States);
#endif // VALUE_LINE
#ifdef STM32F1XX
#ifndef VALUE_LINE
      static inline void enablePrefetch();
      static inline void disablePrefetch();

#endif // !VALUE_LINE
      static inline void enableHalfCycleFlashAccess();
      static inline void disableHalfCycleFlashAccess();
#ifdef VALUE_LINE
      static inline void configure(flash::acr::hlfcya::States);
#else // VALUE_LINE
      static inline void configure(
          flash::acr::latency::States,
          flash::acr::hlfcya::States,
          flash::acr::prftbe::States);
#endif // VALUE_LINE
#else // STM32F1XX
      static inline void enablePrefetch();
      static inline void disablePrefetch();
      static inline void enableDataCache();
      static inline void disableDataCache();
      static inline void enableInstructionCache();
      static inline void disableInstructionCache();

      static inline void configure(
          flash::acr::latency::States,
          flash::acr::prften::States,
          flash::acr::dcen::States,
          flash::acr::icen::States);

#endif // STM32F1XX
    private:
      Functions();
  };
}  // namespace flash

// High-level access to the peripheral
typedef flash::Functions FLASH;

#include "../../bits/flash.tcc"
