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
#define _FLASH  reinterpret_cast<flash::Registers*>(flash::ADDRESS)

// High-level functions
namespace flash {
  class Functions {
    public:
#ifndef VALUE_LINE
      template<
          flash::registers::acr::bits::latency::states::E
      > static INLINE void setLatency();
      #endif
#ifdef STM32F1XX
#ifndef VALUE_LINE
      static INLINE void enablePrefetch(void);
      static INLINE void disablePrefetch(void);

#endif
      static INLINE void enableHalfCycleFlashAccess(void);
      static INLINE void disableHalfCycleFlashAccess(void);
#ifdef VALUE_LINE
      template <
      flash::registers::acr::bits::hlfcya::states::E
      >
      static INLINE void configure(void);
#else
      template<
      flash::registers::acr::bits::latency::states::E,
      flash::registers::acr::bits::hlfcya::states::E,
      flash::registers::acr::bits::prftbe::states::E
      >
      static INLINE void configure(void);
#endif
#else // STM32F1XX
      static INLINE void enablePrefetch(void);
      static INLINE void disablePrefetch(void);
      static INLINE void enableDataCache(void);
      static INLINE void disableDataCache(void);
      static INLINE void enableInstructionCache(void);
      static INLINE void disableInstructionCache(void);

      template<
          flash::registers::acr::bits::latency::states::E,
          flash::registers::acr::bits::prften::states::E,
          flash::registers::acr::bits::dcen::states::E,
          flash::registers::acr::bits::icen::states::E
      >
      static INLINE void configure(void);

#endif // STM32F1XX
    private:
      Functions();
  };
}  // namespace flash

// High-level access to the peripheral
typedef flash::Functions FLASH;

#include "../../bits/flash.tcc"
