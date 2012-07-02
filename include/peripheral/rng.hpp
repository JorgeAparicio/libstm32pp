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
 *                          Random Number Generator
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"

#ifndef STM32F1XX

#include "../defs.hpp"

#include "../../memorymap/rng.hpp"

// Low-level access to registers
#define _RNG reinterpret_cast<rng::Registers *>(rng::ADDRESS)

// High-level functions
namespace rng {
  class Functions {
    public:
      static INLINE void enableGenerator();
      static INLINE void disableGenerator();
      static INLINE void enableInterrupts();
      static INLINE void disableInterrupts();

      template<typename T>
      static INLINE T getValue();

      static INLINE bool isDataReady(void);
      static INLINE bool isSeedValid(void);
      static INLINE bool isClockValid(void);
      static INLINE void clearSeedErrorFlag(void);
      static INLINE void clearClockErrorFlag(void);

    private:
      Functions();
  };

}  // namespace rng

// High-level access to the peripheral
typedef rng::Functions RNG;

#include "../../bits/rng.tcc"

#endif // !STM32F1XX
