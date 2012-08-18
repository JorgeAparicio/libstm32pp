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
 *                    External Interrupt/Event Controller
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"
#include "../defs.hpp"

#include "../../memorymap/exti.hpp"

// Low-level access to the registers
#define EXTI_REGS reinterpret_cast<exti::Registers *>(exti::ADDRESS)

// High-level functions
namespace exti {
  template<u8 LINE>
  class Functions {
      static_assert(LINE < 23, "There are only 23 (0-22) external interrupt lines");

    public:
      static inline void clearPendingFlag();
      static inline void disableAll();
      static inline void disableEvent();
      static inline void disableInterrupt();
      static inline void enableHardwareEventByFallingEdge();
      static inline void enableHardwareEventByRisingEdge();
      static inline void enableHardwareInterruptByFallingEdge();
      static inline void enableHardwareInterruptByRisingEdge();
      static inline void enableSoftwareEvent();
      static inline void enableSoftwareInterrupt();

    private:
      Functions();
  };
}  // namespace exti

// High-level access to the peripheral
typedef exti::Functions<0> EXTI0;
typedef exti::Functions<1> EXTI1;
typedef exti::Functions<2> EXTI2;
typedef exti::Functions<3> EXTI3;
typedef exti::Functions<4> EXTI4;
typedef exti::Functions<5> EXTI5;
typedef exti::Functions<6> EXTI6;
typedef exti::Functions<7> EXTI7;
typedef exti::Functions<8> EXTI8;
typedef exti::Functions<9> EXTI9;
typedef exti::Functions<10> EXTI10;
typedef exti::Functions<11> EXTI11;
typedef exti::Functions<12> EXTI12;
typedef exti::Functions<13> EXTI13;
typedef exti::Functions<14> EXTI14;
typedef exti::Functions<15> EXTI15;
typedef exti::Functions<16> EXTI16;
typedef exti::Functions<17> EXTI17;
typedef exti::Functions<18> EXTI18;
typedef exti::Functions<19> EXTI19;
typedef exti::Functions<20> EXTI20;
typedef exti::Functions<21> EXTI21;
typedef exti::Functions<22> EXTI22;

#include "../../bits/exti.tcc"
