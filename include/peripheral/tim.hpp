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

/*******************************************************************************
 *
 *               Basic/General-Purpose/Advanced-Control Timers
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"
#include "../defs.hpp"

#include "../clock.hpp"
#include "../../memorymap/tim.hpp"

// Low-level access to the registers
#define TIM6_REGS   ((tim::_reserved::RegistersB*) tim::address::TIM6)
#define TIM7_REGS   ((tim::_reserved::RegistersB*) tim::address::TIM7)

#define TIM2_REGS   ((tim::_reserved::RegistersGP0*)tim::address::TIM2)
#define TIM3_REGS   ((tim::_reserved::RegistersGP1*)tim::address::TIM3)
#define TIM4_REGS   ((tim::_reserved::RegistersGP1*)tim::address::TIM4)
#define TIM5_REGS   ((tim::_reserved::RegistersGP2*)tim::address::TIM5)

#define TIM9_REGS   ((tim::_reserved::RegistersGP3*)tim::address::TIM9)
#define TIM12_REGS  ((tim::_reserved::RegistersGP3*)tim::address::TIM12)

#define TIM10_REGS  ((tim::_reserved::RegistersGP4*)tim::address::TIM10)
#define TIM11_REGS  ((tim::_reserved::RegistersGP5*)tim::address::TIM11)
#define TIM13_REGS  ((tim::_reserved::RegistersGP4*)tim::address::TIM13)
#define TIM14_REGS  ((tim::_reserved::RegistersGP4*)tim::address::TIM14)

#ifdef VALUE_LINE
#define TIM15_REGS  ((tim::_reserved::RegistersGP6*)tim::address::E::TIM15)
#define TIM16_REGS  ((tim::_reserved::Registers*)tim::address::E::TIM16)
#define TIM17_REGS  ((tim::_reserved::Registers*)tim::address::E::TIM17)
#endif

#define TIM1_REGS   ((tim::_reserved::RegistersAC*)tim::address::E::TIM1)
#define TIM8_REGS   ((tim::_reserved::RegistersAC*)tim::address::E::TIM8)

// High-level functions
namespace tim {
  template<address::E A>
  class Functions {
    public:
      enum {
        FREQUENCY = u32(A) > u32(alias::APB2) ?
                                                         clock::APB2_TIMERS :
                                                         clock::APB1_TIMERS
      };

      static inline void enableClock();
      static inline void disableClock();
      static inline void startCounter();
      static inline void stopCounter();
      static inline void setMicroSecondResolution();
      static inline void setMiliSecondResolution();
      static void delay(u16 const);
      static inline void setPrescaler(u16 const);
      static inline void setAutoReload(u16 const);
      static inline void setCounter(u16 const);
      static inline u16 getCounter();
      static inline void generateUpdate();
      static inline void enableUpdateInterrupt();
      static inline void disableUpdateInterrupt();
      static inline void clearUpdateFlag();
      static inline void enableUpdateDma();
      static inline void disableUpdateDma();
      static inline bool hasUpdateEventOccurred();

      template<
          u32
      >
      static inline void configurePeriodicInterrupt();

      template<
          tim::registers::cr2::bits::mms::states::E
      >
      static inline void setMasterMode();

      template<
          tim::registers::cr1::bits::cen::states::E,
          tim::registers::cr1::bits::udis::states::E,
          tim::registers::cr1::bits::urs::states::E,
          tim::registers::cr1::bits::opm::states::E,
          tim::registers::cr1::bits::arpe::states::E
      >
      static inline void configureBasicCounter();

      // TODO TIM capture functions
      // TODO TIM compare functions
      // TODO TIM pwm functions

    private:
      Functions();
  };
}  // namespace tim

// High-level access to the peripheral
typedef tim::Functions<tim::address::TIM1> TIM1;
typedef tim::Functions<tim::address::TIM2> TIM2;
typedef tim::Functions<tim::address::TIM3> TIM3;
typedef tim::Functions<tim::address::TIM4> TIM4;
typedef tim::Functions<tim::address::TIM5> TIM5;
typedef tim::Functions<tim::address::TIM6> TIM6;
typedef tim::Functions<tim::address::TIM7> TIM7;
typedef tim::Functions<tim::address::TIM8> TIM8;
typedef tim::Functions<tim::address::TIM9> TIM9;
typedef tim::Functions<tim::address::TIM10> TIM10;
typedef tim::Functions<tim::address::TIM11> TIM11;
typedef tim::Functions<tim::address::TIM12> TIM12;
typedef tim::Functions<tim::address::TIM13> TIM13;
typedef tim::Functions<tim::address::TIM14> TIM14;
#ifdef VALUE_LINE
typedef tim::Functions<tim::address::TIM15> TIM15;
typedef tim::Functions<tim::address::TIM16> TIM16;
typedef tim::Functions<tim::address::TIM17> TIM17;
#endif

#include "../../bits/tim.tcc"
