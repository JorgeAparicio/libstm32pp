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
#define TIM1_REGS   ((tim::Registers*) tim::TIM1)
#define TIM2_REGS   ((tim::Registers*) tim::TIM2)
#define TIM3_REGS   ((tim::Registers*) tim::TIM3)
#define TIM4_REGS   ((tim::Registers*) tim::TIM4)
#define TIM5_REGS   ((tim::Registers*) tim::TIM5)
#define TIM6_REGS   ((tim::Registers*) tim::TIM6)
#define TIM7_REGS   ((tim::Registers*) tim::TIM7)
#define TIM8_REGS   ((tim::Registers*) tim::TIM8)
#define TIM9_REGS   ((tim::Registers*) tim::TIM9)
#define TIM10_REGS   ((tim::Registers*) tim::TIM10)
#define TIM11_REGS   ((tim::Registers*) tim::TIM11)
#define TIM12_REGS   ((tim::Registers*) tim::TIM12)
#define TIM13_REGS   ((tim::Registers*) tim::TIM13)
#define TIM14_REGS   ((tim::Registers*) tim::TIM14)
#ifdef VALUE_LINE
#define TIM15_REGS  ((tim::_reserved::RegistersGP6*)tim::E::TIM15)
#define TIM16_REGS  ((tim::_reserved::Registers*)tim::E::TIM16)
#define TIM17_REGS  ((tim::_reserved::Registers*)tim::E::TIM17)
#endif

// High-level functions
namespace tim {
  template<Address A>
  class Functions {
    public:
      enum {
        FREQUENCY =
        u32(A) > u32(alias::APB2) ?
                                    clk::APB2_TIMERS :
                                    clk::APB1_TIMERS
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

      static inline void setMasterMode(tim::cr2::mms::States);
      static inline void configureBasicCounter(
          tim::cr1::cen::States,
          tim::cr1::udis::States,
          tim::cr1::urs::States,
          tim::cr1::opm::States,
          tim::cr1::arpe::States);

      // TODO TIM capture functions
      // TODO TIM compare functions
      // TODO TIM pwm functions

    private:
      Functions();
  };
}  // namespace tim

// High-level access to the peripheral
typedef tim::Functions<tim::TIM1> TIM1;
typedef tim::Functions<tim::TIM2> TIM2;
typedef tim::Functions<tim::TIM3> TIM3;
typedef tim::Functions<tim::TIM4> TIM4;
typedef tim::Functions<tim::TIM5> TIM5;
typedef tim::Functions<tim::TIM6> TIM6;
typedef tim::Functions<tim::TIM7> TIM7;
typedef tim::Functions<tim::TIM8> TIM8;
typedef tim::Functions<tim::TIM9> TIM9;
typedef tim::Functions<tim::TIM10> TIM10;
typedef tim::Functions<tim::TIM11> TIM11;
typedef tim::Functions<tim::TIM12> TIM12;
typedef tim::Functions<tim::TIM13> TIM13;
typedef tim::Functions<tim::TIM14> TIM14;
#ifdef VALUE_LINE
typedef tim::Functions<tim::TIM15> TIM15;
typedef tim::Functions<tim::TIM16> TIM16;
typedef tim::Functions<tim::TIM17> TIM17;
#endif

#include "../../bits/tim.tcc"
