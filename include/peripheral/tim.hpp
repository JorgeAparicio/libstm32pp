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
#define _TIM6   ((tim::_reserved::RegistersB*) tim::address::TIM6)
#define _TIM7   ((tim::_reserved::RegistersB*) tim::address::TIM7)

#define _TIM2   ((tim::_reserved::RegistersGP0*)tim::address::TIM2)
#define _TIM3   ((tim::_reserved::RegistersGP1*)tim::address::TIM3)
#define _TIM4   ((tim::_reserved::RegistersGP1*)tim::address::TIM4)
#define _TIM5   ((tim::_reserved::RegistersGP2*)tim::address::TIM5)

#define _TIM9   ((tim::_reserved::RegistersGP3*)tim::address::TIM9)
#define _TIM12  ((tim::_reserved::RegistersGP3*)tim::address::TIM12)

#define _TIM10  ((tim::_reserved::RegistersGP4*)tim::address::TIM10)
#define _TIM11  ((tim::_reserved::RegistersGP5*)tim::address::TIM11)
#define _TIM13  ((tim::_reserved::RegistersGP4*)tim::address::TIM13)
#define _TIM14  ((tim::_reserved::RegistersGP4*)tim::address::TIM14)

#ifdef VALUE_LINE
#define _TIM15  ((tim::_reserved::RegistersGP6*)tim::address::E::TIM15)
#define _TIM16  ((tim::_reserved::Registers*)tim::address::E::TIM16)
#define _TIM17  ((tim::_reserved::Registers*)tim::address::E::TIM17)
#endif

#define _TIM1   ((tim::_reserved::RegistersAC*)tim::address::E::TIM1)
#define _TIM8   ((tim::_reserved::RegistersAC*)tim::address::E::TIM8)

// High-level functions
namespace tim {
  template<address::E A>
  class Functions {
    public:
      enum {
        FREQUENCY = u32(A) > u32(alias::address::APB2) ?
                                               clock::APB2_TIMERS :
                                               clock::APB1_TIMERS
      };

      static INLINE void startCounter();
      static INLINE void stopCounter();
      static INLINE void setPrescaler(u16 const);
      static INLINE void setAutoReload(u16 const);
      static INLINE void setCounter(u16 const);
      static INLINE u16 getCounter();
      static INLINE void generateUpdate();
      static INLINE void enableUpdateInterrupt();
      static INLINE void disableUpdateInterrupt();
      static INLINE void clearUpdateFlag();
      static INLINE void enableUpdateDma();
      static INLINE void disableUpdateDma();

      template<
          tim::registers::cr2::bits::mms::states::E
      >
      static INLINE void setMasterMode();

      template<
          tim::registers::cr1::bits::cen::states::E,
          tim::registers::cr1::bits::udis::states::E,
          tim::registers::cr1::bits::urs::states::E,
          tim::registers::cr1::bits::opm::states::E,
          tim::registers::cr1::bits::arpe::states::E
      >
      static INLINE void configureBasicCounter();

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
