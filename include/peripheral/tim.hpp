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

#include "../../memorymap/tim.hpp"

// Low-level access to the registers
#define _TIM6   ((tim::basic::Registers*) tim::basic::address::E::TIM6)
#define _TIM7   ((tim::basic::Registers*) tim::basic::address::E::TIM7)

#define _TIM2   ((tim::generalPurpose1::Registers*)\
    tim::generalPurpose1::address::E::TIM2)
#define _TIM3   ((tim::generalPurpose1::Registers*)\
    tim::generalPurpose1::address::E::TIM3)
#define _TIM4   ((tim::generalPurpose1::Registers*)\
    tim::generalPurpose1::address::E::TIM4)
#define _TIM5   ((tim::generalPurpose1::Registers*)\
    tim::generalPurpose1::address::E::TIM5)

#define _TIM9   ((tim::generalPurpose2::Registers*)\
    tim::::generalPurpose2::address::E::TIM9)
#define _TIM12  ((tim::generalPurpose2::Registers*)\
    tim::generalPurpose2::address::E::TIM12)

#define _TIM10  ((tim::generalPurpose3::Registers*)\
    tim::generalPurpose3::address::E::TIM10)
#define _TIM11  ((tim::generalPurpose3::Registers*)\
    tim::generalPurpose3::address::E::TIM11)
#define _TIM13  ((tim::generalPurpose3::Registers*)\
    tim::generalPurpose3::address::E::TIM13)
#define _TIM14  ((tim::generalPurpose3::Registers*)\
    tim::generalPurpose3::address::E::TIM14)

#ifdef VALUE_LINE
#define _TIM15  ((tim::generalPurpose4::Registers*)\
    tim::generalPurpose4::address::E::TIM15)
#define _TIM16  ((tim::generalPurpose4::Registers*)\
    tim::generalPurpose4::address::E::TIM16)
#define _TIM17  ((tim::generalPurpose4::Registers*)\
    tim::generalPurpose4::address::E::TIM17)
#endif

#define _TIM1   ((tim::advancedControl::Registers*)\
    tim::advancedControl::address::E::TIM1)
#define _TIM8   ((tim::advancedControl::Registers*)\
    tim::advancedControl::address::E::TIM8)

// High-level functions
namespace tim {
  namespace basic {
    template<address::E>
    class Functions {
      public:
        static INLINE void startCounter();
        static INLINE void stopCounter();
        static INLINE void setPrescaler(u16 const);
        static INLINE void setAutoReload(u16 const);
        static INLINE void setCounter(u16 const);
        static INLINE u16 getCounter();
        static INLINE void generateUpdate();
        static INLINE void enableInterrupt();
        static INLINE void disableInterrupt();
        static INLINE void clearFlag();
        static INLINE void enableDMA();
        static INLINE void disableDMA();

        template<
            tim::registers::cr1::bits::cen::states::E,
            tim::registers::cr1::bits::udis::states::E,
            tim::registers::cr1::bits::urs::states::E,
            tim::registers::cr1::bits::opm::states::E,
            tim::registers::cr1::bits::arpe::states::E
        >
        static INLINE void configureCounter();

        template<
            tim::registers::cr2::bits::mms::states::E
        >
        static INLINE void setMasterMode();

      private:
        Functions();
    };
  }  // namespace basic

  namespace generalPurpose1 {
    template<address::E>
    class Functions {
      public:
        static INLINE void startCounter();
        static INLINE void stopCounter();
        static INLINE void setPrescaler(u16 const);
        static INLINE void setAutoReload(u16 const);
        static INLINE void setCounter(u16 const);
        static INLINE u16 getCounter();
        static INLINE void generateUpdate();
        static INLINE void enableInterrupt();
        static INLINE void disableInterrupt();
        static INLINE void clearFlag();
        static INLINE void enableDMA();
        static INLINE void disableDMA();

        template<
            tim::registers::cr1::bits::cen::states::E,
            tim::registers::cr1::bits::udis::states::E,
            tim::registers::cr1::bits::urs::states::E,
            tim::registers::cr1::bits::opm::states::E,
            tim::registers::cr1::bits::arpe::states::E
        >
        static INLINE void configureCounter();

        template<
            tim::registers::cr2::bits::mms::states::E
        >
        static INLINE void setMasterMode();

        // TODO TIM General-Purpose 1 function declarations

      private:
        Functions();
    };
  }  // namespace generalPurpose1

  namespace generalPurpose2 {
    template<address::E>
    class Functions {
      public:
        static INLINE void startCounter();
        static INLINE void stopCounter();
        static INLINE void setPrescaler(u16 const);
        static INLINE void setAutoReload(u16 const);
        static INLINE void setCounter(u16 const);
        static INLINE u16 getCounter();
        static INLINE void generateUpdate();
        static INLINE void enableInterrupt();
        static INLINE void disableInterrupt();
        static INLINE void clearFlag();
        static INLINE void enableDMA();
        static INLINE void disableDMA();

        template<
            tim::registers::cr1::bits::cen::states::E,
            tim::registers::cr1::bits::udis::states::E,
            tim::registers::cr1::bits::urs::states::E,
            tim::registers::cr1::bits::opm::states::E,
            tim::registers::cr1::bits::arpe::states::E
        >
        static INLINE void configureCounter();

        template<
            tim::registers::cr2::bits::mms::states::E
        >
        static INLINE void setMasterMode();

        // TODO TIM General-Purpose 2 function declarations

      private:
        Functions();
    };
  }  // namespace generalPurpose2

  namespace generalPurpose3 {
    template<address::E>
    class Functions {
      public:
        static INLINE void startCounter();
        static INLINE void stopCounter();
        static INLINE void setPrescaler(u16 const);
        static INLINE void setAutoReload(u16 const);
        static INLINE void setCounter(u16 const);
        static INLINE u16 getCounter();
        static INLINE void generateUpdate();
        static INLINE void enableInterrupt();
        static INLINE void disableInterrupt();
        static INLINE void clearFlag();
        static INLINE void enableDMA();
        static INLINE void disableDMA();

        template<
            tim::registers::cr1::bits::cen::states::E,
            tim::registers::cr1::bits::udis::states::E,
            tim::registers::cr1::bits::urs::states::E,
            tim::registers::cr1::bits::opm::states::E,
            tim::registers::cr1::bits::arpe::states::E
        >
        static INLINE void configureCounter();

        template<
            tim::registers::cr2::bits::mms::states::E
        >
        static INLINE void setMasterMode();

        // TODO TIM General-Purpose 3 function declarations

      private:
        Functions();
    };
  }  // namespace generalPurpose3

#ifdef VALUE_LINE
  namespace generalPurpose4 {
    template<address::E>
    class Functions {
      public:
        static INLINE void startCounter();
        static INLINE void stopCounter();
        static INLINE void setPrescaler(u16 const);
        static INLINE void setAutoReload(u16 const);
        static INLINE void setCounter(u16 const);
        static INLINE u16 getCounter();
        static INLINE void generateUpdate();
        static INLINE void enableInterrupt();
        static INLINE void disableInterrupt();
        static INLINE void clearFlag();
        static INLINE void enableDMA();
        static INLINE void disableDMA();

        template<
            tim::registers::cr1::bits::cen::states::E,
            tim::registers::cr1::bits::udis::states::E,
            tim::registers::cr1::bits::urs::states::E,
            tim::registers::cr1::bits::opm::states::E,
            tim::registers::cr1::bits::arpe::states::E
        >
        static INLINE void configureCounter();

        template<
            tim::registers::cr2::bits::mms::states::E
        >
        static INLINE void setMasterMode();

        // TODO TIM General-Purpose 4 function declarations

      private:
        Functions();
    };
  }  // namespace generalPurpose4
#endif

  namespace advancedControl {
    template<address::E>
    class Functions {
      public:
        static INLINE void startCounter();
        static INLINE void stopCounter();
        static INLINE void setPrescaler(u16 const);
        static INLINE void setAutoReload(u16 const);
        static INLINE void setCounter(u16 const);
        static INLINE u16 getCounter();
        static INLINE void generateUpdate();
        static INLINE void enableInterrupt();
        static INLINE void disableInterrupt();
        static INLINE void clearFlag();
        static INLINE void enableDMA();
        static INLINE void disableDMA();

        template<
            tim::registers::cr1::bits::cen::states::E,
            tim::registers::cr1::bits::udis::states::E,
            tim::registers::cr1::bits::urs::states::E,
            tim::registers::cr1::bits::opm::states::E,
            tim::registers::cr1::bits::arpe::states::E
        >
        static INLINE void configureCounter();

        template<
            tim::registers::cr2::bits::mms::states::E
        >
        static INLINE void setMasterMode();

        // TODO TIM Advanced-Control function declarations

      private:
        Functions();
    };
  }  // namespace advancedControl

}  // namespace tim

// High-level access to the peripheral
typedef tim::basic::Functions<
    tim::basic::address::TIM6
> TIM6;
typedef tim::basic::Functions<
    tim::basic::address::TIM7
> TIM7;

typedef tim::generalPurpose1::Functions<
    tim::generalPurpose1::address::TIM2
> TIM2;
typedef tim::generalPurpose1::Functions<
    tim::generalPurpose1::address::TIM3
> TIM3;
typedef tim::generalPurpose1::Functions<
    tim::generalPurpose1::address::TIM4
> TIM4;
typedef tim::generalPurpose1::Functions<
    tim::generalPurpose1::address::TIM5
> TIM5;

typedef tim::generalPurpose2::Functions<
    tim::generalPurpose2::address::TIM9
> TIM9;
typedef tim::generalPurpose2::Functions<
    tim::generalPurpose2::address::TIM12
> TIM12;

typedef tim::generalPurpose3::Functions<
    tim::generalPurpose3::address::TIM10
> TIM10;

typedef tim::generalPurpose3::Functions<
    tim::generalPurpose3::address::TIM11
> TIM11;

typedef tim::generalPurpose3::Functions<
    tim::generalPurpose3::address::TIM13
> TIM13;

typedef tim::generalPurpose3::Functions<
    tim::generalPurpose3::address::TIM14
> TIM14;

#ifdef VALUE_LINE
typedef tim::generalPurpose4::Functions<
    tim::generalPurpose4::address::TIM15
> TIM15;

typedef tim::generalPurpose4::Functions<
    tim::generalPurpose4::address::TIM16
> TIM16;

typedef tim::generalPurpose4::Functions<
    tim::generalPurpose4::address::TIM17
> TIM17;
#endif

typedef tim::advancedControl::Functions<
    tim::advancedControl::address::TIM1
> TIM1;

typedef tim::advancedControl::Functions<
    tim::advancedControl::address::TIM8
> TIM8;

#include "../../bits/tim.tcc"
