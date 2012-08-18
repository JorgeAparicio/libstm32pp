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
 *                         Reset and Clock Control
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"
#include "../defs.hpp"

#include "../../memorymap/rcc.hpp"

// Low-level access to the registers
#define _RCC  reinterpret_cast<rcc::Registers *>(rcc::ADDRESS)

#include "cfunctions.hpp"

// High-level functions
namespace rcc {
  class Functions {
    public:
      static INLINE void enableHse();
      static INLINE void disableHse();
      static INLINE bool isHseStable();
      static INLINE void enableHseOscillator();
      static INLINE void disableHseOscillator();

      static INLINE void enableLse();
      static INLINE void disableLse();
      static INLINE bool isLseStable();
      static INLINE void enableLseOscillator();
      static INLINE void disableLseOscillator();

      static INLINE void enableHsi();
      static INLINE void disableHsi();
      static INLINE bool isHsiStable();

      static INLINE void enableLsi();
      static INLINE void disableLsi();
      static INLINE bool isLsiStable();

      static INLINE void enableRtc();
      static INLINE void disableRtc();

      static INLINE void enablePll();
      static INLINE void disablePll();
      static INLINE bool isPllStable();

#ifdef CONNECTIVITY_LINE
      static INLINE void enablePll2();
      static INLINE void disablePll2();
      static INLINE bool isPll2Stable();
      static INLINE void enablePll3();
      static INLINE void disablePll3();
      static INLINE bool isPll3Stable();
#endif // CONNECTIVITY_LINE
      static INLINE bool isSystemClockSourceStable();

      template<
          rcc::cfgr::sw::States
      >
      static INLINE void selectSystemClockSource();

      template<
          rcc::bdcr::rtcsel::States
      >
      static INLINE void selectRtcClockSource();

      template<
      rcc::apb1enr::Bits...
      >
      static INLINE void enableClocks();

      template<
      rcc::apb1enr::Bits ...
      >
      static INLINE void disableClocks();

      template<
          rcc::apb1rstr::Bits...
      >
      static INLINE void resetPeripherals();

      template<
      rcc::apb2enr::Bits ...
      >
      static INLINE void enableClocks();

      template<
      rcc::apb2enr::Bits ...
      >
      static INLINE void disableClocks();

      template<
      rcc::apb2rstr::Bits ...
      >
      static INLINE void resetPeripherals();

#ifdef STM32F1XX
      template<
      rcc::ahbenr::E ...
      >
      static INLINE void enableClocks();

      template<
      rcc::ahbenr::E ...
      >
      static INLINE void disableClocks();

      template<
      rcc::cfgr::mco::States
      >
      static INLINE void configureClockOutput();

#ifdef CONNECTIVITY_LINE
      template<
      rcc::ahbrstr::E ...
      >
      static INLINE void resetPeripherals();
#endif

#ifdef VALUE_LINE
      template<
      u8 HPRE,
      u8 PPRE1,
      u8 PPRE2,
      u8 ADCPRE
      >
      static INLINE void configureBusPrescalers();
#else // VALUE_LINE
      template<
      u8 HPRE,
      u8 PPRE1,
      u8 PPRE2,
      u8 ADCPRE,
      u8 USBPRE
      >
      static INLINE void configureBusPrescalers();

#endif // VALUE_LINE
#ifdef VALUE_LINE
      template<
      rcc::cfgr::pllsrc::States,
      u8 PLLMUL,
      u8 PREDIV1
      >
      static INLINE void configurePll();
#else // VALUE_LINE
#ifdef CONNECTIVITY_LINE
      template<
      rcc::cfgr::pllsrc::States,
      u8 PLLMUL,
      u8 PREDIV1,
      u8 PREDIV2,
      u8 PLL2MUL,
      u8 PLL3MUL,
      rcc::cfgr2::prediv1src::States,
      rcc::cfgr2::i2s2src::States,
      rcc::cfgr2::i2s3src::States
      >
      static INLINE void configurePll();

#else // CONNECTIVITY_LINE
      template <
      rcc::cfgr::pllsrc::States,
      u8 PLLXTPRE,
      u8 PLLMUL
      >
      static INLINE void configurePll();

#endif // CONNECTIVITY_LINE
#endif // VALUE_LINE
#else // STM32F1XX
      template<
      rcc::ahb1enr::Bits ...
      >
      static INLINE void enableClocks();

      template<
      rcc::ahb1enr::Bits ...
      >
      static INLINE void disableClocks();

      template<
      rcc::ahb1rstr::Bits ...
      >
      static INLINE void resetPeripherals();

      template<
      rcc::ahb2enr::Bits ...
      >
      static INLINE void enableClocks();

      template<
      rcc::ahb2enr::Bits ...
      >
      static INLINE void disableClocks();

      template<
      rcc::ahb2rstr::Bits ...
      >
      static INLINE void resetPeripherals();

      template<
      rcc::pllcfgr::pllsrc::States,
      u8 PLLM,
      u16 PLLN,
      u8 PLLP,
      u8 PLLQ
      >
      static INLINE void configurePll();

      template<
      u16 PLLI2SN,
      u8 PLLI2SR
      >
      static INLINE void configureI2sPll();

      template<
      u8 HPRE,
      u8 PPRE1,
      u8 PPRE2,
      u8 RTCPRE
      >
      static INLINE void configurePrescalers();

      template<
      rcc::cfgr::mco1::States,
      rcc::cfgr::mco2::States,
      u8 MCOPRE1,
      u8 MCOPRE2
      >
      static INLINE void configureClockOutput();

      template<
      rcc::cfgr::i2ssrc::States
      >
      static INLINE void selectI2sSource();

#endif // STM32F1XX
      private:
      Functions();
    };
  }  // namespace rcc

#include "../../bits/rcc.tcc"

// High-level access to the peripheral
  typedef rcc::Functions RCC;
