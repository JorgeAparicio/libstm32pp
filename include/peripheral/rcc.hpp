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
      static inline void enableHse();
      static inline void disableHse();
      static inline bool isHseStable();
      static inline void enableHseOscillator();
      static inline void disableHseOscillator();

      static inline void enableLse();
      static inline void disableLse();
      static inline bool isLseStable();
      static inline void enableLseOscillator();
      static inline void disableLseOscillator();

      static inline void enableHsi();
      static inline void disableHsi();
      static inline bool isHsiStable();

      static inline void enableLsi();
      static inline void disableLsi();
      static inline bool isLsiStable();

      static inline void enableRtc();
      static inline void disableRtc();

      static inline void enablePll();
      static inline void disablePll();
      static inline bool isPllStable();

#ifdef CONNECTIVITY_LINE
      static inline void enablePll2();
      static inline void disablePll2();
      static inline bool isPll2Stable();
      static inline void enablePll3();
      static inline void disablePll3();
      static inline bool isPll3Stable();
#endif // CONNECTIVITY_LINE
      static inline bool isSystemClockSourceStable();

      template<
          rcc::cfgr::sw::States
      >
      static inline void selectSystemClockSource();

      template<
          rcc::bdcr::rtcsel::States
      >
      static inline void selectRtcClockSource();

      template<
      rcc::apb1enr::Bits...
      >
      static inline void enableClocks();

      template<
      rcc::apb1enr::Bits ...
      >
      static inline void disableClocks();

      template<
          rcc::apb1rstr::Bits...
      >
      static inline void resetPeripherals();

      template<
      rcc::apb2enr::Bits ...
      >
      static inline void enableClocks();

      template<
      rcc::apb2enr::Bits ...
      >
      static inline void disableClocks();

      template<
      rcc::apb2rstr::Bits ...
      >
      static inline void resetPeripherals();

#ifdef STM32F1XX
      template<
      rcc::ahbenr::E ...
      >
      static inline void enableClocks();

      template<
      rcc::ahbenr::E ...
      >
      static inline void disableClocks();

      template<
      rcc::cfgr::mco::States
      >
      static inline void configureClockOutput();

#ifdef CONNECTIVITY_LINE
      template<
      rcc::ahbrstr::E ...
      >
      static inline void resetPeripherals();
#endif

#ifdef VALUE_LINE
      template<
      u8 HPRE,
      u8 PPRE1,
      u8 PPRE2,
      u8 ADCPRE
      >
      static inline void configureBusPrescalers();
#else // VALUE_LINE
      template<
      u8 HPRE,
      u8 PPRE1,
      u8 PPRE2,
      u8 ADCPRE,
      u8 USBPRE
      >
      static inline void configureBusPrescalers();

#endif // VALUE_LINE
#ifdef VALUE_LINE
      template<
      rcc::cfgr::pllsrc::States,
      u8 PLLMUL,
      u8 PREDIV1
      >
      static inline void configurePll();
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
      static inline void configurePll();

#else // CONNECTIVITY_LINE
      template <
      rcc::cfgr::pllsrc::States,
      u8 PLLXTPRE,
      u8 PLLMUL
      >
      static inline void configurePll();

#endif // CONNECTIVITY_LINE
#endif // VALUE_LINE
#else // STM32F1XX
      template<
      rcc::ahb1enr::Bits ...
      >
      static inline void enableClocks();

      template<
      rcc::ahb1enr::Bits ...
      >
      static inline void disableClocks();

      template<
      rcc::ahb1rstr::Bits ...
      >
      static inline void resetPeripherals();

      template<
      rcc::ahb2enr::Bits ...
      >
      static inline void enableClocks();

      template<
      rcc::ahb2enr::Bits ...
      >
      static inline void disableClocks();

      template<
      rcc::ahb2rstr::Bits ...
      >
      static inline void resetPeripherals();

      template<
      rcc::pllcfgr::pllsrc::States,
      u8 PLLM,
      u16 PLLN,
      u8 PLLP,
      u8 PLLQ
      >
      static inline void configurePll();

      template<
      u16 PLLI2SN,
      u8 PLLI2SR
      >
      static inline void configureI2sPll();

      template<
      u8 HPRE,
      u8 PPRE1,
      u8 PPRE2,
      u8 RTCPRE
      >
      static inline void configurePrescalers();

      template<
      rcc::cfgr::mco1::States,
      rcc::cfgr::mco2::States,
      u8 MCOPRE1,
      u8 MCOPRE2
      >
      static inline void configureClockOutput();

      template<
      rcc::cfgr::i2ssrc::States
      >
      static inline void selectI2sSource();

#endif // STM32F1XX
      private:
      Functions();
    };
  }  // namespace rcc

#include "../../bits/rcc.tcc"

// High-level access to the peripheral
  typedef rcc::Functions RCC;
