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
 *                          Digital Camera Interface
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"
#include "../defs.hpp"

#ifndef STM32F1XX

#include "../../memorymap/dcmi.hpp"

// Low-level access to the registers
#define _DCMI reinterpret_cast<dcmi::Registers *>(dcmi::ADDRESS)

// High-level functions
namespace dcmi {
  namespace format {
    enum E {
      MONOCHROME = 1,
      YCBCR422 = 2,
      RGB565 = 2
    };
  }  // namespace format

  class Functions {
    public:
      static inline void enableClock();
      static inline void disableClock();
      static inline void enablePeripheral();
      static inline void disablePeripheral();
      static inline void startCapture();
      static inline void stopCapture();
      static inline bool isInLineSynchronization();
      static inline bool isInFrameSynchronization();
      static inline bool hasBufferOverrunOccurred();
      static inline bool hasErrorSynchronizationOccurred();
      static inline void enableCaptureCompleInterrupt();
      static inline void disableCaptureCompleteInterrupt();
      static inline void clearCaptureCompleteFlag();
      static inline void enableBufferOverrunInterrupt();
      static inline void disableBufferOverrunInterrupt();
      static inline void clearBufferOverrunFlag();
      static inline void enableSynchronizationErrorInterrupt();
      static inline void disableSynchronizationErrorInterrupt();
      static inline void clearSynchronizationErrorFlag();
      static inline void enableVerticalSynchronizationInterrupt();
      static inline void disableVerticalSynchronizationInterrupt();
      static inline void clearVerticalSynchronizationFlag();
      static inline void enableLineReceivedInterrupt();
      static inline void disableLineReceivedInterrupt();
      static inline void clearLineReceivedFlag();
      static inline void clearAllFlags();

      template<
          dcmi::registers::cr::bits::capture::states::E,
          dcmi::registers::cr::bits::cm::states::E,
          dcmi::registers::cr::bits::crop::states::E,
          dcmi::registers::cr::bits::jpeg::states::E,
          dcmi::registers::cr::bits::ess::states::E,
          dcmi::registers::cr::bits::pckpol::states::E,
          dcmi::registers::cr::bits::hspol::states::E,
          dcmi::registers::cr::bits::vspol::states::E,
          dcmi::registers::cr::bits::fcrc::states::E,
          dcmi::registers::cr::bits::edm::states::E,
          dcmi::registers::cr::bits::enable::states::E
      >
      static inline void configure();

      template<
          u32, /* Left */
          u32, /* Top */
          u32, /* Width */
          u32 /* Height */,
          format::E
      >
      static inline void setCropDimensions();

    private:
      Functions();
  };
}  // namespace dcmi

#include "../../bits/dcmi.tcc"

// High-level access to the peripheral
typedef dcmi::Functions DCMI;

#endif
