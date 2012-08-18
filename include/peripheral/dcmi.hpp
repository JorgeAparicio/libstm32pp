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

#ifndef STM32F1XX

#include "../defs.hpp"
#include "../../memorymap/dcmi.hpp"

// Low-level access to the registers
#define DCMI_REGS reinterpret_cast<dcmi::Registers *>(dcmi::ADDRESS)

// High-level functions
namespace dcmi {
  enum Format {
    MONOCHROME = 1,
    YCBCR422 = 2,
    RGB565 = 2
  };

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

      static inline void configure(
          dcmi::cr::capture::States,
          dcmi::cr::cm::States,
          dcmi::cr::crop::States,
          dcmi::cr::jpeg::States,
          dcmi::cr::ess::States,
          dcmi::cr::pckpol::States,
          dcmi::cr::hspol::States,
          dcmi::cr::vspol::States,
          dcmi::cr::fcrc::States,
          dcmi::cr::edm::States,
          dcmi::cr::enable::States);

      template<
          u32,
          u32,
          u32,
          u32,
          Format
      >
      static inline void setCropDimensions();

    private:
      Functions();
  };
}  // namespace dcmi

// High-level access to the peripheral
typedef dcmi::Functions DCMI;

#include "../../bits/dcmi.tcc"

#endif
