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
 *                       Direct Memory Access Controller
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"
#include "../defs.hpp"

#include "../../memorymap/dma.hpp"

// Low-level access to the registers
#define DMA1_COMMON_REGS (reinterpret_cast<dma::common::Registers*>\
    (dma::common::DMA1))
#define DMA2_COMMON_REGS (reinterpret_cast<dma::common::Registers*>\
    (dma::common::DMA2))

#ifdef STM32F1XX
#define DMA1_CHANNEL1_REGS (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::DMA1 + dma::channel::CHANNEL_1))

#define DMA1_CHANNEL2_REGS (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::DMA1 + dma::channel::CHANNEL_2))

#define DMA1_CHANNEL3_REGS (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::DMA1 + dma::channel::CHANNEL_3))

#define DMA1_CHANNEL4_REGS (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::DMA1 + dma::channel::CHANNEL_4))

#define DMA1_CHANNEL5_REGS (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::DMA1 + dma::channel::CHANNEL_5))

#define DMA1_CHANNEL6_REGS (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::DMA1 + dma::channel::CHANNEL_6))

#define DMA1_CHANNEL7_REGS (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::DMA1 + dma::channel::CHANNEL_7))

#define DMA2_CHANNEL1_REGS (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::DMA2 + dma::channel::CHANNEL_1))

#define DMA2_CHANNEL2_REGS (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::DMA2 + dma::channel::CHANNEL_2))

#define DMA2_CHANNEL3_REGS (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::DMA2 + dma::channel::CHANNEL_3))

#define DMA2_CHANNEL4_REGS (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::DMA2 + dma::channel::CHANNEL_4))

#define DMA2_CHANNEL5_REGS (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::DMA2 + dma::channel::CHANNEL_5))

#else // STM32F1XX
#define DMA1_STREAM0_REGS (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::DMA1 + dma::stream::STREAM_0))

#define DMA1_STREAM1_REGS (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::DMA1 + dma::stream::STREAM_1))

#define DMA1_STREAM2_REGS (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::DMA1 + dma::stream::STREAM_2))

#define DMA1_STREAM3_REGS (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::DMA1 + dma::stream::STREAM_3))

#define DMA1_STREAM4_REGS (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::DMA1 + dma::stream::STREAM_4))

#define DMA1_STREAM5_REGS (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::DMA1 + dma::stream::STREAM_5))

#define DMA1_STREAM6_REGS (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::DMA1 + dma::stream::STREAM_6))

#define DMA1_STREAM7_REGS (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::DMA1 + dma::stream::STREAM_7))

#define DMA2_STREAM0_REGS (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::DMA2 + dma::stream::STREAM_0))

#define DMA2_STREAM1_REGS (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::DMA2 + dma::stream::STREAM_1))

#define DMA2_STREAM2_REGS (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::DMA2 + dma::stream::STREAM_2))

#define DMA2_STREAM3_REGS (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::DMA2 + dma::stream::STREAM_3))

#define DMA2_STREAM4_REGS (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::DMA2 + dma::stream::STREAM_4))

#define DMA2_STREAM5_REGS (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::DMA2 + dma::stream::STREAM_5))

#define DMA2_STREAM6_REGS (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::DMA2 + dma::stream::STREAM_6))

#define DMA2_STREAM7_REGS (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::DMA2 + dma::stream::STREAM_7))

#endif // STM32F1XX
// High-level functions
namespace dma {
  namespace common {
    template<dma::common::Address>
    class Functions {
      public:
        static inline void enableClock();
        static inline void disableClock();

      private:
        Functions();
    };
  }  // namespace common

#ifdef STM32F1XX
  namespace channel {
    template<dma::common::Address, dma::channel::Address>
    class Functions {
      public:
      static inline void enableClock();
      static inline void enablePeripheral();
      static inline void disablePeripheral();
      static inline void enableGlobalInterrupts();
      static inline void disableGlobalInterrupts();
      static inline void setNumberOfTransactions(u16 const);
      static inline void setPeripheralAddress(void volatile* const);
      static inline void setPeripheralAddress(void* const);
      static inline void setMemoryAddress(void* const);
      static inline void clearGlobalFlag();
      static inline void clearTransferCompleteFlag();
      static inline void clearHalfTransferFlag();
      static inline void clearTransferErrorFlag();

      static inline void configure(
          dma::channel::cr::tcie::States,
          dma::channel::cr::htie::States,
          dma::channel::cr::teie::States,
          dma::channel::cr::dir::States,
          dma::channel::cr::circ::States,
          dma::channel::cr::pinc::States,
          dma::channel::cr::minc::States,
          dma::channel::cr::psize::States,
          dma::channel::cr::msize::States,
          dma::channel::cr::pl::States,
          dma::channel::cr::mem2mem::States);
      private:
      Functions();
    };
  }  // namespace channel

#else
  namespace stream {
    template<common::Address D, Address S>
    class Functions {
      public:
        static inline void enableClock();
        static inline void enablePeripheral();
        static inline void disablePeripheral();
        static inline void enableGlobalInterrupts();
        static inline void disableGlobalInterrupts();
        static inline bool isEnabled();
        static inline void setNumberOfTransactions(u16 const);
        static inline void setPeripheralAddress(void volatile* const);
        static inline void setPeripheralAddress(void* const);
        static inline void setMemory0Address(void* const);
        static inline void setMemory1Address(void* const);
        static inline void clearFifoErrorFlag();
        static inline bool hasFifoErrorOccurred();
        static inline void clearDirectModeErrorFlag();
        static inline bool hasDirectModeErrorOccurred();
        static inline void clearTransferErrorFlag();
        static inline bool hasTransferErrorOccurred();
        static inline void clearHalfTransferFlag();
        static inline bool hasHalfTransferOccurred();
        static inline void clearTransferCompleteFlag();
        static inline bool hasTransferCompleteOccurred();
        static inline bool isMemory1TheCurrentTarget();

        static inline void configure(
            dma::stream::cr::dmeie::States,
            dma::stream::cr::teie::States,
            dma::stream::cr::htie::States,
            dma::stream::cr::tcie::States,
            dma::stream::cr::pfctrl::States,
            dma::stream::cr::dir::States,
            dma::stream::cr::circ::States,
            dma::stream::cr::pinc::States,
            dma::stream::cr::minc::States,
            dma::stream::cr::psize::States,
            dma::stream::cr::msize::States,
            dma::stream::cr::pincos::States,
            dma::stream::cr::pl::States,
            dma::stream::cr::dbm::States,
            dma::stream::cr::ct::States,
            dma::stream::cr::pburst::States,
            dma::stream::cr::mburst::States,
            dma::stream::cr::chsel::States);

        static inline void configureFIFO(
            dma::stream::fcr::fth::States,
            dma::stream::fcr::dmdis::States,
            dma::stream::fcr::feie::States);

      private:
        Functions();

    };
  }  // namespace stream
#endif
}  // namespace dma

// High-level access to the peripheral
typedef dma::common::Functions<dma::common::DMA1> DMA1;
typedef dma::common::Functions<dma::common::DMA2> DMA2;

#ifdef STM32F1XX
typedef dma::channel::Functions<
dma::common::DMA1,
dma::channel::CHANNEL_1
> DMA1_CHANNEL1;

typedef dma::channel::Functions<
dma::common::DMA1,
dma::channel::CHANNEL_2
> DMA1_CHANNEL2;

typedef dma::channel::Functions<
dma::common::DMA1,
dma::channel::CHANNEL_3
> DMA1_CHANNEL3;

typedef dma::channel::Functions<
dma::common::DMA1,
dma::channel::CHANNEL_4
> DMA1_CHANNEL4;

typedef dma::channel::Functions<
dma::common::DMA1,
dma::channel::CHANNEL_5
> DMA1_CHANNEL5;

typedef dma::channel::Functions<
dma::common::DMA1,
dma::channel::CHANNEL_6
> DMA1_CHANNEL6;

typedef dma::channel::Functions<
dma::common::DMA1,
dma::channel::CHANNEL_7
> DMA1_CHANNEL7;

typedef dma::channel::Functions<
dma::common::DMA2,
dma::channel::CHANNEL_1
> DMA2_CHANNEL1;

typedef dma::channel::Functions<
dma::common::DMA2,
dma::channel::CHANNEL_2
> DMA2_CHANNEL2;

typedef dma::channel::Functions<
dma::common::DMA2,
dma::channel::CHANNEL_3
> DMA2_CHANNEL3;

typedef dma::channel::Functions<
dma::common::DMA2,
dma::channel::CHANNEL_4
> DMA2_CHANNEL4;

typedef dma::channel::Functions<
dma::common::DMA2,
dma::channel::CHANNEL_5
> DMA2_CHANNEL5;

#else // STM32F1XX
typedef dma::stream::Functions<
    dma::common::DMA1,
    dma::stream::STREAM_0
> DMA1_STREAM0;

typedef dma::stream::Functions<
    dma::common::DMA1,
    dma::stream::STREAM_1
> DMA1_STREAM1;

typedef dma::stream::Functions<
    dma::common::DMA1,
    dma::stream::STREAM_2
> DMA1_STREAM2;

typedef dma::stream::Functions<
    dma::common::DMA1,
    dma::stream::STREAM_3
> DMA1_STREAM3;

typedef dma::stream::Functions<
    dma::common::DMA1,
    dma::stream::STREAM_4
> DMA1_STREAM4;

typedef dma::stream::Functions<
    dma::common::DMA1,
    dma::stream::STREAM_5
> DMA1_STREAM5;

typedef dma::stream::Functions<
    dma::common::DMA1,
    dma::stream::STREAM_6
> DMA1_STREAM6;

typedef dma::stream::Functions<
    dma::common::DMA1,
    dma::stream::STREAM_7
> DMA1_STREAM7;

typedef dma::stream::Functions<
    dma::common::DMA2,
    dma::stream::STREAM_0
> DMA2_STREAM0;

typedef dma::stream::Functions<
    dma::common::DMA2,
    dma::stream::STREAM_1
> DMA2_STREAM1;

typedef dma::stream::Functions<
    dma::common::DMA2,
    dma::stream::STREAM_2
> DMA2_STREAM2;

typedef dma::stream::Functions<
    dma::common::DMA2,
    dma::stream::STREAM_3
> DMA2_STREAM3;

typedef dma::stream::Functions<
    dma::common::DMA2,
    dma::stream::STREAM_4
> DMA2_STREAM4;

typedef dma::stream::Functions<
    dma::common::DMA2,
    dma::stream::STREAM_5
> DMA2_STREAM5;

typedef dma::stream::Functions<
    dma::common::DMA2,
    dma::stream::STREAM_6
> DMA2_STREAM6;

typedef dma::stream::Functions<
    dma::common::DMA2,
    dma::stream::STREAM_7
> DMA2_STREAM7;
#endif // STM32F1XX
#include "../../bits/dma.tcc"
