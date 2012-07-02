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
#define _DMA1 (reinterpret_cast<dma::common::Registers*>\
    (dma::common::address::DMA1))
#define _DMA2 (reinterpret_cast<dma::common::Registers*>\
    (dma::common::address::DMA2))

#ifdef STM32F1XX
#define _DMA1_CHANNEL1 (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::address::DMA1 + dma::channel::address::CHANNEL_1))

#define _DMA1_CHANNEL2 (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::address::DMA1 + dma::channel::address::CHANNEL_2))

#define _DMA1_CHANNEL3 (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::address::DMA1 + dma::channel::address::CHANNEL_3))

#define _DMA1_CHANNEL4 (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::address::DMA1 + dma::channel::address::CHANNEL_4))

#define _DMA1_CHANNEL5 (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::address::DMA1 + dma::channel::address::CHANNEL_5))

#define _DMA1_CHANNEL6 (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::address::DMA1 + dma::channel::address::CHANNEL_6))

#define _DMA1_CHANNEL7 (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::address::DMA1 + dma::channel::address::CHANNEL_7))

#define _DMA2_CHANNEL1 (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::address::DMA2 + dma::channel::address::CHANNEL_1))

#define _DMA2_CHANNEL2 (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::address::DMA2 + dma::channel::address::CHANNEL_2))

#define _DMA2_CHANNEL3 (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::address::DMA2 + dma::channel::address::CHANNEL_3))

#define _DMA2_CHANNEL4 (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::address::DMA2 + dma::channel::address::CHANNEL_4))

#define _DMA2_CHANNEL5 (reinterpret_cast<dma::channel::Registers*>\
        (dma::common::address::DMA2 + dma::channel::address::CHANNEL_5))

#else // STM32F1XX
#define _DMA1_STREAM0 (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::address::DMA1 + dma::stream::address::STREAM_0))

#define _DMA1_STREAM1 (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::address::DMA1 + dma::stream::address::STREAM_1))

#define _DMA1_STREAM2 (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::address::DMA1 + dma::stream::address::STREAM_2))

#define _DMA1_STREAM3 (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::address::DMA1 + dma::stream::address::STREAM_3))

#define _DMA1_STREAM4 (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::address::DMA1 + dma::stream::address::STREAM_4))

#define _DMA1_STREAM5 (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::address::DMA1 + dma::stream::address::STREAM_5))

#define _DMA1_STREAM6 (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::address::DMA1 + dma::stream::address::STREAM_6))

#define _DMA1_STREAM7 (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::address::DMA1 + dma::stream::address::STREAM_7))

#define _DMA2_STREAM0 (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::address::DMA2 + dma::stream::address::STREAM_0))

#define _DMA2_STREAM1 (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::address::DMA2 + dma::stream::address::STREAM_1))

#define _DMA2_STREAM2 (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::address::DMA2 + dma::stream::address::STREAM_2))

#define _DMA2_STREAM3 (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::address::DMA2 + dma::stream::address::STREAM_3))

#define _DMA2_STREAM4 (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::address::DMA2 + dma::stream::address::STREAM_4))

#define _DMA2_STREAM5 (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::address::DMA2 + dma::stream::address::STREAM_5))

#define _DMA2_STREAM6 (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::address::DMA2 + dma::stream::address::STREAM_6))

#define _DMA2_STREAM7 (reinterpret_cast<dma::stream::Registers*>\
        (dma::common::address::DMA2 + dma::stream::address::STREAM_7))

#endif // STM32F1XX
// High-level functions
namespace dma {
#ifdef STM32F1XX
  namespace channel {
    template<dma::common::address::E, dma::channel::address::E>
    class Functions {
      public:
      static INLINE void enable();
      static INLINE void disable();
      static INLINE void setNumberOfTransactions(u16 const);
      static INLINE void setPeripheralAddress(volatile void const*);
      static INLINE void setPeripheralAddress(void const*);
      static INLINE void setMemoryAddress(void const*);
      static INLINE void clearGlobalFlag();
      static INLINE void clearTransferCompleteFlag();
      static INLINE void clearHalfTransferFlag();
      static INLINE void clearTransferErrorFlag();

      template<
      dma::channel::registers::cr::bits::tcie::states::E,
      dma::channel::registers::cr::bits::htie::states::E,
      dma::channel::registers::cr::bits::teie::states::E,
      dma::channel::registers::cr::bits::dir::states::E,
      dma::channel::registers::cr::bits::circ::states::E,
      dma::channel::registers::cr::bits::pinc::states::E,
      dma::channel::registers::cr::bits::minc::states::E,
      dma::channel::registers::cr::bits::psize::states::E,
      dma::channel::registers::cr::bits::msize::states::E,
      dma::channel::registers::cr::bits::pl::states::E,
      dma::channel::registers::cr::bits::mem2mem::states::E
      > static INLINE void configure();
      private:
      Functions();
    };
  }  // namespace channel

#else
  namespace stream {
    template<common::address::E D, address::E S>
    class Functions {
      public:
        static INLINE void enable();
        static INLINE void disable();
        static INLINE void setNumberOfTransactions(u16 const);
        static INLINE void setPeripheralAddress(volatile void*);
        static INLINE void setPeripheralAddress(void*);
        static INLINE void setMemory0Address(void*);
        static INLINE void setMemory1Address(void*);
        static INLINE void clearFifoErrorFlag();
        static INLINE void clearDirectModeErrorFlag();
        static INLINE void clearTransferErrorFlag();
        static INLINE void clearHalfTransferFlag();
        static INLINE void clearTransferCompleteFlag();

        template<
            dma::stream::registers::cr::bits::dmeie::states::E,
            dma::stream::registers::cr::bits::teie::states::E,
            dma::stream::registers::cr::bits::htie::states::E,
            dma::stream::registers::cr::bits::tcie::states::E,
            dma::stream::registers::cr::bits::pfctrl::states::E,
            dma::stream::registers::cr::bits::dir::states::E,
            dma::stream::registers::cr::bits::circ::states::E,
            dma::stream::registers::cr::bits::pinc::states::E,
            dma::stream::registers::cr::bits::minc::states::E,
            dma::stream::registers::cr::bits::psize::states::E,
            dma::stream::registers::cr::bits::msize::states::E,
            dma::stream::registers::cr::bits::pincos::states::E,
            dma::stream::registers::cr::bits::pl::states::E,
            dma::stream::registers::cr::bits::dbm::states::E,
            dma::stream::registers::cr::bits::ct::states::E,
            dma::stream::registers::cr::bits::pburst::states::E,
            dma::stream::registers::cr::bits::mburst::states::E,
            dma::stream::registers::cr::bits::chsel::states::E
        >
        static INLINE void configure();

        template<
            dma::stream::registers::fcr::bits::fth::states::E,
            dma::stream::registers::fcr::bits::dmdis::states::E,
            dma::stream::registers::fcr::bits::feie::states::E
        >
        static INLINE void configureFIFO();

      private:
        Functions();

    };
  }// namespace stream
#endif
}  // namespace dma

// High-level access to the peripheral
#ifdef STM32F1XX
  typedef dma::channel::Functions<
  dma::common::address::DMA1,
  dma::channel::address::CHANNEL_1
  > DMA1_CHANNEL1;

  typedef dma::channel::Functions<
  dma::common::address::DMA1,
  dma::channel::address::CHANNEL_2
  > DMA1_CHANNEL2;

  typedef dma::channel::Functions<
  dma::common::address::DMA1,
  dma::channel::address::CHANNEL_3
  > DMA1_CHANNEL3;

  typedef dma::channel::Functions<
  dma::common::address::DMA1,
  dma::channel::address::CHANNEL_4
  > DMA1_CHANNEL4;

  typedef dma::channel::Functions<
  dma::common::address::DMA1,
  dma::channel::address::CHANNEL_5
  > DMA1_CHANNEL5;

  typedef dma::channel::Functions<
  dma::common::address::DMA1,
  dma::channel::address::CHANNEL_6
  > DMA1_CHANNEL6;

  typedef dma::channel::Functions<
  dma::common::address::DMA1,
  dma::channel::address::CHANNEL_7
  > DMA1_CHANNEL7;

  typedef dma::channel::Functions<
  dma::common::address::DMA2,
  dma::channel::address::CHANNEL_1
  > DMA2_CHANNEL1;

  typedef dma::channel::Functions<
  dma::common::address::DMA2,
  dma::channel::address::CHANNEL_2
  > DMA2_CHANNEL2;

  typedef dma::channel::Functions<
  dma::common::address::DMA2,
  dma::channel::address::CHANNEL_3
  > DMA2_CHANNEL3;

  typedef dma::channel::Functions<
  dma::common::address::DMA2,
  dma::channel::address::CHANNEL_4
  > DMA2_CHANNEL4;

  typedef dma::channel::Functions<
  dma::common::address::DMA2,
  dma::channel::address::CHANNEL_5
  > DMA2_CHANNEL5;

#else // STM32F1XX
  typedef dma::stream::Functions<
      dma::common::address::DMA1,
      dma::stream::address::STREAM_0
  > DMA1_STREAM0;

  typedef dma::stream::Functions<
      dma::common::address::DMA1,
      dma::stream::address::STREAM_1
  > DMA1_STREAM1;

  typedef dma::stream::Functions<
      dma::common::address::DMA1,
      dma::stream::address::STREAM_2
  > DMA1_STREAM2;

  typedef dma::stream::Functions<
      dma::common::address::DMA1,
      dma::stream::address::STREAM_3
  > DMA1_STREAM3;

  typedef dma::stream::Functions<
      dma::common::address::DMA1,
      dma::stream::address::STREAM_4
  > DMA1_STREAM4;

  typedef dma::stream::Functions<
      dma::common::address::DMA1,
      dma::stream::address::STREAM_5
  > DMA1_STREAM5;

  typedef dma::stream::Functions<
      dma::common::address::DMA1,
      dma::stream::address::STREAM_6
  > DMA1_STREAM6;

  typedef dma::stream::Functions<
      dma::common::address::DMA1,
      dma::stream::address::STREAM_7
  > DMA1_STREAM7;

  typedef dma::stream::Functions<
      dma::common::address::DMA2,
      dma::stream::address::STREAM_0
  > DMA2_STREAM0;

  typedef dma::stream::Functions<
      dma::common::address::DMA2,
      dma::stream::address::STREAM_1
  > DMA2_STREAM1;

  typedef dma::stream::Functions<
      dma::common::address::DMA2,
      dma::stream::address::STREAM_2
  > DMA2_STREAM2;

  typedef dma::stream::Functions<
      dma::common::address::DMA2,
      dma::stream::address::STREAM_3
  > DMA2_STREAM3;

  typedef dma::stream::Functions<
      dma::common::address::DMA2,
      dma::stream::address::STREAM_4
  > DMA2_STREAM4;

  typedef dma::stream::Functions<
      dma::common::address::DMA2,
      dma::stream::address::STREAM_5
  > DMA2_STREAM5;

  typedef dma::stream::Functions<
      dma::common::address::DMA2,
      dma::stream::address::STREAM_6
  > DMA2_STREAM6;

  typedef dma::stream::Functions<
      dma::common::address::DMA2,
      dma::stream::address::STREAM_7
  > DMA2_STREAM7;
#endif // STM32F1XX

#include "../../bits/dma.tcc"
