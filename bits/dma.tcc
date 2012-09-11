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

#include "bitband.hpp"
#include "../include/peripheral/rcc.hpp"
#include "../include/core/nvic.hpp"

namespace dma {
  namespace common {
    /**
     * @brief Enables the DMA's clock.
     * @note  Registers can't be written when the clock is disabled.
     */
    template<Address D>
    void Functions<D>::enableClock()
    {
      switch (D) {
        case DMA1:
          #ifndef STM32F1XX
          RCC::enableClocks<rcc::ahb1enr::DMA1>();
#else // STM32F1XX
          RCC::enableClocks<rcc::ahbenr::DMA1>();
#endif // STM32F1XX
          break;
        case DMA2:
          #ifndef STM32F1XX
          RCC::enableClocks<rcc::ahb1enr::DMA2>();
#else // STM32F1XX
          RCC::enableClocks<rcc::ahbenr::DMA2>();
#endif // STM32F1XX
          break;
      }
    }

    /**
     * @brief Disables the DMA's clock.
     * @note  Registers can't be written when the clock is disabled.
     */
    template<Address D>
    void Functions<D>::disableClock()
    {
      switch (D) {
        case DMA1:
          #ifndef STM32F1XX
          RCC::disableClocks<rcc::ahb1enr::DMA1>();
#else // STM32F1XX
          RCC::disableClocks<rcc::ahbenr::DMA1>();
#endif // STM32F1XX
          break;
        case DMA2:
          #ifndef STM32F1XX
          RCC::disableClocks<rcc::ahb1enr::DMA2>();
#else // STM32F1XX
          RCC::disableClocks<rcc::ahbenr::DMA2>();
#endif // STM32F1XX
          break;
      }
    }

  }  // namespace common

#ifdef STM32F1XX
  namespace channel {
    /**
     * @brief Enables the DMA's clock.
     * @note  Registers can't be written when the clock is disabled.
     */
    template<dma::common::Address D, Address C>
    void Functions<D, C>::enableClock()
    {
      switch (D) {
        case dma::common::DMA1:
          RCC::enableClocks<rcc::ahbenr::DMA1>();
          break;
        case dma::common::DMA2:
          RCC::enableClocks<rcc::ahbenr::DMA2>();
          break;
      }
    }

    /**
     * @brief Enables the DMA channel.
     */
    template<dma::common::Address D, Address C>
    void Functions<D, C>::enablePeripheral()
    {
      *(u32 volatile*) (bitband::peripheral<
          D + C + cr::OFFSET,
          cr::en::POSITION
      >()) = 1;
    }

    /**
     * @brief Disables the DMA channel.
     */
    template<dma::common::Address D, Address C>
    void Functions<D, C>::disablePeripheral()
    {
      *(u32 volatile*) (bitband::peripheral<
          D + C + cr::OFFSET,
          cr::en::POSITION
      >()) = 0;
    }

    /**
     * @brief Enables all the DMA stream interrupts.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::enableGlobalInterrupts()
    {
      switch (D) {
        case common::DMA1:
          switch (S) {
            case CHANNEL_1:
              NVIC::enableInterrupt<
                  nvic::irqn::DMA1_Channel1
              >();
              break;
            case CHANNEL_2:
              NVIC::enableInterrupt<
                  nvic::irqn::DMA1_Channel2
              >();
              break;
            case CHANNEL_3:
              NVIC::enableInterrupt<
                  nvic::irqn::DMA1_Channel3
              >();
              break;
            case CHANNEL_4:
              NVIC::enableInterrupt<
                  nvic::irqn::DMA1_Channel4
              >();
              break;
            case CHANNEL_5:
              NVIC::enableInterrupt<
                  nvic::irqn::DMA1_Channel5
              >();
              break;
            case CHANNEL_6:
              NVIC::enableInterrupt<
                  nvic::irqn::DMA1_Channel6
              >();
              break;
            case CHANNEL_7:
              NVIC::enableInterrupt<
                  nvic::irqn::DMA1_Channel7
              >();
              break;
          }
          break;
        case common::DMA2:
          switch (S) {
            case CHANNEL_1:
              NVIC::enableInterrupt<
                  nvic::irqn::DMA2_Channel1
              >();
              break;
            case CHANNEL_2:
              NVIC::enableInterrupt<
                  nvic::irqn::DMA2_Channel2
              >();
              break;
            case CHANNEL_3:
              NVIC::enableInterrupt<
                  nvic::irqn::DMA2_Channel3
              >();
              break;
#ifdef CONNECTIVITY_LINE
            case CHANNEL_4:
              NVIC::enableInterrupt<
                  nvic::irqn::DMA2_Channel4
              >();
              break;
            case CHANNEL_5:
              NVIC::enableInterrupt<
                  nvic::irqn::DMA2_Channel5
              >();
              break;
#else // CONNECTIVITY_LINE
              case CHANNEL_4:
              NVIC::enableInterrupt<
              nvic::irqn::DMA2_Channel4_5
              >();
              break;
              case CHANNEL_5:
              NVIC::enableInterrupt<
              nvic::irqn::DMA2_Channel4_5
              >();
              break;
#endif // CONNECTIVITY_LINE
          }
          break;
      }
    }

    /**
     * @brief Disables all the DMA stream interrupts.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::disableGlobalInterrupts()
    {

      switch (D) {
        case common::DMA1:
          switch (S) {
            case CHANNEL_1:
              NVIC::disableInterrupt<
                  nvic::irqn::DMA1_Channel1
              >();
              break;
            case CHANNEL_2:
              NVIC::disableInterrupt<
                  nvic::irqn::DMA1_Channel2
              >();
              break;
            case CHANNEL_3:
              NVIC::disableInterrupt<
                  nvic::irqn::DMA1_Channel3
              >();
              break;
            case CHANNEL_4:
              NVIC::disableInterrupt<
                  nvic::irqn::DMA1_Channel4
              >();
              break;
            case CHANNEL_5:
              NVIC::disableInterrupt<
                  nvic::irqn::DMA1_Channel5
              >();
              break;
            case CHANNEL_6:
              NVIC::disableInterrupt<
                  nvic::irqn::DMA1_Channel6
              >();
              break;
            case CHANNEL_7:
              NVIC::disableInterrupt<
                  nvic::irqn::DMA1_Channel7
              >();
              break;
          }
          break;
        case common::DMA2:
          switch (S) {
            case CHANNEL_1:
              NVIC::disableInterrupt<
                  nvic::irqn::DMA2_Channel1
              >();
              break;
            case CHANNEL_2:
              NVIC::disableInterrupt<
                  nvic::irqn::DMA2_Channel2
              >();
              break;
            case CHANNEL_3:
              NVIC::disableInterrupt<
                  nvic::irqn::DMA2_Channel3
              >();
              break;
#ifdef CONNECTIVITY_LINE
            case CHANNEL_4:
              NVIC::disableInterrupt<
                  nvic::irqn::DMA2_Channel4
              >();
              break;
            case CHANNEL_5:
              NVIC::disableInterrupt<
                  nvic::irqn::DMA2_Channel5
              >();
              break;
#else // CONNECTIVITY_LINE
              case CHANNEL_4:
              static_assert(S != CHANNEL_4,
                  "Don't use this function, "
                  "use NVIC::disableInterrupts() instead.");
              break;
              case CHANNEL_5:
              static_assert(S != CHANNEL_5,
                  "Don't use this function, "
                  "use NVIC::disableInterrupts() instead.");
              break;
#endif // CONNECTIVITY_LINE
          }
          break;
      }
    }

    /**
     * @brief Specifies the amount of data to be transfered.
     */
    template<dma::common::Address D, Address C>
    void Functions<D, C>::setNumberOfTransactions(u16 const N)
    {
      reinterpret_cast<Registers*>(D + C)->CNDTR = N;
    }

    /**
     * @brief Specifies the target peripheral memory address.
     */
    template<dma::common::Address D, Address C>
    void Functions<D, C>::setPeripheralAddress(void volatile* const address)
    {
      reinterpret_cast<Registers*>(D + C)->CPAR = u32(address);
    }

    /**
     * @brief Specifies the target peripheral memory address.
     */
    template<dma::common::Address D, Address C>
    void Functions<D, C>::setPeripheralAddress(void* const address)
    {
      reinterpret_cast<Registers*>(D + C)->CPAR = u32(address);
    }

    /**
     * @brief Specifies the target memory address.
     */
    template<dma::common::Address D, Address C>
    void Functions<D, C>::setMemoryAddress(void* const address)
    {
      reinterpret_cast<Registers*>(D + C)->CMAR = u32(address);
    }

    /**
     * @brief Clears all the interrupt flags.
     */
    template<dma::common::Address D, Address C>
    void Functions<D, C>::clearGlobalFlag()
    {
      enum {
        Channel = (C - 8) / 20
      };

      // TODO DMA, replace the hard-coded numbers
      *(u32 volatile*) (bitband::peripheral<
          D + C + dma::common::ifcr::OFFSET,
          4 * Channel>()) = 1;
    }

    /**
     * @brief Clears the transfer complete interrupt flag.
     */
    template<dma::common::Address D, Address C>
    void Functions<D, C>::clearTransferCompleteFlag()
    {
      enum {
        Channel = (C - 8) / 20
      };

      // TODO DMA, replace the hard-coded numbers
      *(u32 volatile*) (bitband::peripheral<
          D + C + dma::common::ifcr::OFFSET,
          4 * Channel + 1>()) = 1;
    }

    /**
     * @brief Clears the half transfer interrupt flag.
     */
    template<dma::common::Address D, Address C>
    void Functions<D, C>::clearHalfTransferFlag()
    {
      enum {
        Channel = (C - 8) / 20
      };

      // TODO DMA, replace the hard-coded numbers
      *(u32 volatile*) (bitband::peripheral<
          D + C + dma::common::ifcr::OFFSET,
          4 * Channel + 2>()) = 1;
    }

    /**
     * @brief Clears the transfer error interrupt flag.
     */
    template<dma::common::Address D, Address C>
    void Functions<D, C>::clearTransferErrorFlag()
    {
      enum {
        Channel = (C - 8) / 20
      };

      // TODO DMA, replace the hard-coded numbers
      *(u32 volatile*) (bitband::peripheral<
          D + C + dma::common::ifcr::OFFSET,
          4 * Channel + 3>()) = 1;
    }

    /**
     * @brief Configures the DMA peripheral.
     * @note  The channel must be disabled before the configuration.
     * @note  Overrides the old configuration.
     */
    template<dma::common::Address D, Address C>
    void Functions<D, C>::configure(
        cr::tcie::States TCIE,
        cr::htie::States HTIE,
        cr::teie::States TEIE,
        cr::dir::States DIR,
        cr::circ::States CIRC,
        cr::pinc::States PINC,
        cr::minc::States MINC,
        cr::psize::States PSIZE,
        cr::msize::States MSIZE,
        cr::pl::States PL,
        cr::mem2mem::States MEM2MEM)
    {
      reinterpret_cast<Registers*>(D + C)->CCR =
          TCIE + HTIE + TEIE + DIR + CIRC + PINC + MINC + PSIZE + MSIZE +
              PL + MEM2MEM;
    }
  }  //namespace chanel
#else
  namespace stream {
    /**
     * @brief Enables the DMA's clock.
     * @note  Registers can't be written when the clock is disabled.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::enableClock()
    {
      switch (D) {
        case dma::common::DMA1:
        RCC::enableClocks<rcc::ahb1enr::DMA1>();
        break;
        case dma::common::DMA2:
        RCC::enableClocks<rcc::ahb1enr::DMA2>();
        break;
      }
    }

    /**
     * @brief Enables the DMA stream.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::enablePeripheral()
    {
      *(u32 volatile*) (bitband::peripheral<
          D + S + cr::OFFSET,
          cr::en::POSITION
          >()) = 1;
    }

    /**
     * @brief Disables the DMA stream.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::disablePeripheral()
    {
      *(u32 volatile*) (bitband::peripheral<
          D + S + cr::OFFSET,
          cr::en::POSITION
          >()) = 0;
    }

    /**
     * @brief Enables all the DMA stream interrupts.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::enableGlobalInterrupts()
    {
      switch (D) {
        case common::DMA1:
        switch (S) {
          case STREAM_0:
          NVIC::enableInterrupt<
          nvic::irqn::DMA1_Stream0
          >();
          break;
          case STREAM_1:
          NVIC::enableInterrupt<
          nvic::irqn::DMA1_Stream1
          >();
          break;
          case STREAM_2:
          NVIC::enableInterrupt<
          nvic::irqn::DMA1_Stream2
          >();
          break;
          case STREAM_3:
          NVIC::enableInterrupt<
          nvic::irqn::DMA1_Stream3
          >();
          break;
          case STREAM_4:
          NVIC::enableInterrupt<
          nvic::irqn::DMA1_Stream4
          >();
          break;
          case STREAM_5:
          NVIC::enableInterrupt<
          nvic::irqn::DMA1_Stream5
          >();
          break;
          case STREAM_6:
          NVIC::enableInterrupt<
          nvic::irqn::DMA1_Stream6
          >();
          break;
          case STREAM_7:
          NVIC::enableInterrupt<
          nvic::irqn::DMA1_Stream7
          >();
          break;
        }
        break;
        case common::DMA2:
        switch (S) {
          case STREAM_0:
          NVIC::enableInterrupt<
          nvic::irqn::DMA2_Stream0
          >();
          break;
          case STREAM_1:
          NVIC::enableInterrupt<
          nvic::irqn::DMA2_Stream1
          >();
          break;
          case STREAM_2:
          NVIC::enableInterrupt<
          nvic::irqn::DMA2_Stream2
          >();
          break;
          case STREAM_3:
          NVIC::enableInterrupt<
          nvic::irqn::DMA2_Stream3
          >();
          break;
          case STREAM_4:
          NVIC::enableInterrupt<
          nvic::irqn::DMA2_Stream4
          >();
          break;
          case STREAM_5:
          NVIC::enableInterrupt<
          nvic::irqn::DMA2_Stream5
          >();
          break;
          case STREAM_6:
          NVIC::enableInterrupt<
          nvic::irqn::DMA2_Stream6
          >();
          break;
          case STREAM_7:
          NVIC::enableInterrupt<
          nvic::irqn::DMA2_Stream7
          >();
          break;
        }
        break;
      }
    }

    /**
     * @brief Disables all the DMA stream interrupts.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::disableGlobalInterrupts()
    {

      switch (D) {
        case common::DMA1:
        switch (S) {
          case STREAM_0:
          NVIC::disableInterrupt<
          nvic::irqn::DMA1_Stream0
          >();
          break;
          case STREAM_1:
          NVIC::disableInterrupt<
          nvic::irqn::DMA1_Stream1
          >();
          break;
          case STREAM_2:
          NVIC::disableInterrupt<
          nvic::irqn::DMA1_Stream2
          >();
          break;
          case STREAM_3:
          NVIC::disableInterrupt<
          nvic::irqn::DMA1_Stream3
          >();
          break;
          case STREAM_4:
          NVIC::disableInterrupt<
          nvic::irqn::DMA1_Stream4
          >();
          break;
          case STREAM_5:
          NVIC::disableInterrupt<
          nvic::irqn::DMA1_Stream5
          >();
          break;
          case STREAM_6:
          NVIC::disableInterrupt<
          nvic::irqn::DMA1_Stream6
          >();
          break;
          case STREAM_7:
          NVIC::disableInterrupt<
          nvic::irqn::DMA1_Stream7
          >();
          break;
        }
        break;
        case common::DMA2:
        switch (S) {
          case STREAM_0:
          NVIC::disableInterrupt<
          nvic::irqn::DMA2_Stream0
          >();
          break;
          case STREAM_1:
          NVIC::disableInterrupt<
          nvic::irqn::DMA2_Stream1
          >();
          break;
          case STREAM_2:
          NVIC::disableInterrupt<
          nvic::irqn::DMA2_Stream2
          >();
          break;
          case STREAM_3:
          NVIC::disableInterrupt<
          nvic::irqn::DMA2_Stream3
          >();
          break;
          case STREAM_4:
          NVIC::disableInterrupt<
          nvic::irqn::DMA2_Stream4
          >();
          break;
          case STREAM_5:
          NVIC::disableInterrupt<
          nvic::irqn::DMA2_Stream5
          >();
          break;
          case STREAM_6:
          NVIC::disableInterrupt<
          nvic::irqn::DMA2_Stream6
          >();
          break;
          case STREAM_7:
          NVIC::disableInterrupt<
          nvic::irqn::DMA2_Stream7
          >();
          break;
        }
        break;
      }
    }

    /**
     * @brief Returns true if the the DMA stream is enabled.
     */
    template<dma::common::Address D, Address S>
    bool Functions<D, S>::isEnabled()
    {
      // FIXME DMA *(*bool) cast generates Hard Fault exception.
      return *(u32 volatile*) (bitband::peripheral<
          D + S + cr::OFFSET,
          cr::en::POSITION
          >());
    }

    /**
     * @brief Specifies the amount of data to be transfered.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::setNumberOfTransactions(u16 const N)
    {
      reinterpret_cast<Registers*>(D + S)->NDTR = N;
    }

    /**
     * @brief Specifies the target peripheral memory address.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::setPeripheralAddress(void volatile* const address)
    {
      reinterpret_cast<Registers*>(D + S)->PAR = u32(address);
    }

    /**
     * @brief Specifies the target peripheral memory address.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::setPeripheralAddress(void* const address)
    {
      reinterpret_cast<Registers*>(D + S)->PAR = u32(address);
    }

    /**
     * @brief Specifies the target memory address.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::setMemory0Address(void* const address)
    {
      reinterpret_cast<Registers*>(D + S)->M0AR = u32(address);
    }

    /**
     * @brief Specifies the alternate target memory address.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::setMemory1Address(void* const address)
    {
      reinterpret_cast<Registers*>(D + S)->M1AR = u32(address);
    }

    /**
     * @brief Clears the fifo error interrupt flag.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::clearFifoErrorFlag()
    {
      enum {
        Stream = (S - 0x10) / 0x18
      };

      *(u32 volatile*) (bitband::peripheral<
          (Stream > 3 ?
              D + dma::common::hifcr::OFFSET :
              D + dma::common::lifcr::OFFSET),
          (Stream % 4 == 0 ?
              0 :
              (Stream % 4 == 1 ?
                  6 :
                  (Stream % 4 == 2 ?
                      16 :
                      22)))
          >()) = 1;
    }

    /**
     * @brief Clears the fifo error interrupt flag.
     */
    template<dma::common::Address D, Address S>
    bool Functions<D, S>::hasFifoErrorOccurred()
    {
      // FIXME DMA *(*bool) cast generates Hard Fault exception.

      enum {
        Stream = (S - 0x10) / 0x18
      };

      return *(u32 volatile*) (bitband::peripheral<
          (Stream > 3 ?
              D + dma::common::hisr::OFFSET :
              D + dma::common::lisr::OFFSET),
          (Stream % 4 == 0 ?
              0 :
              (Stream % 4 == 1 ?
                  6 :
                  (Stream % 4 == 2 ?
                      16 :
                      22)))
          >());
    }

    /**
     * @brief Clears the direct mode error interrupt flag.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::clearDirectModeErrorFlag()
    {
      enum {
        Stream = (S - 0x10) / 0x18
      };

      *(u32 volatile*) (bitband::peripheral<
          (Stream > 3 ?
              D + dma::common::hifcr::OFFSET :
              D + dma::common::lifcr::OFFSET),
          (Stream % 4 == 0 ?
              2 :
              (Stream % 4 == 1 ?
                  8 :
                  (Stream % 4 == 2 ?
                      18 :
                      24)))
          >()) = 1;
    }

    /**
     * @brief Returns true if a direct mode error has occurred.
     */
    template<dma::common::Address D, Address S>
    bool Functions<D, S>::hasDirectModeErrorOccurred()
    {
      // FIXME DMA *(*bool) cast generates Hard Fault exception.

      enum {
        Stream = (S - 0x10) / 0x18
      };

      return *(u32 volatile*) (bitband::peripheral<
          (Stream > 3 ?
              D + dma::common::hisr::OFFSET :
              D + dma::common::lisr::OFFSET),
          (Stream % 4 == 0 ?
              2 :
              (Stream % 4 == 1 ?
                  8 :
                  (Stream % 4 == 2 ?
                      18 :
                      24)))
          >());
    }

    /**
     * @brief Clears the transfer error interrupt flag.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::clearTransferErrorFlag()
    {
      enum {
        Stream = (S - 0x10) / 0x18
      };

      *(u32 volatile*) (bitband::peripheral<
          (Stream > 3 ?
              D + dma::common::hifcr::OFFSET :
              D + dma::common::lifcr::OFFSET),
          (Stream % 4 == 0 ?
              3 :
              (Stream % 4 == 1 ?
                  9 :
                  (Stream % 4 == 2 ?
                      19 :
                      25)))
          >()) = 1;
    }

    /**
     * @brief Returns true if half of the transfer has occurred.
     */
    template<dma::common::Address D, Address S>
    bool Functions<D, S>::hasTransferErrorOccurred()
    {
      // FIXME DMA *(*bool) cast generates Hard Fault exception.

      enum {
        Stream = (S - 0x10) / 0x18
      };

      return *(u32 volatile*) (bitband::peripheral<
          (Stream > 3 ?
              D + dma::common::hisr::OFFSET :
              D + dma::common::lisr::OFFSET),
          (Stream % 4 == 0 ?
              3 :
              (Stream % 4 == 1 ?
                  9 :
                  (Stream % 4 == 2 ?
                      19 :
                      25)))
          >());
    }

    /**
     * @brief Clears the half transfer interrupt flag.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::clearHalfTransferFlag()
    {
      enum {
        Stream = (S - 0x10) / 0x18
      };

      *(u32 volatile*) (bitband::peripheral<
          (Stream > 3 ?
              D + dma::common::hifcr::OFFSET :
              D + dma::common::lifcr::OFFSET),
          (Stream % 4 == 0 ?
              4 :
              (Stream % 4 == 1 ?
                  10 :
                  (Stream % 4 == 2 ?
                      20 :
                      26)))
          >()) = 1;
    }

    /**
     * @brief Returns true if half of the transfer has occurred.
     */
    template<dma::common::Address D, Address S>
    bool Functions<D, S>::hasHalfTransferOccurred()
    {
      // FIXME DMA *(*bool) cast generates Hard Fault exception.

      enum {
        Stream = (S - 0x10) / 0x18
      };

      return *(u32 volatile*) (bitband::peripheral<
          (Stream > 3 ?
              D + dma::common::hisr::OFFSET :
              D + dma::common::lisr::OFFSET),
          (Stream % 4 == 0 ?
              4 :
              (Stream % 4 == 1 ?
                  10 :
                  (Stream % 4 == 2 ?
                      20 :
                      26)))
          >());
    }

    /**
     * @brief Clears the transfer complete interrupt flag.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::clearTransferCompleteFlag()
    {
      enum {
        Stream = (S - 0x10) / 0x18
      };

      *(u32 volatile*) (bitband::peripheral<
          (Stream > 3 ?
              D + dma::common::hifcr::OFFSET :
              D + dma::common::lifcr::OFFSET),
          (Stream % 4 == 0 ?
              5 :
              (Stream % 4 == 1 ?
                  11 :
                  (Stream % 4 == 2 ?
                      21 :
                      27)))
          >()) = 1;
    }

    /**
     * @brief Returns true if a complete transfer has occurred.
     */
    template<dma::common::Address D, Address S>
    bool Functions<D, S>::hasTransferCompleteOccurred()
    {
      // FIXME DMA *(*bool) cast generates Hard Fault exception.

      enum {
        Stream = (S - 0x10) / 0x18
      };

      return *(u32 volatile*) (bitband::peripheral<
          (Stream > 3 ?
              D + dma::common::hisr::OFFSET :
              D + dma::common::lisr::OFFSET),
          (Stream % 4 == 0 ?
              5 :
              (Stream % 4 == 1 ?
                  11 :
                  (Stream % 4 == 2 ?
                      21 :
                      27)))
          >());
    }

    /**
     * @brief Returns true if memory 1 is the current target of the DMA.
     */
    template<dma::common::Address D, Address S>
    bool Functions<D, S>::isMemory1TheCurrentTarget()
    {
      // FIXME DMA *(*bool) cast generates Hard Fault exception.

      return *(u32 volatile*) (bitband::peripheral<
          D + S + cr::OFFSET,
          cr::ct::POSITION
          >());
    }

    /**
     * @brief Configures the stream operation.
     * @note  The stream must be disabled before the setup.
     * @note  Overrides the old configuration.
     */
    template<dma::common::Address D, Address S>
    void Functions<D, S>::configure(
        cr::dmeie::States DMEIE,
        cr::teie::States TEIE,
        cr::htie::States HTIE,
        cr::tcie::States TCIE,
        cr::pfctrl::States PFCTRL,
        cr::dir::States DIR,
        cr::circ::States CIRC,
        cr::pinc::States PINC,
        cr::minc::States MINC,
        cr::psize::States PSIZE,
        cr::msize::States MSIZE,
        cr::pincos::States PINCOS,
        cr::pl::States PL,
        cr::dbm::States DBM,
        cr::ct::States CT,
        cr::pburst::States PBURST,
        cr::mburst::States MBURST,
        cr::chsel::States CHSEL)
    {
      reinterpret_cast<stream::Registers*>(D + S)->CR =
      DMEIE + TEIE + HTIE + TCIE + PFCTRL + DIR + CIRC + PINC +
      MINC + PSIZE + MSIZE + PINCOS + PL + DBM + CT + PBURST +
      MBURST + CHSEL;
    }

    /**
     * @brief Configures the stream FIFO operation.
     * @note  Overrides the old configuration.
     */
    template<dma::common::Address D, stream::Address S>
    void Functions<D, S>::configureFIFO(
        fcr::fth::States FTH,
        fcr::dmdis::States DMDIS,
        fcr::feie::States FEIE)
    {
      reinterpret_cast<stream::Registers*>(D + S)->FCR = FTH + DMDIS + FEIE;
    }
  }  // namespace stream
#endif

}  // namespace dma
