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

namespace dma {
  namespace common {
    /**
     * @brief Enables the DMA's clock.
     * @note  Registers can't be written when the clock is disabled.
     */
    template<address::E D>
    void Functions<D>::enableClock()
    {
      RCC::enableClocks<
          D == address::DMA1 ?
              rcc::registers::ahb1enr::bits::DMA1 :
              (D == address::DMA2 ?
                                    rcc::registers::ahb1enr::bits::DMA2 :
                                    0)
      >();
    }

    /**
     * @brief Disables the DMA's clock.
     * @note  Registers can't be written when the clock is disabled.
     */
    template<address::E D>
    void Functions<D>::disableClock()
    {
      RCC::disableClocks<
          D == address::DMA1 ?
              rcc::registers::ahb1enr::bits::DMA1 :
              (D == address::DMA2 ?
                                    rcc::registers::ahb1enr::bits::DMA2 :
                                    0)
      >();
    }

  }  // namespace common

#ifdef STM32F1XX
  namespace channel {
    /**
     * @brief Enables the DMA's clock.
     * @note  Registers can't be written when the clock is disabled.
     */
    template<dma::common::address::E D, address::E C>
    void Functions<D, C>::enableClock()
    {
      RCC::enableClocks<
          D == common::address::DMA1 ?
              rcc::registers::ahb1enr::bits::DMA1 :
              (D == common::address::DMA2 ?
                                    rcc::registers::ahb1enr::bits::DMA2 :
                                    0)
      >();
    }

    /**
     * @brief Enables the DMA channel.
     */
    template<dma::common::address::E D, address::E C>
    void Functions<D, C>::enablePeripheral()
    {
      *(u32*) (bitband::Peripheral<
          D + C + registers::cr::OFFSET,
          registers::cr::bits::en::POSITION
          >::address) = 1;
    }

    /**
     * @brief Disables the DMA channel.
     */
    template<dma::common::address::E D, address::E C>
    void Functions<D, C>::disablePeripheral()
    {
      *(u32*) (bitband::Peripheral<
          D + C + registers::cr::OFFSET,
          registers::cr::bits::en::POSITION
          >::address) = 0;
    }

    /**
     * @brief Specifies the amount of data to be transfered.
     */
    template<dma::common::address::E D, address::E C>
    void Functions<D, C>::setNumberOfTransactions(u16 const N)
    {
      reinterpret_cast<Registers*>(D + C)->CNDTR = N;
    }

    /**
     * @brief Specifies the target peripheral memory address.
     */
    template<dma::common::address::E D, address::E C>
    void Functions<D, C>::setPeripheralAddress(volatile void const* address)
    {
      reinterpret_cast<Registers*>(D + C)->CPAR = u32(address);
    }

    /**
     * @brief Specifies the target peripheral memory address.
     */
    template<dma::common::address::E D, address::E C>
    void Functions<D, C>::setPeripheralAddress(void const* address)
    {
      reinterpret_cast<Registers*>(D + C)->CPAR = u32(address);
    }

    /**
     * @brief Specifies the target memory address.
     */
    template<dma::common::address::E D, address::E C>
    void Functions<D, C>::setMemoryAddress(void const* address)
    {
      reinterpret_cast<Registers*>(D + C)->CMAR = u32(address);
    }

    /**
     * @brief Clears all the interrupt flags.
     */
    template<dma::common::address::E D, address::E C>
    void Functions<D, C>::clearGlobalFlag()
    {
      enum {
        Channel = (C - 8) / 20
      };

      // TODO DMA, replace the hard-coded numbers
      *(u32*) (bitband::Peripheral<
          D + C + dma::common::registers::ifcr::OFFSET,
          4 * Channel>::address) = 1;
    }

    /**
     * @brief Clears the transfer complete interrupt flag.
     */
    template<dma::common::address::E D, address::E C>
    void Functions<D, C>::clearTransferCompleteFlag()
    {
      // TODO DMA, replace the hard-coded numbers
      *(u32*) (bitband::Peripheral<
          D + C + dma::common::registers::ifcr::OFFSET,
          4 * Channel + 1>::address) = 1;
    }

    /**
     * @brief Clears the half transfer interrupt flag.
     */
    template<dma::common::address::E D, address::E C>
    void Functions<D, C>::clearHalfTransferFlag()
    {
      // TODO DMA, replace the hard-coded numbers
      *(u32*) (bitband::Peripheral<
          D + C + dma::common::registers::ifcr::OFFSET,
          4 * Channel + 2>::address) = 1;
    }

    /**
     * @brief Clears the transfer error interrupt flag.
     */
    template<dma::common::address::E D, address::E C>
    void Functions<D, C>::clearTransferErrorFlag()
    {
      // TODO DMA, replace the hard-coded numbers
      *(u32*) (bitband::Peripheral<
          D + C + dma::common::registers::ifcr::OFFSET,
          4 * Channel + 3>::address) = 1;
    }

    /**
     * @brief Configures the DMA peripheral.
     * @note  The channel must be disabled before the configuration.
     * @note  Overrides the old configuration.
     */
    template<dma::common::address::E D, address::E C>
    template<
    registers::cr::bits::tcie::states::E TCIE,
    registers::cr::bits::htie::states::E HTIE,
    registers::cr::bits::teie::states::E TEIE,
    registers::cr::bits::dir::states::E DIR,
    registers::cr::bits::circ::states::E CIRC,
    registers::cr::bits::pinc::states::E PINC,
    registers::cr::bits::minc::states::E MINC,
    registers::cr::bits::psize::states::E PSIZE,
    registers::cr::bits::msize::states::E MSIZE,
    registers::cr::bits::pl::states::E PL,
    registers::cr::bits::mem2mem::states::E MEM2MEM
    > void Functions<D, C>::configure()
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
    template<dma::common::address::E D, address::E S>
    void Functions<D, S>::enableClock()
    {
      RCC::enableClocks<
          D == common::address::DMA1 ?
              rcc::registers::ahb1enr::bits::DMA1 :
              (D == common::address::DMA2 ?
                                    rcc::registers::ahb1enr::bits::DMA2 :
                                    0)
      >();
    }

    /**
     * @brief Enables the DMA stream.
     */
    template<dma::common::address::E D, address::E S>
    void Functions<D, S>::enablePeripheral()
    {
      *(u32*) (bitband::Peripheral<
          D + S + registers::cr::OFFSET,
          registers::cr::bits::en::POSITION
      >::address) = 1;
    }

    /**
     * @brief Disables the DMA stream.
     */
    template<dma::common::address::E D, address::E S>
    void Functions<D, S>::disablePeripheral()
    {
      *(u32*) (bitband::Peripheral<
          D + S + registers::cr::OFFSET,
          registers::cr::bits::en::POSITION
      >::address) = 0;
    }

    /**
     * @brief Returns true if the the DMA stream is enabled.
     */
    template<dma::common::address::E D, address::E S>
    bool Functions<D, S>::isEnabled()
    {
      // FIXME DMA *(*bool) cast generates Hard Fault exception.
      return *(u32*) (bitband::Peripheral<
          D + S + registers::cr::OFFSET,
          registers::cr::bits::en::POSITION
      >::address);
    }

    /**
     * @brief Specifies the amount of data to be transfered.
     */
    template<dma::common::address::E D, address::E S>
    void Functions<D, S>::setNumberOfTransactions(u16 const N)
    {
      reinterpret_cast<Registers*>(D + S)->NDTR = N;
    }

    /**
     * @brief Specifies the target peripheral memory address.
     */
    template<dma::common::address::E D, address::E S>
    void Functions<D, S>::setPeripheralAddress(volatile void* address)
    {
      reinterpret_cast<Registers*>(D + S)->PAR = u32(address);
    }

    /**
     * @brief Specifies the target peripheral memory address.
     */
    template<dma::common::address::E D, address::E S>
    void Functions<D, S>::setPeripheralAddress(void* address)
    {
      reinterpret_cast<Registers*>(D + S)->PAR = u32(address);
    }

    /**
     * @brief Specifies the target memory address.
     */
    template<dma::common::address::E D, address::E S>
    void Functions<D, S>::setMemory0Address(void* address)
    {
      reinterpret_cast<Registers*>(D + S)->M0AR = u32(address);
    }

    /**
     * @brief Specifies the alternate target memory address.
     */
    template<dma::common::address::E D, address::E S>
    void Functions<D, S>::setMemory1Address(void* address)
    {
      reinterpret_cast<Registers*>(D + S)->M1AR = u32(address);
    }

    /**
     * @brief Clears the fifo error interrupt flag.
     */
    template<dma::common::address::E D, address::E S>
    void Functions<D, S>::clearFifoErrorFlag()
    {
      enum {
        Stream = (S - 0x10) / 0x18
      };

      *(u32*) (bitband::Peripheral<
          (Stream > 3 ?
                        D + dma::common::registers::hifcr::OFFSET :
                        D + dma::common::registers::lifcr::OFFSET),
          (Stream % 4 == 0 ?
                             0 :
                             (Stream % 4 == 1 ?
                                                6 :
                                                (Stream % 4 == 2 ?
                                                                   16 :
                                                                   22)))
      >::address) = 1;
    }

    /**
     * @brief Clears the fifo error interrupt flag.
     */
    template<dma::common::address::E D, address::E S>
    bool Functions<D, S>::hasFifoErrorOccurred()
    {
      // FIXME DMA *(*bool) cast generates Hard Fault exception.

      enum {
        Stream = (S - 0x10) / 0x18
      };

      return *(u32*) (bitband::Peripheral<
          (Stream > 3 ?
                        D + dma::common::registers::hisr::OFFSET :
                        D + dma::common::registers::lisr::OFFSET),
          (Stream % 4 == 0 ?
                             0 :
                             (Stream % 4 == 1 ?
                                                6 :
                                                (Stream % 4 == 2 ?
                                                                   16 :
                                                                   22)))
      >::address);
    }

    /**
     * @brief Clears the direct mode error interrupt flag.
     */
    template<dma::common::address::E D, address::E S>
    void Functions<D, S>::clearDirectModeErrorFlag()
    {
      enum {
        Stream = (S - 0x10) / 0x18
      };

      *(u32*) (bitband::Peripheral<
          (Stream > 3 ?
                        D + dma::common::registers::hifcr::OFFSET :
                        D + dma::common::registers::lifcr::OFFSET),
          (Stream % 4 == 0 ?
                             2 :
                             (Stream % 4 == 1 ?
                                                8 :
                                                (Stream % 4 == 2 ?
                                                                   18 :
                                                                   24)))
      >::address) = 1;
    }

    /**
     * @brief Returns true if a direct mode error has occurred.
     */
    template<dma::common::address::E D, address::E S>
    bool Functions<D, S>::hasDirectModeErrorOccurred()
    {
      // FIXME DMA *(*bool) cast generates Hard Fault exception.

      enum {
        Stream = (S - 0x10) / 0x18
      };

      return *(u32*) (bitband::Peripheral<
          (Stream > 3 ?
                        D + dma::common::registers::hisr::OFFSET :
                        D + dma::common::registers::lisr::OFFSET),
          (Stream % 4 == 0 ?
                             2 :
                             (Stream % 4 == 1 ?
                                                8 :
                                                (Stream % 4 == 2 ?
                                                                   18 :
                                                                   24)))
      >::address);
    }

    /**
     * @brief Clears the transfer error interrupt flag.
     */
    template<dma::common::address::E D, address::E S>
    void Functions<D, S>::clearTransferErrorFlag()
    {
      enum {
        Stream = (S - 0x10) / 0x18
      };

      *(u32*) (bitband::Peripheral<
          (Stream > 3 ?
                        D + dma::common::registers::hifcr::OFFSET :
                        D + dma::common::registers::lifcr::OFFSET),
          (Stream % 4 == 0 ?
                             3 :
                             (Stream % 4 == 1 ?
                                                9 :
                                                (Stream % 4 == 2 ?
                                                                   19 :
                                                                   25)))
      >::address) = 1;
    }

    /**
     * @brief Returns true if half of the transfer has occurred.
     */
    template<dma::common::address::E D, address::E S>
    bool Functions<D, S>::hasTransferErrorOccurred()
    {
      // FIXME DMA *(*bool) cast generates Hard Fault exception.

      enum {
        Stream = (S - 0x10) / 0x18
      };

      return *(u32*) (bitband::Peripheral<
          (Stream > 3 ?
                        D + dma::common::registers::hisr::OFFSET :
                        D + dma::common::registers::lisr::OFFSET),
          (Stream % 4 == 0 ?
                             3 :
                             (Stream % 4 == 1 ?
                                                9 :
                                                (Stream % 4 == 2 ?
                                                                   19 :
                                                                   25)))
      >::address);
    }

    /**
     * @brief Clears the half transfer interrupt flag.
     */
    template<dma::common::address::E D, address::E S>
    void Functions<D, S>::clearHalfTransferFlag()
    {
      enum {
        Stream = (S - 0x10) / 0x18
      };

      *(u32*) (bitband::Peripheral<
          (Stream > 3 ?
                        D + dma::common::registers::hifcr::OFFSET :
                        D + dma::common::registers::lifcr::OFFSET),
          (Stream % 4 == 0 ?
                             4 :
                             (Stream % 4 == 1 ?
                                                10 :
                                                (Stream % 4 == 2 ?
                                                                   20 :
                                                                   26)))
      >::address) = 1;
    }

    /**
     * @brief Returns true if half of the transfer has occurred.
     */
    template<dma::common::address::E D, address::E S>
    bool Functions<D, S>::hasHalfTransferOccurred()
    {
      // FIXME DMA *(*bool) cast generates Hard Fault exception.

      enum {
        Stream = (S - 0x10) / 0x18
      };

      return *(u32*) (bitband::Peripheral<
          (Stream > 3 ?
                        D + dma::common::registers::hisr::OFFSET :
                        D + dma::common::registers::lisr::OFFSET),
          (Stream % 4 == 0 ?
                             4 :
                             (Stream % 4 == 1 ?
                                                10 :
                                                (Stream % 4 == 2 ?
                                                                   20 :
                                                                   26)))
      >::address);
    }

    /**
     * @brief Clears the transfer complete interrupt flag.
     */
    template<dma::common::address::E D, address::E S>
    void Functions<D, S>::clearTransferCompleteFlag()
    {
      enum {
        Stream = (S - 0x10) / 0x18
      };

      *(u32*) (bitband::Peripheral<
          (Stream > 3 ?
                        D + dma::common::registers::hifcr::OFFSET :
                        D + dma::common::registers::lifcr::OFFSET),
          (Stream % 4 == 0 ?
                             5 :
                             (Stream % 4 == 1 ?
                                                11 :
                                                (Stream % 4 == 2 ?
                                                                   21 :
                                                                   27)))
      >::address) = 1;
    }

    /**
     * @brief Returns true if a complete transfer has occurred.
     */
    template<dma::common::address::E D, address::E S>
    bool Functions<D, S>::hasTransferCompleteOccurred()
    {
      // FIXME DMA *(*bool) cast generates Hard Fault exception.

      enum {
        Stream = (S - 0x10) / 0x18
      };

      return *(u32*) (bitband::Peripheral<
          (Stream > 3 ?
                        D + dma::common::registers::hisr::OFFSET :
                        D + dma::common::registers::lisr::OFFSET),
          (Stream % 4 == 0 ?
                             5 :
                             (Stream % 4 == 1 ?
                                                11 :
                                                (Stream % 4 == 2 ?
                                                                   21 :
                                                                   27)))
      >::address);
    }

    /**
     * @brief Returns true if memory 1 is the current target of the DMA.
     */
    template<dma::common::address::E D, address::E S>
    bool Functions<D, S>::isMemory1TheCurrentTarget()
    {
      // FIXME DMA *(*bool) cast generates Hard Fault exception.

      return *(u32 *) (bitband::Peripheral<
          D + S + registers::cr::OFFSET,
          registers::cr::bits::ct::POSITION
      >::address);
    }

    /**
     * @brief Configures the stream operation.
     * @note  The stream must be disabled before the setup.
     * @note  Overrides the old configuration.
     */
    template<dma::common::address::E D, address::E S>
    template<
        registers::cr::bits::dmeie::states::E DMEIE,
        registers::cr::bits::teie::states::E TEIE,
        registers::cr::bits::htie::states::E HTIE,
        registers::cr::bits::tcie::states::E TCIE,
        registers::cr::bits::pfctrl::states::E PFCTRL,
        registers::cr::bits::dir::states::E DIR,
        registers::cr::bits::circ::states::E CIRC,
        registers::cr::bits::pinc::states::E PINC,
        registers::cr::bits::minc::states::E MINC,
        registers::cr::bits::psize::states::E PSIZE,
        registers::cr::bits::msize::states::E MSIZE,
        registers::cr::bits::pincos::states::E PINCOS,
        registers::cr::bits::pl::states::E PL,
        registers::cr::bits::dbm::states::E DBM,
        registers::cr::bits::ct::states::E CT,
        registers::cr::bits::pburst::states::E PBURST,
        registers::cr::bits::mburst::states::E MBURST,
        registers::cr::bits::chsel::states::E CHSEL
    >
    void Functions<D, S>::configure()
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
    template<dma::common::address::E D, stream::address::E S>
    template<
        registers::fcr::bits::fth::states::E FTH,
        registers::fcr::bits::dmdis::states::E DMDIS,
        registers::fcr::bits::feie::states::E FEIE
    >
    void Functions<D, S>::configureFIFO()
    {
      reinterpret_cast<stream::Registers*>(D + S)->FCR = FTH + DMDIS + FEIE;
    }
  }  // namespace stream
#endif

}  // namespace dma
