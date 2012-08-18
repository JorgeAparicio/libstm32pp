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
 *                        Serial Peripheral Interface
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"
#include "../defs.hpp"

#include "../../memorymap/spi.hpp"

// Low-level access to the registers
#define SPI1_REGS reinterpret_cast<spi::Registers*>(spi::address::SPI1)
#define SPI2_REGS reinterpret_cast<spi::Registers*>(spi::address::SPI2)
#define SPI3_REGS reinterpret_cast<spi::Registers*>(spi::address::SPI3)

// High-level functions
namespace spi {
  template<address::E S>
  class Functions {
    public:
      static inline void enableClock();
      static inline void disableClock();
      static inline void sendByte(const u8);
      static inline u8 getByte();
      static inline void sendWord(const u16);
      static inline u16 getWord();
      static inline void enable();
      static inline void disable();
      static inline bool candSendData();
      static inline bool hasReceivedData();

      template<
          spi::registers::cr1::bits::cpha::states::E,
          spi::registers::cr1::bits::cpol::states::E,
          spi::registers::cr1::bits::msrt::states::E,
          spi::registers::cr1::bits::br::states::E,
          spi::registers::cr1::bits::lsbfirst::states::E,
          spi::registers::cr1::bits::ssm::states::E,
          spi::registers::cr1::bits::rxonly::states::E,
          spi::registers::cr1::bits::dff::states::E,
          spi::registers::cr1::bits::crcnext::states::E,
          spi::registers::cr1::bits::crcen::states::E,
          spi::registers::cr1::bits::bidioe::states::E,
          spi::registers::cr1::bits::bidimode::states::E,
          spi::registers::cr2::bits::errie::states::E,
          spi::registers::cr2::bits::frf::states::E,
          spi::registers::cr2::bits::rxdmaen::states::E,
          spi::registers::cr2::bits::rxneie::states::E,
          spi::registers::cr2::bits::ssoe::states::E,
          spi::registers::cr2::bits::txdmaen::states::E,
          spi::registers::cr2::bits::txeie::states::E
      >
      static inline void configure();

    private:
      Functions();
  };
}  // namespace spi

// High-level access to the peripheral
typedef spi::Functions<spi::address::SPI1> SPI1;
typedef spi::Functions<spi::address::SPI2> SPI2;
typedef spi::Functions<spi::address::SPI3> SPI3;

#include "../../bits/spi.tcc"
