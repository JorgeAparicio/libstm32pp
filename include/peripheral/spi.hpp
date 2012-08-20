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
#define SPI1_REGS reinterpret_cast<spi::Registers*>(spi::SPI1)
#define SPI2_REGS reinterpret_cast<spi::Registers*>(spi::SPI2)
#define SPI3_REGS reinterpret_cast<spi::Registers*>(spi::SPI3)

// High-level functions
namespace spi {
  template<Address S>
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
      static inline void configure(
          spi::cr1::cpha::States,
          spi::cr1::cpol::States,
          spi::cr1::msrt::States,
          spi::cr1::br::States,
          spi::cr1::lsbfirst::States,
          spi::cr1::ssm::States,
          spi::cr1::rxonly::States,
          spi::cr1::dff::States,
          spi::cr1::crcnext::States,
          spi::cr1::crcen::States,
          spi::cr1::bidioe::States,
          spi::cr1::bidimode::States,
          spi::cr2::errie::States,
          spi::cr2::frf::States,
          spi::cr2::rxdmaen::States,
          spi::cr2::rxneie::States,
          spi::cr2::ssoe::States,
          spi::cr2::txdmaen::States,
          spi::cr2::txeie::States);

    private:
      Functions();
  };
}  // namespace spi

// High-level access to the peripheral
typedef spi::Functions<spi::SPI1> SPI1;
typedef spi::Functions<spi::SPI2> SPI2;
typedef spi::Functions<spi::SPI3> SPI3;

#include "../../bits/spi.tcc"
