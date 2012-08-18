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

#include "../include/peripheral/rcc.hpp"

namespace spi {
  template<address::E S>
  void Functions<S>::enableClock()
  {
    switch (S) {
      case address::SPI1:
        RCC::enableClocks<rcc::apb2enr::SPI1>();
        break;
      case address::SPI2:
        RCC::enableClocks<rcc::apb1enr::SPI2>();
        break;
      case address::SPI3:
        RCC::enableClocks<rcc::apb1enr::SPI3>();
        break;
    }
  }

  template<address::E S>
  void Functions<S>::disableClock()
  {
    switch (S) {
      case address::SPI1:
        RCC::disableClocks<rcc::apb2enr::SPI1>();
        break;
      case address::SPI2:
        RCC::disableClocks<rcc::apb1enr::SPI2>();
        break;
      case address::SPI3:
        RCC::disableClocks<rcc::apb1enr::SPI3>();
        break;
    }
  }

  template<address::E S>
  void Functions<S>::sendByte(const u8 byte)
  {
    reinterpret_cast<Registers*>(S)->DR = byte;
  }

  template<address::E S>
  u8 Functions<S>::getByte()
  {
    return reinterpret_cast<Registers*>(S)->DR;
  }

  template<address::E S>
  void Functions<S>::sendWord(const u16 word)
  {
    reinterpret_cast<Registers*>(S)->DR = word;
  }

  template<address::E S>
  u16 Functions<S>::getWord()
  {
    return reinterpret_cast<Registers*>(S)->DR;
  }

  template<address::E S>
  void Functions<S>::enable()
  {
    *(volatile u32*) (bitband::peripheral<
        S + registers::cr1::OFFSET,
        registers::cr1::bits::spe::POSITION
    >()) = 1;
  }

  template<address::E S>
  void Functions<S>::disable()
  {
    *(volatile u32*) (bitband::peripheral<
        S + registers::cr1::OFFSET,
        registers::cr1::bits::spe::POSITION
    >()) = 0;
  }

  template<address::E S>
  bool Functions<S>::candSendData()
  {
    return *(volatile bool*) (bitband::peripheral<
        S + registers::sr::OFFSET,
        registers::sr::bits::txe::POSITION
    >());
  }

  template<address::E S>
  bool Functions<S>::hasReceivedData()
  {
    return *(volatile bool*) (bitband::peripheral<
        S + registers::sr::OFFSET,
        registers::sr::bits::rxne::POSITION
    >());
  }

  /**
   * @note  The peripheral must be turned off during the configuration.
   */
  template<address::E S>
  template<
      registers::cr1::bits::cpha::states::E CPHA,
      registers::cr1::bits::cpol::states::E CPOL,
      registers::cr1::bits::msrt::states::E MSRT,
      registers::cr1::bits::br::states::E BR,
      registers::cr1::bits::lsbfirst::states::E LSBFIRST,
      registers::cr1::bits::ssm::states::E SSM,
      registers::cr1::bits::rxonly::states::E RXONLY,
      registers::cr1::bits::dff::states::E DFF,
      registers::cr1::bits::crcnext::states::E CRCNEXT,
      registers::cr1::bits::crcen::states::E CRCEN,
      registers::cr1::bits::bidioe::states::E BIDIOE,
      registers::cr1::bits::bidimode::states::E BIDIMODE,
      registers::cr2::bits::errie::states::E ERRIE,
      registers::cr2::bits::frf::states::E FRF,
      registers::cr2::bits::rxdmaen::states::E RXDMAEN,
      registers::cr2::bits::rxneie::states::E RXNEIE,
      registers::cr2::bits::ssoe::states::E SSOE,
      registers::cr2::bits::txdmaen::states::E TXDMAEN,
      registers::cr2::bits::txeie::states::E TXEIE
  >
  void Functions<S>::configure()
  {
    reinterpret_cast<Registers*>(S)->CR1 = CPHA + CPOL + MSRT + BR +
        LSBFIRST + SSM + RXONLY + DFF + CRCNEXT + CRCEN + BIDIOE + BIDIMODE;

    reinterpret_cast<Registers*>(S)->CR2 = ERRIE + FRF + RXDMAEN + RXNEIE +
        SSOE + TXDMAEN + TXEIE;
  }

}  // namespace spi
