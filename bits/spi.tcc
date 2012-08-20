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
  template<Address S>
  void Functions<S>::enableClock()
  {
    switch (S) {
      case SPI1:
        RCC::enableClocks<rcc::apb2enr::SPI1>();
        break;
      case SPI2:
        RCC::enableClocks<rcc::apb1enr::SPI2>();
        break;
      case SPI3:
        RCC::enableClocks<rcc::apb1enr::SPI3>();
        break;
    }
  }

  template<Address S>
  void Functions<S>::disableClock()
  {
    switch (S) {
      case SPI1:
        RCC::disableClocks<rcc::apb2enr::SPI1>();
        break;
      case SPI2:
        RCC::disableClocks<rcc::apb1enr::SPI2>();
        break;
      case SPI3:
        RCC::disableClocks<rcc::apb1enr::SPI3>();
        break;
    }
  }

  template<Address S>
  void Functions<S>::sendByte(const u8 byte)
  {
    reinterpret_cast<Registers*>(S)->DR = byte;
  }

  template<Address S>
  u8 Functions<S>::getByte()
  {
    return reinterpret_cast<Registers*>(S)->DR;
  }

  template<Address S>
  void Functions<S>::sendWord(const u16 word)
  {
    reinterpret_cast<Registers*>(S)->DR = word;
  }

  template<Address S>
  u16 Functions<S>::getWord()
  {
    return reinterpret_cast<Registers*>(S)->DR;
  }

  template<Address S>
  void Functions<S>::enable()
  {
    *(volatile u32*) (bitband::peripheral<
        S + cr1::OFFSET,
        cr1::spe::POSITION
    >()) = 1;
  }

  template<Address S>
  void Functions<S>::disable()
  {
    *(volatile u32*) (bitband::peripheral<
        S + cr1::OFFSET,
        cr1::spe::POSITION
    >()) = 0;
  }

  template<Address S>
  bool Functions<S>::candSendData()
  {
    return *(volatile bool*) (bitband::peripheral<
        S + sr::OFFSET,
        sr::txe::POSITION
    >());
  }

  template<Address S>
  bool Functions<S>::hasReceivedData()
  {
    return *(volatile bool*) (bitband::peripheral<
        S + sr::OFFSET,
        sr::rxne::POSITION
    >());
  }

  /**
   * @note  The peripheral must be turned off during the configuration.
   */
  template<Address S>
  void Functions<S>::configure(
      cr1::cpha::States CPHA,
      cr1::cpol::States CPOL,
      cr1::msrt::States MSRT,
      cr1::br::States BR,
      cr1::lsbfirst::States LSBFIRST,
      cr1::ssm::States SSM,
      cr1::rxonly::States RXONLY,
      cr1::dff::States DFF,
      cr1::crcnext::States CRCNEXT,
      cr1::crcen::States CRCEN,
      cr1::bidioe::States BIDIOE,
      cr1::bidimode::States BIDIMODE,
      cr2::errie::States ERRIE,
      cr2::frf::States FRF,
      cr2::rxdmaen::States RXDMAEN,
      cr2::rxneie::States RXNEIE,
      cr2::ssoe::States SSOE,
      cr2::txdmaen::States TXDMAEN,
      cr2::txeie::States TXEIE)
  {
    reinterpret_cast<Registers*>(S)->CR1 = CPHA + CPOL + MSRT + BR +
        LSBFIRST + SSM + RXONLY + DFF + CRCNEXT + CRCEN + BIDIOE + BIDIMODE;

    reinterpret_cast<Registers*>(S)->CR2 = ERRIE + FRF + RXDMAEN + RXNEIE +
        SSOE + TXDMAEN + TXEIE;
  }

}  // namespace spi
