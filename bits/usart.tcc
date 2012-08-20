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

namespace usart {
  /**
   * @brief Enables the USART's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  template<Address A>
  void Asynchronous<A>::enableClock()
  {
    switch (A) {
      case USART1:
        RCC::enableClocks<rcc::apb2enr::USART1>();
        break;
      case USART2:
        RCC::enableClocks<rcc::apb1enr::USART2>();
        break;
      case USART3:
        RCC::enableClocks<rcc::apb1enr::USART3>();
        break;
      case UART4:
        RCC::enableClocks<rcc::apb1enr::UART4>();
        break;
      case UART5:
        RCC::enableClocks<rcc::apb1enr::UART5>();
        break;
#ifndef STM32F1XX
      case USART6:
        RCC::enableClocks<rcc::apb2enr::USART6>();
        break;
#endif // STM32F1XX
    }
  }

  /**
   * @brief Disables the USART's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  template<Address A>
  void Asynchronous<A>::disableClock()
  {
    switch (A) {
      case USART1:
        RCC::disableClocks<rcc::apb2enr::USART1>();
        break;
      case USART2:
        RCC::disableClocks<rcc::apb1enr::USART2>();
        break;
      case USART3:
        RCC::disableClocks<rcc::apb1enr::USART3>();
        break;
      case UART4:
        RCC::disableClocks<rcc::apb1enr::UART4>();
        break;
      case UART5:
        RCC::disableClocks<rcc::apb1enr::UART5>();
        break;
#ifndef STM32F1XX
      case USART6:
        RCC::disableClocks<rcc::apb2enr::USART6>();
        break;
#endif // STM32F1XX
    }
  }

  /**
   * @brief Sends data through the UART.
   */
  template<Address A>
  void Asynchronous<A>::sendData(u8 const data)
  {
    reinterpret_cast<Registers*>(A)->DR = data;
  }

  /**
   * @brief Gets data from the receiver buffer.
   */
  template<Address A>
  u8 Asynchronous<A>::getData()
  {
    return reinterpret_cast<Registers*>(A)->DR;
  }

  /**
   * @brief Returns true if new data can be send.
   */
  template<Address A>
  bool Asynchronous<A>::canSendDataYet()
  {
    return *(volatile bool*) (bitband::peripheral<
        A + sr::OFFSET,
        sr::txe::POSITION>());
  }

  /**
   * @brief Returns true if there is new data available.
   */
  template<Address A>
  bool Asynchronous<A>::isThereDataAvailable()
  {
    return *(volatile bool*) (bitband::peripheral<
        A + sr::OFFSET,
        sr::rxne::POSITION>());
  }

  /**
   * @brief Sets the baud rate.
   * @note  Only valid for OVERSAMPLING_BY_16 configuration
   */
  template<Address A>
  template<u32 BAUD_RATE>
  void Asynchronous<A>::setBaudRate()
  {
    enum {
      _BRR = FREQUENCY / BAUD_RATE
    };

    static_assert(_BRR < 65536,
        "This baud rate can't be achieved with the current APB clock.");

    reinterpret_cast<Registers*>(A)->BRR = _BRR;
  }

  /**
   * @brief Configures the USART for asynchronous operation.
   * @note  Overrides the old configuration.
   */
  template<Address A>
  void Asynchronous<A>::configure(
      cr1::rwu::States RWU,
      cr1::re::States RE,
      cr1::te::States TE,
      cr1::idleie::States IDLEIE,
      cr1::rxneie::States RXNEIE,
      cr1::tcie::States TCIE,
      cr1::txeie::States TXEIE,
      cr1::peie::States PEIE,
      cr1::ps::States PS,
      cr1::pce::States PCE,
      cr1::wake::States WAKE,
      cr1::m::States M,
      cr1::ue::States UE,
      cr1::over8::States OVER8,
      cr2::stop::States STOP,
      cr3::eie::States EIE,
      cr3::hdsel::States HDSEL,
      cr3::dmar::States DMAR,
      cr3::dmat::States DMAT,
      cr3::rtse::States RSTE,
      cr3::ctse::States CTSE,
      cr3::ctsie::States CTSIE,
      cr3::onebit::States ONEBIT)
  {
    reinterpret_cast<Registers*>(A)->CR1 =
        RWU + RE + TE + IDLEIE + RXNEIE + TCIE + TXEIE + PEIE + PS +
            PCE + WAKE + M + UE + OVER8;

    reinterpret_cast<Registers*>(A)->CR2 = STOP;

    reinterpret_cast<Registers*>(A)->CR3 =
        EIE + HDSEL + DMAR + DMAT + RSTE + CTSE + CTSIE + ONEBIT;
  }

}  // namespace usart

