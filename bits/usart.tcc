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
  template<address::E A>
  void Asynchronous<A>::enableClock()
  {
    RCC::enableClocks<
        A == address::USART2 ?
            rcc::apb1enr::USART2 :
            (A == address::USART3 ?
                rcc::apb1enr::USART3 :
                (A == address::UART4 ?
                    rcc::apb1enr::UART4 :
                    (A == address::UART5 ?
                                           rcc::apb1enr::UART5 :
                                           rcc::apb1enr::Bits(0))))
    >();

    RCC::enableClocks<
        A == address::USART1 ?
                               rcc::apb2enr::USART1 :
#ifndef STM32F1XX
                               (A == address::USART6 ?
                                   rcc::apb2enr::USART6 :
                                   rcc::apb2enr::Bits(0))
#else // !STM32F1XX
                               rcc::apb2enr::Bits(0)
    #endif // !STM32F1XX
    >();
  }

  /**
   * @brief Disables the USART's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  template<address::E A>
  void Asynchronous<A>::disableClock()
  {
    RCC::disableClocks<
        A == address::USART2 ?
            rcc::apb1enr::USART2 :
            (A == address::USART3 ?
                rcc::apb1enr::USART3 :
                (A == address::UART4 ?
                    rcc::apb1enr::UART4 :
                    (A == address::UART5 ?
                                           rcc::apb1enr::UART5 :
                                           rcc::apb1enr::Bits(0))))
    >();

    RCC::disableClocks<
        A == address::USART1 ?
                               rcc::apb2enr::USART1 :
#ifndef STM32F1XX
                               (A == address::USART6 ?
                                   rcc::apb2enr::USART6 :
                                   rcc::apb2enr::Bits(0))
#else // !STM32F1XX
                               rcc::apb2enr::Bits(0)
    #endif // !STM32F1XX
    >();
  }

  /**
   * @brief Sends data through the UART.
   */
  template<address::E A>
  void Asynchronous<A>::sendData(u8 const data)
  {
    reinterpret_cast<Registers*>(A)->DR = data;
  }

  /**
   * @brief Gets data from the receiver buffer.
   */
  template<address::E A>
  u8 Asynchronous<A>::getData(void)
  {
    return reinterpret_cast<Registers*>(A)->DR;
  }

  /**
   * @brief Returns true if new data can be send.
   */
  template<address::E A>
  bool Asynchronous<A>::canSendDataYet(void)
  {
    return *(volatile bool*) (bitband::peripheral<
        A + registers::sr::OFFSET,
        registers::sr::bits::txe::POSITION>());
  }

  /**
   * @brief Returns true if there is new data available.
   */
  template<address::E A>
  bool Asynchronous<A>::isThereDataAvailable(void)
  {
    return *(volatile bool*) (bitband::peripheral<
        A + registers::sr::OFFSET,
        registers::sr::bits::rxne::POSITION>());
  }

  /**
   * @brief Sets the baud rate.
   * @note  Only valid for OVERSAMPLING_BY_16 configuration
   */
  template<address::E A>
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
  template<address::E A>
  template<
      usart::registers::cr1::bits::rwu::states::E RWU,
      usart::registers::cr1::bits::re::states::E RE,
      usart::registers::cr1::bits::te::states::E TE,
      usart::registers::cr1::bits::idleie::states::E IDLEIE,
      usart::registers::cr1::bits::rxneie::states::E RXNEIE,
      usart::registers::cr1::bits::tcie::states::E TCIE,
      usart::registers::cr1::bits::txeie::states::E TXEIE,
      usart::registers::cr1::bits::peie::states::E PEIE,
      usart::registers::cr1::bits::ps::states::E PS,
      usart::registers::cr1::bits::pce::states::E PCE,
      usart::registers::cr1::bits::wake::states::E WAKE,
      usart::registers::cr1::bits::m::states::E M,
      usart::registers::cr1::bits::ue::states::E UE,
      usart::registers::cr1::bits::over8::states::E OVER8,
      usart::registers::cr2::bits::stop::states::E STOP,
      usart::registers::cr3::bits::eie::states::E EIE,
      usart::registers::cr3::bits::hdsel::states::E HDSEL,
      usart::registers::cr3::bits::dmar::states::E DMAR,
      usart::registers::cr3::bits::dmat::states::E DMAT,
      usart::registers::cr3::bits::rtse::states::E RSTE,
      usart::registers::cr3::bits::ctse::states::E CTSE,
      usart::registers::cr3::bits::ctsie::states::E CTSIE,
      usart::registers::cr3::bits::onebit::states::E ONEBIT
  >
  void Asynchronous<A>::configure(void)
  {
    reinterpret_cast<Registers*>(A)->CR1 =
        RWU + RE + TE + IDLEIE + RXNEIE + TCIE + TXEIE + PEIE + PS +
            PCE + WAKE + M + UE + OVER8;

    reinterpret_cast<Registers*>(A)->CR2 = STOP;

    reinterpret_cast<Registers*>(A)->CR3 =
        EIE + HDSEL + DMAR + DMAT + RSTE + CTSE + CTSIE + ONEBIT;
  }

}  // namespace usart

