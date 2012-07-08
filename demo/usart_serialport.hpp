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

// DO NOT INCLUDE THIS FILE ANYWHERE. THIS DEMO IS JUST A REFERENCE TO BE USED
// IN YOUR MAIN SOURCE FILE.

#include "interrupt.hpp"
#include "peripheral/gpio.hpp"
#include "peripheral/rcc.hpp"
#include "peripheral/usart.hpp"

// TODO USART demo for STM32F1XX

typedef PA9 U1TX;
typedef PA10 U1RX;

const char msg[] = "Hello World!\n\r";

int main()
{
  RCC::enableClocks<
      rcc::registers::ahb1enr::bits::GPIOA
  >();

  PA9::setAlternateFunction<
      gpio::registers::afr::states::USART1_3
  >();

  PA9::setMode<
      gpio::registers::moder::states::ALTERNATE
  >();

  PA10::setAlternateFunction<
      gpio::registers::afr::states::USART1_3
  >();

  PA10::setMode<
      gpio::registers::moder::states::ALTERNATE
  >();

  RCC::enableClocks<
      rcc::registers::apb2enr::bits::USART1
  >();

  RCC::enableClocks<
      rcc::registers::apb2enr::bits::USART1
  >();

  USART1::configure<
      usart::registers::cr1::bits::rwu::states::RECEIVER_IN_ACTIVE_MODE,
      usart::registers::cr1::bits::re::states::RECEIVER_ENABLED,
      usart::registers::cr1::bits::te::states::TRANSMITTER_ENABLED,
      usart::registers::cr1::bits::idleie::states::IDLE_INTERRUPT_DISABLED,
      usart::registers::cr1::bits::rxneie::states::RXNE_ORE_INTERRUPT_DISABLED,
      usart::registers::cr1::bits::tcie::states::TC_INTERRUPT_DISABLED,
      usart::registers::cr1::bits::txeie::states::TXEIE_INTERRUPT_DISABLED,
      usart::registers::cr1::bits::peie::states::PEIE_INTERRUPT_DISABLED,
      usart::registers::cr1::bits::ps::states::EVEN_PARITY,
      usart::registers::cr1::bits::pce::states::PARITY_CONTROL_DISABLED,
      usart::registers::cr1::bits::wake::states::WAKE_ON_ADDRESS_MARK,
      usart::registers::cr1::bits::m::states::START_8_DATA_N_STOP,
      usart::registers::cr1::bits::ue::states::USART_ENABLED,
      usart::registers::cr1::bits::over8::states::OVERSAMPLING_BY_16,
      usart::registers::cr2::bits::stop::states::_1_STOP_BIT,
      usart::registers::cr3::bits::eie::states::ERROR_INTERRUPT_DISABLED,
      usart::registers::cr3::bits::hdsel::states::FULL_DUPLEX,
      usart::registers::cr3::bits::dmar::states::RECEIVER_DMA_DISABLED,
      usart::registers::cr3::bits::dmat::states::TRANSMITTER_DMA_DISABLED,
      usart::registers::cr3::bits::rtse::states::RTS_HARDWARE_FLOW_DISABLED,
      usart::registers::cr3::bits::ctse::states::CTS_HARDWARE_FLOW_DISABLED,
      usart::registers::cr3::bits::ctsie::states::CTS_INTERRUPT_DISABLED,
      usart::registers::cr3::bits::onebit::states::ONE_SAMPLE_BIT_METHOD
  >();

  USART1::setBaudRate(1667);  // 9600 bps @ 16MHZ APB2

  for (u8 i = 0; i < sizeof(msg); i++) {
    while (!USART1::canSendDataYet()) {
    }

    USART1::sendData(msg[i]);
  }

  while (true) {
  }
}
