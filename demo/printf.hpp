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

// DO NOT INCLUDE THIS FILE ANYWHERE. THIS DEMO IS JUST A REFERENCE TO BE USED
// IN YOUR MAIN SOURCE FILE.
////////////////////////////////////////////////////////////////////////////////
// Tested on STM32VLDISCOVERY
// Tested on F4Dev
////////////////////////////////////////////////////////////////////////////////
// IMPORTANT: Include system_call_cpp.hpp in a source file once, this will
//            define the newlib stubs necessary for new, delete, printf, etc.
//
//            Also go to sytem_call.hpp and define which USART port will be
//            used for stdout.
#include "clock.hpp"

#include "peripheral/gpio.hpp"

typedef PA9 U1TX;
typedef PA10 U1RX;

#include "peripheral/usart.hpp"

#include <stdio.h>

void initializeGpio()
{
  GPIOA::enableClock();

#ifdef STM32F1XX
  U1TX::setMode(gpio::cr::AF_PUSH_PULL_2MHZ);

  U1RX::setMode(gpio::cr::AF_PUSH_PULL_2MHZ);
#else
  U1TX::setAlternateFunction(gpio::afr::USART1_3);
  U1TX::setMode(gpio::moder::ALTERNATE);

  U1RX::setAlternateFunction(gpio::afr::USART1_3);
  U1RX::setMode(gpio::moder::ALTERNATE);
#endif
}

void initializeUsart()
{
  USART1::enableClock();
  USART1::configure(
      usart::cr1::rwu::RECEIVER_IN_ACTIVE_MODE,
      usart::cr1::re::RECEIVER_ENABLED,
      usart::cr1::te::TRANSMITTER_ENABLED,
      usart::cr1::idleie::IDLE_INTERRUPT_DISABLED,
      usart::cr1::rxneie::RXNE_ORE_INTERRUPT_DISABLED,
      usart::cr1::tcie::TC_INTERRUPT_DISABLED,
      usart::cr1::txeie::TXEIE_INTERRUPT_DISABLED,
      usart::cr1::peie::PEIE_INTERRUPT_DISABLED,
      usart::cr1::ps::EVEN_PARITY,
      usart::cr1::pce::PARITY_CONTROL_DISABLED,
      usart::cr1::wake::WAKE_ON_IDLE_LINE,
      usart::cr1::m::START_8_DATA_N_STOP,
      usart::cr1::ue::USART_ENABLED,
      usart::cr1::over8::OVERSAMPLING_BY_16,
      usart::cr2::stop::_1_STOP_BIT,
      usart::cr3::eie::ERROR_INTERRUPT_DISABLED,
      usart::cr3::hdsel::FULL_DUPLEX,
      usart::cr3::dmar::RECEIVER_DMA_DISABLED,
      usart::cr3::dmat::TRANSMITTER_DMA_DISABLED,
      usart::cr3::rtse::RTS_HARDWARE_FLOW_DISABLED,
      usart::cr3::ctse::CTS_HARDWARE_FLOW_DISABLED,
      usart::cr3::ctsie::CTS_INTERRUPT_DISABLED,
      usart::cr3::onebit::ONE_SAMPLE_BIT_METHOD);
  USART1::setBaudRate<
      9600 /* bps */
  >();
}

void initializePeripherals()
{
  initializeGpio();
  initializeUsart();
}

void loop()
{
  // IMPORTANT: printf employs a buffer under the hood, and only writes to
  //            stdout (UART) when a '\n' (newline) character is spotted.
  // The next line won't print
//  printf("Hello World!");
  // Any of the following lines will print
//  printf("Hello World!\n\r"); // Uses the buffer and more flash memory.
  printf("\rHello World!\n"); // Uses less flash memory.
}

int main()
{
  clk::initialize();

  initializePeripherals();

  while (true) {
    loop();
  }
}
