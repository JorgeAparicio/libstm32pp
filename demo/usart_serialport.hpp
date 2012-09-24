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
#define UART_BAUD_RATE 9600

#include "clock.hpp"

#include "interrupt.hpp"

#include "peripheral/gpio.hpp"

#include "peripheral/usart.hpp"

typedef PA9 U1TX;
typedef PA10 U1RX;

const char msg[] = "Hello World!\n\r";

void initializeGpio()
{
  GPIOA::enableClock();

#ifdef STM32F1XX
  U1TX::setMode(gpio::cr::AF_PUSH_PULL_2MHZ);

  U1RX::setMode(gpio::cr::FLOATING_INPUT);
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
      UART_BAUD_RATE /* bps */
  >();
}

void initializePeripherals()
{
  initializeGpio();
  initializeUsart();
}

void loop()
{
  for (u8 i = 0; i < sizeof(msg); i++) {
    while (!USART1::canSendDataYet()) {
    }

    USART1::sendData(msg[i]);
  }
}

int main()
{
  clk::initialize();

  initializePeripherals();

  while (true) {
    loop();
  }
}
