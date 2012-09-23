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
// This demo is for configuring the BlueTooth.
// If want to communicate through the BlueTooth, check the usart_bluetooth demo.
// The BlueTooth must be UNPAIRED and a correct baud rate must be used.
#include "clock.hpp"

#include "interrupt.hpp"

#include "peripheral/gpio.hpp"

typedef PB10 U1TX;
typedef PB11 U1RX;

#include "peripheral/usart.hpp"

#include "peripheral/dma.hpp"

typedef DMA1_STREAM3 DMA_U3TX;
typedef DMA1_STREAM1 DMA_U3RX;

char ans[16];

// COMMANDS
// Uncomment one of the char cmd[] lines.
// Note:  "string" is a null-terminated char array.

// BlueTooth responds "OK"
char cmd[] = "AT";

// BlueTooth responds "OKlinvorV1.5"
//char cmd[] = "AT+VERSION";

// Sets BlueTooth name to "foo" (can be any other name)
// BlueTooth responds OKsetname
//char cmd[] = "AT+NAMEfoo";

// Sets BlueTooth pin code to "1234" (4 digits)
// BlueTooth responds OKsetPIN
//char cmd[] = "AT+PIN1234";

// Sets BlueTooth baud rate to 1200
// BlueTooth responds OK1200
//char cmd[] = "AT+BAUD1";

// Sets BlueTooth baud rate to 2400
// BlueTooth responds OK2400
//char cmd[] = "AT+BAUD2";

// Sets BlueTooth baud rate to 4800
// BlueTooth responds OK4800
//char cmd[] = "AT+BAUD3";

// Sets BlueTooth baud rate to 9600
// BlueTooth responds OK9600
//char cmd[] = "AT+BAUD4";

// Sets BlueTooth baud rate to 19200
// BlueTooth responds OK19200
//char cmd[] = "AT+BAUD5";

// Sets BlueTooth baud rate to 38400
// BlueTooth responds OK38400
//char cmd[] = "AT+BAUD6";

// Sets BlueTooth baud rate to 57600
// BlueTooth responds OK57600
//char cmd[] = "AT+BAUD7";

// Sets BlueTooth baud rate to 115200
// BlueTooth responds OK115200
//char cmd[] = "AT+BAUD8";

// Sets BlueTooth baud rate to 230400
// BlueTooth responds OK230400
//char cmd[] = "AT+BAUD9";

// Sets BlueTooth baud rate to 460800
// BlueTooth responds OK460800
//char cmd[] = "AT+BAUDA";

// Sets BlueTooth baud rate to 921600
// BlueTooth responds OK921600
//char cmd[] = "AT+BAUDB";

// Sets BlueTooth baud rate to 1382400
// BlueTooth responds OK1382400
//char cmd[] = "AT+BAUDC";

void initializeGpio()
{
  GPIOB::enableClock();

  U1TX::setAlternateFunction(gpio::afr::USART1_3);
  U1TX::setMode(gpio::moder::ALTERNATE);

  U1RX::setAlternateFunction(gpio::afr::USART1_3);
  U1RX::setMode(gpio::moder::ALTERNATE);
}

void initializeUsart()
{
  USART3::enableClock();
  USART3::configure(
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
      usart::cr3::dmar::RECEIVER_DMA_ENABLED,
      usart::cr3::dmat::TRANSMITTER_DMA_ENABLED,
      usart::cr3::rtse::RTS_HARDWARE_FLOW_DISABLED,
      usart::cr3::ctse::CTS_HARDWARE_FLOW_DISABLED,
      usart::cr3::ctsie::CTS_INTERRUPT_DISABLED,
      usart::cr3::onebit::THREE_SAMPLE_BIT_METHOD);
  USART3::setBaudRate<
      9600 /* bps */
  >();
}

void initializeDma()
{
  DMA1::enableClock();

  DMA_U3TX::configure(
      dma::stream::cr::dmeie::DIRECT_MODE_ERROR_INTERRUPT_DISABLED,
      dma::stream::cr::teie::TRANSFER_ERROR_INTERRUPT_DISABLED,
      dma::stream::cr::htie::HALF_TRANSFER_INTERRUPT_DISABLED,
      dma::stream::cr::tcie::TRANSFER_COMPLETE_INTERRUPT_DISABLED,
      dma::stream::cr::pfctrl::DMA_FLOW_CONTROLLER,
      dma::stream::cr::dir::MEMORY_TO_PERIPHERAL,
      dma::stream::cr::circ::CIRCULAR_MODE_DISABLED,
      dma::stream::cr::pinc::PERIPHERAL_INCREMENT_MODE_DISABLED,
      dma::stream::cr::minc::MEMORY_INCREMENT_MODE_ENABLED,
      dma::stream::cr::psize::PERIPHERAL_SIZE_8BITS,
      dma::stream::cr::msize::MEMORY_SIZE_8BITS,
      dma::stream::cr::pincos::PERIPHERAL_INCREMENT_OFFSET_SIZE_PSIZE,
      dma::stream::cr::pl::PRIORITY_LEVEL_MEDIUM,
      dma::stream::cr::dbm::DOUBLE_BUFFER_MODE_DISABLED,
      dma::stream::cr::ct::CURRENT_TARGET_MEMORY_0,
      dma::stream::cr::pburst::PERIPHERAL_BURST_TRANSFER_SINGLE,
      dma::stream::cr::mburst::MEMORY_BURST_TRANSFER_SINGLE,
      dma::stream::cr::chsel::CHANNEL_4);
  DMA_U3TX::setMemory0Address(&cmd);
  DMA_U3TX::setPeripheralAddress(&USART3_REGS->DR);
  DMA_U3TX::setNumberOfTransactions(sizeof(cmd) - 1); // -1 for the null char

  DMA_U3RX::configure(
      dma::stream::cr::dmeie::DIRECT_MODE_ERROR_INTERRUPT_DISABLED,
      dma::stream::cr::teie::TRANSFER_ERROR_INTERRUPT_DISABLED,
      dma::stream::cr::htie::HALF_TRANSFER_INTERRUPT_DISABLED,
      dma::stream::cr::tcie::TRANSFER_COMPLETE_INTERRUPT_DISABLED,
      dma::stream::cr::pfctrl::DMA_FLOW_CONTROLLER,
      dma::stream::cr::dir::PERIPHERAL_TO_MEMORY,
      dma::stream::cr::circ::CIRCULAR_MODE_ENABLED,
      dma::stream::cr::pinc::PERIPHERAL_INCREMENT_MODE_DISABLED,
      dma::stream::cr::minc::MEMORY_INCREMENT_MODE_ENABLED,
      dma::stream::cr::psize::PERIPHERAL_SIZE_8BITS,
      dma::stream::cr::msize::MEMORY_SIZE_8BITS,
      dma::stream::cr::pincos::PERIPHERAL_INCREMENT_OFFSET_SIZE_PSIZE,
      dma::stream::cr::pl::PRIORITY_LEVEL_HIGH,
      dma::stream::cr::dbm::DOUBLE_BUFFER_MODE_DISABLED,
      dma::stream::cr::ct::CURRENT_TARGET_MEMORY_0,
      dma::stream::cr::pburst::PERIPHERAL_BURST_TRANSFER_SINGLE,
      dma::stream::cr::mburst::MEMORY_BURST_TRANSFER_SINGLE,
      dma::stream::cr::chsel::CHANNEL_4);
  DMA_U3RX::setMemory0Address(&ans);
  DMA_U3RX::setPeripheralAddress(&USART3_REGS->DR);
  DMA_U3RX::setNumberOfTransactions(sizeof(ans));
  DMA_U3RX::enablePeripheral();
}

void initializePeripherals()
{
  initializeGpio();
  initializeUsart();
  initializeDma();

  DMA_U3TX::enablePeripheral(); // This line will send the command.
  // Check the ans variable afterwards.
}

void loop()
{

}

int main()
{
  clk::initialize();

  initializePeripherals();

  while (true) {
    loop();
  }
}
