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

#include "clock.hpp"

#include "interrupt.hpp"

#include "peripheral/gpio.hpp"

typedef PA9 U1TX;
typedef PA10 U1RX;
typedef PC13 LED;

#include "peripheral/usart.hpp"

typedef USART1 USART;

#include "peripheral/dma.hpp"

typedef DMA2_STREAM7 DMA_U1TX;
typedef DMA2_STREAM5 DMA_U1RX;

char outputBuffer[] = "Hello World!\n\r";
char inputBuffer[1];

void configureGpio()
{
  GPIOA::enableClock();

  U1TX::setAlternateFunction(gpio::afr::USART1_3);
  U1TX::setMode(gpio::moder::ALTERNATE);

  U1RX::setAlternateFunction(gpio::afr::USART1_3);
  U1RX::setMode(gpio::moder::ALTERNATE);

  LED::enableClock();
  LED::setMode(gpio::moder::OUTPUT);
}

void configureUsart()
{
  USART::enableClock();

  USART::configure(
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
  USART::setBaudRate<
      2000000 /* bps */
  >();
}

void configureDma()
{
  DMA2::enableClock();

  DMA_U1TX::configure(
      dma::stream::cr::dmeie::DIRECT_MODE_ERROR_INTERRUPT_DISABLED,
      dma::stream::cr::teie::TRANSFER_ERROR_INTERRUPT_DISABLED,
      dma::stream::cr::htie::HALF_TRANSFER_INTERRUPT_DISABLED,
      dma::stream::cr::tcie::TRANSFER_COMPLETE_INTERRUPT_DISABLED,
      dma::stream::cr::pfctrl::DMA_FLOW_CONTROLLER,
      dma::stream::cr::dir::MEMORY_TO_PERIPHERAL,
      dma::stream::cr::circ::CIRCULAR_MODE_ENABLED,
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
  DMA_U1TX::setMemory0Address(&outputBuffer);
  DMA_U1TX::setPeripheralAddress(&USART1_REGS->DR);
  DMA_U1TX::setNumberOfTransactions(sizeof(outputBuffer));

  DMA_U1RX::configure(
      dma::stream::cr::dmeie::DIRECT_MODE_ERROR_INTERRUPT_DISABLED,
      dma::stream::cr::teie::TRANSFER_ERROR_INTERRUPT_DISABLED,
      dma::stream::cr::htie::HALF_TRANSFER_INTERRUPT_DISABLED,
      dma::stream::cr::tcie::TRANSFER_COMPLETE_INTERRUPT_ENABLED,
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
  DMA_U1RX::setMemory0Address(&inputBuffer);
  DMA_U1RX::setPeripheralAddress(&USART1_REGS->DR);
  DMA_U1RX::setNumberOfTransactions(sizeof(inputBuffer));
  DMA_U1RX::enableGlobalInterrupts();
  DMA_U1RX::enablePeripheral();
}

void configure()
{
  configureGpio();
  configureUsart();
  configureDma();
}

void loop()
{

}

int main()
{
  clk::initialize();

  configure();

  while (true) {
    loop();
  }
}

void interrupt::DMA2_Stream5()
{
  DMA_U1RX::clearTransferCompleteFlag();

  switch (inputBuffer[0]) {
    case 'E':
      LED::setLow();
      DMA_U1TX::disablePeripheral();
      break;

    case 'S':
      LED::setHigh();
      DMA_U1TX::enablePeripheral();
      break;
  }
}
