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
#include "core.hpp"
#include "driver.hpp"
#include "peripheral.hpp"

typedef PC13 S1;
typedef PC0 S2;
typedef PC2 S3;
typedef PA0 S4;
typedef PA2 S5;
typedef PA5 S6;
typedef PA7 S7;
typedef PC5 S8;
typedef PB1 S9;
typedef PE7 S10;
typedef PE9 S11;
typedef PE11 S12;
typedef PE13 S13;
typedef PE15 S14;
typedef PB11 S15;
typedef PB12 S16;
typedef PB14 S17;
typedef PD8 S18;
typedef PD10 S19;
typedef PD12 S20;

typedef DMA2_STREAM2 DMA_RX;

typedef USART6 USART;
typedef PC6 TX;
typedef PC7 RX;

#define NUMBER_OF_SERVOS 20

servo::Functions<
    tim::TIM6,
    50,  // Hz
    tim::TIM7,
    1500,  // us
    NUMBER_OF_SERVOS
> Servo;

u8 inputBuffer[2 * NUMBER_OF_SERVOS];

void mcuSetup()
{
  // GPIO configuration

  GPIOA::enableClock();
  GPIOB::enableClock();
  GPIOC::enableClock();
  GPIOD::enableClock();
  GPIOE::enableClock();

  S1::setMode(gpio::moder::OUTPUT);
  S2::setMode(gpio::moder::OUTPUT);
  S3::setMode(gpio::moder::OUTPUT);
  S4::setMode(gpio::moder::OUTPUT);
  S5::setMode(gpio::moder::OUTPUT);
  S6::setMode(gpio::moder::OUTPUT);
  S7::setMode(gpio::moder::OUTPUT);
  S8::setMode(gpio::moder::OUTPUT);
  S9::setMode(gpio::moder::OUTPUT);
  S10::setMode(gpio::moder::OUTPUT);
  S11::setMode(gpio::moder::OUTPUT);
  S12::setMode(gpio::moder::OUTPUT);
  S13::setMode(gpio::moder::OUTPUT);
  S14::setMode(gpio::moder::OUTPUT);
  S15::setMode(gpio::moder::OUTPUT);
  S16::setMode(gpio::moder::OUTPUT);
  S17::setMode(gpio::moder::OUTPUT);
  S18::setMode(gpio::moder::OUTPUT);
  S19::setMode(gpio::moder::OUTPUT);
  S20::setMode(gpio::moder::OUTPUT);

  // USART Configuration

  USART::enableClock();

  TX::setAlternateFunction(gpio::afr::USART4_6);
  TX::setMode(gpio::moder::ALTERNATE);
  RX::setAlternateFunction(gpio::afr::USART4_6);
  RX::setMode(gpio::moder::ALTERNATE);

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
      usart::cr3::onebit::ONE_SAMPLE_BIT_METHOD);

  USART::setBaudRate<
      921600 /* bps */
  >();

  // DMA Configuration

  DMA2::enableClock();

  DMA_RX::configure(
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
      dma::stream::cr::chsel::CHANNEL_5);

  NVIC::enableInterrupt<
      nvic::irqn::DMA2_Stream2
  >();

  DMA_RX::setNumberOfTransactions(sizeof(inputBuffer));
  DMA_RX::setPeripheralAddress(&USART6_REGS->DR);
  DMA_RX::setMemory0Address(&inputBuffer);
  DMA_RX::enablePeripheral();

  // Servo configuration

  Servo.setPin(0, (u32*) PC13::OUT_ADDRESS);
  Servo.setPin(1, (u32*) PC0::OUT_ADDRESS);
  Servo.setPin(2, (u32*) PC2::OUT_ADDRESS);
  Servo.setPin(3, (u32*) PA0::OUT_ADDRESS);
  Servo.setPin(4, (u32*) PA2::OUT_ADDRESS);
  Servo.setPin(5, (u32*) PA5::OUT_ADDRESS);
  Servo.setPin(6, (u32*) PA7::OUT_ADDRESS);
  Servo.setPin(7, (u32*) PC5::OUT_ADDRESS);
  Servo.setPin(8, (u32*) PB1::OUT_ADDRESS);
  Servo.setPin(9, (u32*) PE7::OUT_ADDRESS);
  Servo.setPin(10, (u32*) PE9::OUT_ADDRESS);
  Servo.setPin(11, (u32*) PE11::OUT_ADDRESS);
  Servo.setPin(12, (u32*) PE13::OUT_ADDRESS);
  Servo.setPin(13, (u32*) PE15::OUT_ADDRESS);
  Servo.setPin(14, (u32*) PB11::OUT_ADDRESS);
  Servo.setPin(15, (u32*) PB12::OUT_ADDRESS);
  Servo.setPin(16, (u32*) PB14::OUT_ADDRESS);
  Servo.setPin(17, (u32*) PD8::OUT_ADDRESS);
  Servo.setPin(18, (u32*) PD10::OUT_ADDRESS);
  Servo.setPin(19, (u32*) PD12::OUT_ADDRESS);

  RCC::enableClocks<
      rcc::apb1enr::TIM6,
      rcc::apb1enr::TIM7
  >();

  NVIC::enableInterrupt<
      nvic::irqn::TIM6_DAC
  >();

  NVIC::enableInterrupt<
      nvic::irqn::TIM7
  >();

  Servo.initialize();
  Servo.start();

}

void mcuLoop()
{

}

int main(void)
{
  mcuSetup();

  while (true)
  {
    mcuLoop();
  }
}

void interrupt::TIM6_DAC()
{
  Servo.onPeriodTimerInterrupt();
}

void interrupt::TIM7()
{
  Servo.onDutyCycleTimerInterrupt();
}

void interrupt::DMA2_Stream2()
{
  DMA_RX::clearTransferCompleteFlag();

  Servo.load((s16 (&)[NUMBER_OF_SERVOS]) (inputBuffer));
}
