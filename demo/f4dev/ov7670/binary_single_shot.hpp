/*******************************************************************************
 *
 * Copyright (C) 2013 Jorge Aparicio <jorge.aparicio.r@gmail.com>
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
// What does this demo do?
// Upon receiving the letter 'S' via USART1, the uC will capture a QCIF frame
// in the default YCbCr422 image format from the OV7670 camera.
// After the capture is complete, all the frame data will be reported via the
// USART1 in binary form:
//
// The PC side should be able to display the uncompressed image.
//
// Tested with following clock configuration:
// STM32F407VE - custom board (F4Dev)
// SYSCLK = AHB = APB1 = APB2 = 42 MHz (HSI + PLL)
// MCO1 = 16 MHz (HSI)
//
// qSerialTerm was used to display the image.
//
// ** Don't forget to enable interrupts.

// Camera
#define OV7670_SCCB_ADDRESS 0x21

// Image format: "QCIF" (the standard defines QCIF as 176 x 144)
// Hoever my OV7670 outputs 174 x 144 frames
#define WIDTH 174
#define HEIGHT 144
#define BYTES_PER_PIXEL 2

// Communication
#define UART_BAUD_RATE 115200

#include "clock.hpp"

#include "interrupt.hpp"

#include "peripheral/gpio.hpp"

// SCCB (SDIOC: PC14, SDIOD: PC15)
#include "driver/sccb.hpp"

typedef sccb::Functions<
    gpio::GPIOC,
    14,
    gpio::GPIOC,
    15,
    tim::TIM6,
    400000
    > SCCB;

// MCO (XCLK)
typedef PA8 MCO1; // a.k.a. XCLK

// DCMI
#include "peripheral/dcmi.hpp"

typedef PC6 DCMI_D0;
typedef PC7 DCMI_D1;
typedef PE0 DCMI_D2;
typedef PE1 DCMI_D3;
typedef PE4 DCMI_D4;
typedef PB6 DCMI_D5;
typedef PE5 DCMI_D6;
typedef PE6 DCMI_D7;
typedef PA6 DCMI_PIXCK; // a.k.a. PCLK
typedef PA4 DCMI_HSYNC; // a.k.a. HREF
typedef PB7 DCMI_VSYNC;

// USART
#include "peripheral/usart.hpp"

typedef PA9 U1_TX;
typedef PA10 U1_RX;

// LED
typedef PC13 LED;

// DMA
#include "peripheral/dma.hpp"

typedef DMA2_STREAM1 DMA_DCMI;
typedef DMA2_STREAM5 DMA_U1_RX;
typedef DMA2_STREAM7 DMA_U1_TX;

u8 outputBuffer[WIDTH * HEIGHT * BYTES_PER_PIXEL];
char inputBuffer;

void initializeGpio()
{
  // Enable all ports
  GPIOA::enableClock();
  GPIOB::enableClock();
  GPIOC::enableClock();
  GPIOD::enableClock();
  GPIOE::enableClock();

  // SCCB
  SCCB::initialize();

  // MCO (XCLK)
  MCO1::setAlternateFunction(gpio::afr::SYSTEM);
  MCO1::setMode(gpio::moder::ALTERNATE);

  // DCMI
  DCMI_D0::setAlternateFunction(gpio::afr::DCMI);
  DCMI_D0::setMode(gpio::moder::ALTERNATE);

  DCMI_D1::setAlternateFunction(gpio::afr::DCMI);
  DCMI_D1::setMode(gpio::moder::ALTERNATE);

  DCMI_D2::setAlternateFunction(gpio::afr::DCMI);
  DCMI_D2::setMode(gpio::moder::ALTERNATE);

  DCMI_D3::setAlternateFunction(gpio::afr::DCMI);
  DCMI_D3::setMode(gpio::moder::ALTERNATE);

  DCMI_D4::setAlternateFunction(gpio::afr::DCMI);
  DCMI_D4::setMode(gpio::moder::ALTERNATE);

  DCMI_D5::setAlternateFunction(gpio::afr::DCMI);
  DCMI_D5::setMode(gpio::moder::ALTERNATE);

  DCMI_D6::setAlternateFunction(gpio::afr::DCMI);
  DCMI_D6::setMode(gpio::moder::ALTERNATE);

  DCMI_D7::setAlternateFunction(gpio::afr::DCMI);
  DCMI_D7::setMode(gpio::moder::ALTERNATE);

  DCMI_PIXCK::setAlternateFunction(gpio::afr::DCMI);
  DCMI_PIXCK::setMode(gpio::moder::ALTERNATE);

  DCMI_HSYNC::setAlternateFunction(gpio::afr::DCMI);
  DCMI_HSYNC::setMode(gpio::moder::ALTERNATE);

  DCMI_VSYNC::setAlternateFunction(gpio::afr::DCMI);
  DCMI_VSYNC::setMode(gpio::moder::ALTERNATE);

  // USART
  U1_TX::setAlternateFunction(gpio::afr::USART1_3);
  U1_TX::setMode(gpio::moder::ALTERNATE);

  U1_RX::setAlternateFunction(gpio::afr::USART1_3);
  U1_RX::setMode(gpio::moder::ALTERNATE);

  // LED
  LED::enableClock();
  LED::setMode(gpio::moder::OUTPUT);
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
      usart::cr3::dmar::RECEIVER_DMA_ENABLED,
      usart::cr3::dmat::TRANSMITTER_DMA_ENABLED,
      usart::cr3::rtse::RTS_HARDWARE_FLOW_DISABLED,
      usart::cr3::ctse::CTS_HARDWARE_FLOW_DISABLED,
      usart::cr3::ctsie::CTS_INTERRUPT_DISABLED,
      usart::cr3::onebit::THREE_SAMPLE_BIT_METHOD);
  USART1::setBaudRate<
  UART_BAUD_RATE /* bps */
  >();
}

void initializeDma()
{
  DMA2::enableClock();

  DMA_DCMI::configure(
      dma::stream::cr::dmeie::DIRECT_MODE_ERROR_INTERRUPT_DISABLED,
      dma::stream::cr::teie::TRANSFER_ERROR_INTERRUPT_DISABLED,
      dma::stream::cr::htie::HALF_TRANSFER_INTERRUPT_DISABLED,
      dma::stream::cr::tcie::TRANSFER_COMPLETE_INTERRUPT_DISABLED,
      dma::stream::cr::pfctrl::DMA_FLOW_CONTROLLER,
      dma::stream::cr::dir::PERIPHERAL_TO_MEMORY,
      dma::stream::cr::circ::CIRCULAR_MODE_ENABLED,
      dma::stream::cr::pinc::PERIPHERAL_INCREMENT_MODE_DISABLED,
      dma::stream::cr::minc::MEMORY_INCREMENT_MODE_ENABLED,
      dma::stream::cr::psize::PERIPHERAL_SIZE_32BITS,
      dma::stream::cr::msize::MEMORY_SIZE_32BITS,
      dma::stream::cr::pincos::PERIPHERAL_INCREMENT_OFFSET_SIZE_32BITS,
      dma::stream::cr::pl::PRIORITY_LEVEL_VERY_HIGH,
      dma::stream::cr::dbm::DOUBLE_BUFFER_MODE_DISABLED,
      dma::stream::cr::ct::CURRENT_TARGET_MEMORY_0,
      dma::stream::cr::pburst::PERIPHERAL_BURST_TRANSFER_SINGLE,
      dma::stream::cr::mburst::MEMORY_BURST_TRANSFER_SINGLE,
      dma::stream::cr::chsel::CHANNEL_1);
  DMA_DCMI::setMemory0Address(&outputBuffer);
  DMA_DCMI::setPeripheralAddress(&DCMI_REGS->DR);
  DMA_DCMI::setNumberOfTransactions(sizeof(outputBuffer) / sizeof(u32));
  DMA_DCMI::enablePeripheral();

  DMA_U1_TX::configure(
      dma::stream::cr::dmeie::DIRECT_MODE_ERROR_INTERRUPT_DISABLED,
      dma::stream::cr::teie::TRANSFER_ERROR_INTERRUPT_DISABLED,
      dma::stream::cr::htie::HALF_TRANSFER_INTERRUPT_DISABLED,
      dma::stream::cr::tcie::TRANSFER_COMPLETE_INTERRUPT_ENABLED,
      dma::stream::cr::pfctrl::DMA_FLOW_CONTROLLER,
      dma::stream::cr::dir::MEMORY_TO_PERIPHERAL,
      dma::stream::cr::circ::CIRCULAR_MODE_DISABLED,
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
  DMA_U1_TX::setPeripheralAddress(&USART1_REGS->DR);
  DMA_U1_TX::unmaskInterrupts();

  DMA_U1_RX::configure(
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
  DMA_U1_RX::setMemory0Address(&inputBuffer);
  DMA_U1_RX::setPeripheralAddress(&USART1_REGS->DR);
  DMA_U1_RX::setNumberOfTransactions(sizeof(inputBuffer));
  DMA_U1_RX::unmaskInterrupts();
  DMA_U1_RX::enablePeripheral();
}

void initializeDcmi()
{
  DCMI::enableClock();

  DCMI::configure(
      dcmi::cr::capture::CAPTURE_DISABLED,
      dcmi::cr::cm::SNAPSHOT_MODE,
      dcmi::cr::crop::FULL_IMAGE_IS_CAPTURED,
      dcmi::cr::jpeg::UNCOMPRESSED_VIDEO_FORMAT,
      dcmi::cr::ess::HARDWARE_SYNCHRONIZATION,
      dcmi::cr::pckpol::CAPTURE_ON_RISING_EDGE,
      dcmi::cr::hspol::DATA_CAPTURED_ON_HSYNC_HIGH,
      dcmi::cr::vspol::DATA_CAPTURED_ON_VSYNC_LOW,
      dcmi::cr::fcrc::ALL_FRAMES_ARE_CAPTURED,
      dcmi::cr::edm::EVERY_PIXEL_CLOCK_CAPTURES_8_BITS,
      dcmi::cr::enable::DCMI_ENABLED);
  DCMI::enableCaptureCompleInterrupt();
  DCMI::unmaskInterrupts();
  DCMI::enablePeripheral();
}

void initializePeripherals()
{
  initializeGpio();
  initializeUsart();
  initializeDma();
  initializeDcmi();
}

// Change to QCIF format
void configureOV7670()
{
  u8 reg;

  // COM3 register: Enable format scaling
  if(!SCCB::readSlaveRegister(OV7670_SCCB_ADDRESS, 0x0C, reg))
    while(true) {}

  if(!SCCB::writeSlaveRegister(OV7670_SCCB_ADDRESS, 0x0C, reg | 0b00001000))
    while(true) {}

  // COM7 register: Select QCIF format
  if(!SCCB::readSlaveRegister(OV7670_SCCB_ADDRESS, 0x12, reg))
    while(true) {}

  if(!SCCB::writeSlaveRegister(OV7670_SCCB_ADDRESS, 0x12, (reg & 0b11000111) | 0b00001000))
    while(true) {}
}

void loop()
{

}

int main()
{
  clk::initialize();

  initializePeripherals();
  configureOV7670();

  while (true) {
    loop();
  }
}

void interrupt::DMA2_Stream5()
{
  DMA_U1_RX::clearTransferCompleteFlag();

  switch (inputBuffer) {
    case 'S':
      DCMI::startCapture();
      break;
  }
}

void interrupt::DCMI()
{
  DCMI::clearAllFlags();

  LED::setHigh();

  DMA_U1_TX::setMemory0Address(&outputBuffer);
  DMA_U1_TX::setNumberOfTransactions(sizeof(outputBuffer));
  DMA_U1_TX::enablePeripheral();
}

void interrupt::DMA2_Stream7()
{
  DMA_U1_TX::clearTransferCompleteFlag();

  LED::setLow();
}
