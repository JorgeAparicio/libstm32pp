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
// In this demo, the device reports the readings from its onboard Magnetometer,
// Gyroscope and Accelerometer.
// When you send a 'S' character, the device will start sampling and sending
// the readings through UART<->USB.
// When you send a 'E' character, the device will stop sampling.
////////////////////////////////////////////////////////////////////////////////
// IMPORTANT: Include system_call_cpp.hpp in a source file once, this will
//            define the newlib stubs necessary for new, delete, printf, etc.
//
//            Also go to sytem_call.hpp and define USART1 as STDOUT.
////////////////////////////////////////////////////////////////////////////////
#define UART_BAUD_RATE 115200
#define MARG_SAMPLE_RATE 10

#include "clock.hpp"

#include "interrupt.hpp"

#include "peripheral/gpio.hpp"

typedef PA9 U1TX;
typedef PA10 U1RX;

typedef PB8 SCL;
typedef PB9 SDA;

typedef PC13 LED;

#include "peripheral/usart.hpp"

#include "peripheral/dma.hpp"

typedef DMA2_STREAM5 DMA_U1RX;
typedef DMA2_STREAM7 DMA_U1TX;

#include "peripheral/i2c.hpp"

#include "peripheral/tim.hpp"

#include "driver/lsm303dlhc.hpp"

typedef lsm303dlhc::accelerometer::Functions<i2c::I2C1> Accelerometer;
typedef lsm303dlhc::magnetometer::Functions<i2c::I2C1> Magnetometer;

#include "driver/l3gd20.hpp"

typedef l3gd20::Functions<i2c::I2C1, l3gd20::L3G4200D_1> Gyroscope;
//typedef l3gd20::Functions<i2c::I2C1, l3gd20::L3GD20_1> Gyroscope;

#include <stdio.h>

bool go = false;

union S16 {
    s16 value;
    u8 byte[2];
};

struct XYZ {
    S16 X;
    S16 Y;
    S16 Z;
};

struct MARG {
    XYZ M;
    XYZ AR;
    XYZ G;
};

MARG reading;
char input;

void initializeGpio()
{
  GPIOA::enableClock();
  GPIOB::enableClock();

  U1TX::setAlternateFunction(gpio::afr::USART1_3);
  U1TX::setMode(gpio::moder::ALTERNATE);

  U1RX::setAlternateFunction(gpio::afr::USART1_3);
  U1RX::setMode(gpio::moder::ALTERNATE);

  SCL::setAlternateFunction(gpio::afr::I2C);
  SCL::setPullMode(gpio::pupdr::PULL_UP);
  SCL::setMode(gpio::moder::ALTERNATE);

  SDA::setAlternateFunction(gpio::afr::I2C);
  SDA::setPullMode(gpio::pupdr::PULL_UP);
  SDA::setMode(gpio::moder::ALTERNATE);

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
      usart::cr3::dmat::TRANSMITTER_DMA_DISABLED,
      usart::cr3::rtse::RTS_HARDWARE_FLOW_DISABLED,
      usart::cr3::ctse::CTS_HARDWARE_FLOW_DISABLED,
      usart::cr3::ctsie::CTS_INTERRUPT_DISABLED,
      usart::cr3::onebit::THREE_SAMPLE_BIT_METHOD);
  USART1::setBaudRate<
      UART_BAUD_RATE /* bps */
  >();
}

void initializeI2c()
{
  I2C1::enableClock();
  I2C1::configure(
      i2c::cr1::pe::PERIPHERAL_ENABLED,
      i2c::cr1::enpec::PACKET_ERROR_CHECKING_DISABLED,
      i2c::cr1::engc::GENERAL_CALL_DISABLED,
      i2c::cr1::nostretch::CLOCK_STRETCHING_DISABLED,
      i2c::cr2::iterren::ERROR_INTERRUPT_DISABLED,
      i2c::cr2::itevten::EVENT_INTERRUPT_DISABLED,
      i2c::cr2::itbufen::BUFFER_INTERRUPT_DISABLED,
      i2c::cr2::dmaen::DMA_REQUEST_DISABLED,
      i2c::cr2::last::NEXT_DMA_IS_NOT_THE_LAST_TRANSFER);
  I2C1::configureClock<
      i2c::ccr::f_s::FAST_MODE,
      i2c::ccr::duty::T_LOW_2_T_HIGH_1,
      400000 /* Hz */
  >();
}

void initializeTimer()
{
  TIM6::enableClock();
  TIM6::configurePeriodicInterrupt<
      MARG_SAMPLE_RATE /* Hz */
  >();
}

void initializeDma()
{
  DMA_U1RX::enableClock();

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
  DMA_U1RX::setMemory0Address(&input);
  DMA_U1RX::setPeripheralAddress(&USART1_REGS->DR);
  DMA_U1RX::setNumberOfTransactions(sizeof(input));
  DMA_U1RX::unmaskInterrupts();
  DMA_U1RX::enablePeripheral();
}

void initializeMarg()
{
  Accelerometer::configure(
      lsm303dlhc::accelerometer::ctrl1::lpen::NORMAL_MODE,
      lsm303dlhc::accelerometer::ctrl1::odr::NORMAL_DATA_RATE_1344HZ,
      lsm303dlhc::accelerometer::ctrl1::xen::X_AXIS_ENABLED,
      lsm303dlhc::accelerometer::ctrl1::yen::Y_AXIS_ENABLED,
      lsm303dlhc::accelerometer::ctrl1::zen::Z_AXIS_ENABLED,
      lsm303dlhc::accelerometer::ctrl4::sim::SPI_3_WIRE_INTERFACE,
      lsm303dlhc::accelerometer::ctrl4::hr::HIGH_RESOLUTION_ENABLED,
      lsm303dlhc::accelerometer::ctrl4::fs::RANGE_PLUS_MINUS_2G,
      lsm303dlhc::accelerometer::ctrl4::ble::LITTLE_ENDIAN_DATA_FORMAT,
      lsm303dlhc::accelerometer::ctrl4::bdu::CONTINUOUS_UPDATE);

  Gyroscope::configure(
      l3gd20::ctrl1::xen::X_AXIS_ENABLED,
      l3gd20::ctrl1::yen::Y_AXIS_ENABLED,
      l3gd20::ctrl1::zen::Z_AXIS_ENABLED,
      l3gd20::ctrl1::pd::NORMAL_MODE,
      l3gd20::ctrl1::bw_odr::DATA_RATE_760HZ_CUTOFF_30,
      l3gd20::ctrl4::sim::SPI_3_WIRE_INTERFACE,
      l3gd20::ctrl4::fs::SCALE_250_DPS,
      l3gd20::ctrl4::ble::LITTLE_ENDIAN_FORMAT,
      l3gd20::ctrl4::bdu::CONTINUOUS_UPDATE);

  Magnetometer::setMode(
      lsm303dlhc::magnetometer::mr::md::CONTINOUS_CONVERSION);
  Magnetometer::setReadingRange(
      lsm303dlhc::magnetometer::crb::gn::PLUS_MINUS_1_DOT_3_GAUSS);
  Magnetometer::setDataRate(
      lsm303dlhc::magnetometer::cra::do_::_220_HZ);
}

void initializePeripherals()
{
  initializeGpio();
  initializeUsart();
  initializeI2c();
  initializeTimer();
  initializeDma();
  initializeMarg();
}

void loop()
{
  if (go) {
    go = false;

    reading.M.X.byte[0] = Magnetometer::readXLow();
    reading.M.X.byte[1] = Magnetometer::readXHigh();
    reading.M.Y.byte[0] = Magnetometer::readYLow();
    reading.M.Y.byte[1] = Magnetometer::readYHigh();
    reading.M.Z.byte[0] = Magnetometer::readZLow();
    reading.M.Z.byte[1] = Magnetometer::readZHigh();

    reading.AR.X.byte[0] = Gyroscope::readXLow();
    reading.AR.X.byte[1] = Gyroscope::readXHigh();
    reading.AR.Y.byte[0] = Gyroscope::readYLow();
    reading.AR.Y.byte[1] = Gyroscope::readYHigh();
    reading.AR.Z.byte[0] = Gyroscope::readZLow();
    reading.AR.Z.byte[1] = Gyroscope::readZHigh();

    reading.G.X.byte[0] = Accelerometer::readXLow();
    reading.G.X.byte[1] = Accelerometer::readXHigh();
    reading.G.Y.byte[0] = Accelerometer::readYLow();
    reading.G.Y.byte[1] = Accelerometer::readYHigh();
    reading.G.Z.byte[0] = Accelerometer::readZLow();
    reading.G.Z.byte[1] = Accelerometer::readZHigh();

    printf("0x%04X\t0x%04X\t0x%04X\t"
        "0x%04X\t0x%04X\t0x%04X\t"
        "0x%04X\t0x%04X\t0x%04X\n\r",
        u16(reading.M.X.value),
        u16(reading.M.Y.value),
        u16(reading.M.Z.value),
        u16(reading.AR.X.value),
        u16(reading.AR.Y.value),
        u16(reading.AR.Z.value),
        u16(reading.G.X.value),
        u16(reading.G.Y.value),
        u16(reading.G.Z.value));
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

void interrupt::TIM6_DAC()
{
  TIM6::clearUpdateFlag();

  go = true;
}

void interrupt::DMA2_Stream5()
{
  DMA_U1RX::clearTransferCompleteFlag();

  switch (input) {
    case 'S':
      LED::setHigh();
      TIM6::startCounter();
      break;

    case 'E':
      LED::setLow();
      TIM6::stopCounter();
      break;
  }
}
