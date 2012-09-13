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

#include "peripheral/gpio.hpp"

#include "peripheral/usart.hpp"

typedef USART6 USART;
typedef PC6 TX;
typedef PC7 RX;

#include "peripheral/dma.hpp"

typedef DMA2_STREAM7 DMA_U6TX;

#include "peripheral/i2c.hpp"

typedef I2C1 I2C;
typedef PB6 SCL;
typedef PB7 SDA;

#include "peripheral/tim.hpp"

#include "core/nvic.hpp"
#include "interrupt.hpp"

#include "driver/lsm303dlhc.hpp"

typedef lsm303dlhc::accelerometer::Functions<i2c::I2C1> Accelerometer;
typedef lsm303dlhc::magnetometer::Functions<i2c::I2C1> Magnetometer;

#include "driver/l3gd20.hpp"

typedef l3gd20::Functions<i2c::I2C1, l3gd20::GYRO2> Gyroscope;

volatile bool go = false;

union {
    s16 value;
    struct {
        u8 bytes[sizeof(s16)];
    };
} reading[9];

int main()
{
  clk::initialize();

  // USART CONFIGURATION
  GPIOC::enableClock();
  TX::setAlternateFunction(gpio::afr::USART4_6);
  TX::setMode(gpio::moder::ALTERNATE);
  RX::setAlternateFunction(gpio::afr::USART4_6);
  RX::setMode(gpio::moder::ALTERNATE);

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
      usart::cr3::dmar::RECEIVER_DMA_DISABLED,
      usart::cr3::dmat::TRANSMITTER_DMA_ENABLED,
      usart::cr3::rtse::RTS_HARDWARE_FLOW_DISABLED,
      usart::cr3::ctse::CTS_HARDWARE_FLOW_DISABLED,
      usart::cr3::ctsie::CTS_INTERRUPT_DISABLED,
      usart::cr3::onebit::ONE_SAMPLE_BIT_METHOD);

  USART::setBaudRate<
      921600 /* bps */
  >();

  // I2C CONFIGURATION
  GPIOB::enableClock();
  SCL::setAlternateFunction(gpio::afr::I2C);
  SCL::setPullMode(gpio::pupdr::PULL_UP);
  SCL::setMode(gpio::moder::ALTERNATE);

  SDA::setAlternateFunction(gpio::afr::I2C);
  SDA::setPullMode(gpio::pupdr::PULL_UP);
  SDA::setMode(gpio::moder::ALTERNATE);

  I2C::enableClock();
  I2C::configure(
      i2c::cr1::pe::PERIPHERAL_ENABLED,
      i2c::cr1::enpec::PACKET_ERROR_CHECKING_DISABLED,
      i2c::cr1::engc::GENERAL_CALL_DISABLED,
      i2c::cr1::nostretch::CLOCK_STRETCHING_DISABLED,
      i2c::cr2::iterren::ERROR_INTERRUPT_DISABLED,
      i2c::cr2::itevten::EVENT_INTERRUPT_DISABLED,
      i2c::cr2::itbufen::BUFFER_INTERRUPT_DISABLED,
      i2c::cr2::dmaen::DMA_REQUEST_DISABLED,
      i2c::cr2::last::NEXT_DMA_IS_NOT_THE_LAST_TRANSFER);

  I2C::configureClock<
      i2c::ccr::f_s::FAST_MODE,
      i2c::ccr::duty::T_LOW_2_T_HIGH_1,
      400000 /* Hz */
  >();

  // TIMER CONFIGURATION
  TIM6::enableClock();
  TIM6::configurePeriodicInterrupt<
      100 /* Hz */
  >();
  NVIC::enableInterrupt<
      nvic::irqn::TIM6_DAC
  >();

  // ACCELEROMETER CONFIGURATION
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

  // GYROSCOPE CONFIGURATION
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

  // MAGNETOMETER CONFIGURATION
  Magnetometer::setMode(
      lsm303dlhc::magnetometer::mr::md::CONTINOUS_CONVERSION);
  Magnetometer::setReadingRange(
      lsm303dlhc::magnetometer::crb::gn::PLUS_MINUS_1_DOT_3_GAUSS);
  Magnetometer::setDataRate(
      lsm303dlhc::magnetometer::cra::do_::_220_HZ);

  // DMA CONFIGURATION
  DMA_U6TX::enableClock();

  DMA_U6TX::configure(
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
      dma::stream::cr::pincos::PERIPHERAL_INCREMENT_OFFSET_SIZE_32BITS,
      dma::stream::cr::pl::PRIORITY_LEVEL_MEDIUM,
      dma::stream::cr::dbm::DOUBLE_BUFFER_MODE_DISABLED,
      dma::stream::cr::ct::CURRENT_TARGET_MEMORY_0,
      dma::stream::cr::pburst::PERIPHERAL_BURST_TRANSFER_SINGLE,
      dma::stream::cr::mburst::MEMORY_BURST_TRANSFER_SINGLE,
      dma::stream::cr::chsel::CHANNEL_5);

  DMA_U6TX::setMemory0Address(&reading);
  DMA_U6TX::setPeripheralAddress(&USART6_REGS->DR);

  TIM6::startCounter();

  while (true) {
    if (go) {
      go = false;

      reading[0].bytes[0] = Accelerometer::readXLow();
      reading[0].bytes[1] = Accelerometer::readXHigh();
      reading[1].bytes[0] = Accelerometer::readYLow();
      reading[1].bytes[1] = Accelerometer::readYHigh();
      reading[2].bytes[0] = Accelerometer::readZLow();
      reading[2].bytes[1] = Accelerometer::readZHigh();

      reading[3].bytes[0] = Gyroscope::readXLow();
      reading[3].bytes[1] = Gyroscope::readXHigh();
      reading[4].bytes[0] = Gyroscope::readYLow();
      reading[4].bytes[1] = Gyroscope::readYHigh();
      reading[5].bytes[0] = Gyroscope::readZLow();
      reading[5].bytes[1] = Gyroscope::readZHigh();

      reading[6].bytes[0] = Magnetometer::readXLow();
      reading[6].bytes[1] = Magnetometer::readXHigh();
      reading[7].bytes[0] = Magnetometer::readYLow();
      reading[7].bytes[1] = Magnetometer::readYHigh();
      reading[8].bytes[0] = Magnetometer::readZLow();
      reading[8].bytes[1] = Magnetometer::readZHigh();

      DMA_U6TX::clearTransferCompleteFlag();
      DMA_U6TX::setNumberOfTransactions(sizeof(reading));
      DMA_U6TX::enablePeripheral();
    }
  }
}

void interrupt::TIM6_DAC()
{
  TIM6::clearUpdateFlag();

  go = true;
}
