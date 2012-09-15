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

#include "peripheral/usart.hpp"

typedef USART6 USART;
typedef PC6 TX;
typedef PC7 RX;

#include "peripheral/dma.hpp"

typedef DMA2_STREAM7 DMA_U6TX;
typedef DMA2_STREAM2 DMA_U6RX;

#include "peripheral/i2c.hpp"

typedef I2C1 I2C;
typedef PB6 SCL;
typedef PB7 SDA;

#include "peripheral/tim.hpp"

#include "driver/lsm303dlhc.hpp"

typedef lsm303dlhc::accelerometer::Functions<i2c::I2C1> Accelerometer;
typedef lsm303dlhc::magnetometer::Functions<i2c::I2C1> Magnetometer;

#include "driver/l3gd20.hpp"

typedef l3gd20::Functions<i2c::I2C1, l3gd20::GYRO2> Gyroscope;

#include "driver/servo.hpp"

#define NUMBER_OF_SERVOS 20

servo::Functions<
    tim::TIM6,
    50,  // Hz
    tim::TIM7,
    1500,  // us
    NUMBER_OF_SERVOS
> Servo;

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

volatile bool go = false;

struct __attribute__ ((packed)) {
  u8 command;
  s16 angles[NUMBER_OF_SERVOS];
} inputBuffer;

struct {
    struct {
        union {
            s16 value;
            u8 byte[2];
        } x;
        union {
            s16 value;
            u8 byte[2];
        } y;
        union {
            s16 value;
            u8 byte[2];
        } z;
    } m;
    struct {
        union {
            s16 value;
            u8 byte[2];
        } x;
        union {
            s16 value;
            u8 byte[2];
        } y;
        union {
            s16 value;
            u8 byte[2];
        } z;
    } ar;
    struct {
        union {
            s16 value;
            u8 byte[2];
        } x;
        union {
            s16 value;
            u8 byte[2];
        } y;
        union {
            s16 value;
            u8 byte[2];
        } z;
    } g;
} outputBuffer;

void configure()
{
  // GPIO CONFIGURATION
  GPIOA::enableClock();
  GPIOB::enableClock();
  GPIOC::enableClock();
  GPIOD::enableClock();
  GPIOE::enableClock();

  // SERVO CONFIGURATION
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

  Servo.setPin(0, (u32*) S1::OUT_ADDRESS);
  Servo.setPin(1, (u32*) S2::OUT_ADDRESS);
  Servo.setPin(2, (u32*) S3::OUT_ADDRESS);
  Servo.setPin(3, (u32*) S4::OUT_ADDRESS);
  Servo.setPin(4, (u32*) S5::OUT_ADDRESS);
  Servo.setPin(5, (u32*) S6::OUT_ADDRESS);
  Servo.setPin(6, (u32*) S7::OUT_ADDRESS);
  Servo.setPin(7, (u32*) S8::OUT_ADDRESS);
  Servo.setPin(8, (u32*) S9::OUT_ADDRESS);
  Servo.setPin(9, (u32*) S10::OUT_ADDRESS);
  Servo.setPin(10, (u32*) S11::OUT_ADDRESS);
  Servo.setPin(11, (u32*) S12::OUT_ADDRESS);
  Servo.setPin(12, (u32*) S13::OUT_ADDRESS);
  Servo.setPin(13, (u32*) S14::OUT_ADDRESS);
  Servo.setPin(14, (u32*) S15::OUT_ADDRESS);
  Servo.setPin(15, (u32*) S16::OUT_ADDRESS);
  Servo.setPin(16, (u32*) S17::OUT_ADDRESS);
  Servo.setPin(17, (u32*) S18::OUT_ADDRESS);
  Servo.setPin(18, (u32*) S19::OUT_ADDRESS);
  Servo.setPin(19, (u32*) S20::OUT_ADDRESS);

  Servo.initialize();

  // USART CONFIGURATION
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
      usart::cr3::dmar::RECEIVER_DMA_ENABLED,
      usart::cr3::dmat::TRANSMITTER_DMA_ENABLED,
      usart::cr3::rtse::RTS_HARDWARE_FLOW_DISABLED,
      usart::cr3::ctse::CTS_HARDWARE_FLOW_DISABLED,
      usart::cr3::ctsie::CTS_INTERRUPT_DISABLED,
      usart::cr3::onebit::ONE_SAMPLE_BIT_METHOD);

  USART::setBaudRate<
      921600 /* bps */
  >();

  // I2C CONFIGURATION
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
  DMA_U6TX::setMemory0Address(&outputBuffer);
  DMA_U6TX::setPeripheralAddress(&USART6_REGS->DR);

  DMA_U6RX::enableClock();
  DMA_U6RX::configure(
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
      dma::stream::cr::pincos::PERIPHERAL_INCREMENT_OFFSET_SIZE_32BITS,
      dma::stream::cr::pl::PRIORITY_LEVEL_MEDIUM,
      dma::stream::cr::dbm::DOUBLE_BUFFER_MODE_DISABLED,
      dma::stream::cr::ct::CURRENT_TARGET_MEMORY_0,
      dma::stream::cr::pburst::PERIPHERAL_BURST_TRANSFER_SINGLE,
      dma::stream::cr::mburst::MEMORY_BURST_TRANSFER_SINGLE,
      dma::stream::cr::chsel::CHANNEL_5);
  DMA_U6RX::setMemory0Address(&inputBuffer);
  DMA_U6RX::setPeripheralAddress(&USART6_REGS->DR);
  DMA_U6RX::setNumberOfTransactions(sizeof(inputBuffer));
  DMA_U6RX::enableGlobalInterrupts();
  DMA_U6RX::enablePeripheral();

  // TIMER CONFIGURATION
  TIM2::enableClock();
  TIM2::configurePeriodicInterrupt<
      1000 /* Hz */
  >();
}

void loop()
{
  if (go) {
    go = false;

    outputBuffer.g.x.byte[0] = Accelerometer::readXLow();
    outputBuffer.g.x.byte[1] = Accelerometer::readXHigh();
    outputBuffer.g.y.byte[0] = Accelerometer::readYLow();
    outputBuffer.g.y.byte[1] = Accelerometer::readYHigh();
    outputBuffer.g.z.byte[0] = Accelerometer::readZLow();
    outputBuffer.g.z.byte[1] = Accelerometer::readZHigh();

    outputBuffer.ar.x.byte[0] = Gyroscope::readXLow();
    outputBuffer.ar.x.byte[1] = Gyroscope::readXHigh();
    outputBuffer.ar.y.byte[0] = Gyroscope::readYLow();
    outputBuffer.ar.y.byte[1] = Gyroscope::readYHigh();
    outputBuffer.ar.z.byte[0] = Gyroscope::readZLow();
    outputBuffer.ar.z.byte[1] = Gyroscope::readZHigh();

    outputBuffer.m.x.byte[0] = Magnetometer::readXLow();
    outputBuffer.m.x.byte[1] = Magnetometer::readXHigh();
    outputBuffer.m.y.byte[0] = Magnetometer::readYLow();
    outputBuffer.m.y.byte[1] = Magnetometer::readYHigh();
    outputBuffer.m.z.byte[0] = Magnetometer::readZLow();
    outputBuffer.m.z.byte[1] = Magnetometer::readZHigh();

    DMA_U6TX::clearTransferCompleteFlag();
    DMA_U6TX::setNumberOfTransactions(sizeof(outputBuffer));
    DMA_U6TX::enablePeripheral();
  }
}

int main()
{
  clk::initialize();

  configure();

  while (true) {
    loop();
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

void interrupt::TIM2()
{
  TIM2::clearUpdateFlag();

  go = true;
}

void interrupt::DMA2_Stream2()
{
  DMA_U6RX::clearTransferCompleteFlag();

  switch (inputBuffer.command) {
    case 'R': // On/off report
      if (TIM2::isCounting())
        TIM2::stopCounter();
      else
        TIM2::startCounter();
      break;
    case 'S': // On/off Servo
      if (Servo.isActive())
        Servo.stop();
      else
        Servo.start();
      break;
    case 'M': // Manual control
      Servo.load(inputBuffer.angles);
      break;
  }
}
