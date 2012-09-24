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

#define UART_BAUD_RATE 921600
#define MARG_SAMPLE_RATE 100
#define NUMBER_OF_SERVOS 19

#include "clock.hpp"

#include "interrupt.hpp"

#include "peripheral/gpio.hpp"

typedef PB10 U3TX;
typedef PB11 U3RX;

typedef PB8 SCL;
typedef PB9 SDA;

typedef PC13 Servo1;
typedef PC0 Servo2;
typedef PC2 Servo3;
typedef PA0 Servo4;
typedef PA2 Servo5;
typedef PA5 Servo6;
typedef PA7 Servo7;
typedef PC5 Servo8;
typedef PB1 Servo9;
typedef PE7 Servo10;
typedef PE9 Servo11;
typedef PE11 Servo12;
typedef PE13 Servo13;
typedef PE15 Servo14;
//typedef PB11 Servo15; // Crash!
typedef PB12 Servo16;
typedef PB14 Servo17;
typedef PD8 Servo18;
typedef PD10 Servo19;
typedef PD12 Servo20;

#include "peripheral/usart.hpp"

#include "peripheral/dma.hpp"

typedef DMA1_STREAM3 DMA_U3TX;
typedef DMA1_STREAM1 DMA_U3RX;

#include "peripheral/i2c.hpp"

#include "peripheral/tim.hpp"

#include "driver/lsm303dlhc.hpp"

typedef lsm303dlhc::accelerometer::Functions<i2c::I2C1> Accelerometer;
typedef lsm303dlhc::magnetometer::Functions<i2c::I2C1> Magnetometer;

#include "driver/l3gd20.hpp"

typedef l3gd20::Functions<i2c::I2C1, l3gd20::L3G4200D_1> Gyroscope;

#include "driver/servo.hpp"

servo::Functions<
    tim::TIM6,
    50,  // Hz
    tim::TIM7,
    1500,  // us
    NUMBER_OF_SERVOS
> ServoController;

bool go = false;

struct __attribute__ ((packed)) Input {
  u8 command;
  s16 angles[NUMBER_OF_SERVOS];
};

Input input;

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

void initializeGpio()
{
  GPIOA::enableClock();
  GPIOB::enableClock();
  GPIOC::enableClock();
  GPIOD::enableClock();
  GPIOE::enableClock();

  U3TX::setAlternateFunction(gpio::afr::USART1_3);
  U3TX::setMode(gpio::moder::ALTERNATE);
  U3RX::setAlternateFunction(gpio::afr::USART1_3);
  U3RX::setMode(gpio::moder::ALTERNATE);

  SCL::setAlternateFunction(gpio::afr::I2C);
  SCL::setPullMode(gpio::pupdr::PULL_UP);
  SCL::setMode(gpio::moder::ALTERNATE);
  SDA::setAlternateFunction(gpio::afr::I2C);
  SDA::setPullMode(gpio::pupdr::PULL_UP);
  SDA::setMode(gpio::moder::ALTERNATE);

  Servo1::setMode(gpio::moder::OUTPUT);
  Servo2::setMode(gpio::moder::OUTPUT);
  Servo3::setMode(gpio::moder::OUTPUT);
  Servo4::setMode(gpio::moder::OUTPUT);
  Servo5::setMode(gpio::moder::OUTPUT);
  Servo6::setMode(gpio::moder::OUTPUT);
  Servo7::setMode(gpio::moder::OUTPUT);
  Servo8::setMode(gpio::moder::OUTPUT);
  Servo9::setMode(gpio::moder::OUTPUT);
  Servo10::setMode(gpio::moder::OUTPUT);
  Servo11::setMode(gpio::moder::OUTPUT);
  Servo12::setMode(gpio::moder::OUTPUT);
  Servo13::setMode(gpio::moder::OUTPUT);
  Servo14::setMode(gpio::moder::OUTPUT);
//  Servo15::setMode(gpio::moder::OUTPUT);  // Crash!
  Servo16::setMode(gpio::moder::OUTPUT);
  Servo17::setMode(gpio::moder::OUTPUT);
  Servo18::setMode(gpio::moder::OUTPUT);
  Servo19::setMode(gpio::moder::OUTPUT);
  Servo20::setMode(gpio::moder::OUTPUT);
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
  TIM2::enableClock();
  TIM2::configurePeriodicInterrupt<
      1000 /* Hz */
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
  DMA_U3TX::setMemory0Address(&reading);
  DMA_U3TX::setPeripheralAddress(&USART3_REGS->DR);
  DMA_U3TX::setNumberOfTransactions(sizeof(reading));
  DMA_U3RX::configure(
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
  DMA_U3RX::setMemory0Address(&input);
  DMA_U3RX::setPeripheralAddress(&USART3_REGS->DR);
  DMA_U3RX::setNumberOfTransactions(sizeof(input));
  DMA_U3RX::unmaskInterrupts();
  DMA_U3RX::enablePeripheral();
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

void initializeServoController()
{
  ServoController.setPin(0, (u32*) Servo1::OUT_ADDRESS);
  ServoController.setPin(1, (u32*) Servo2::OUT_ADDRESS);
  ServoController.setPin(2, (u32*) Servo3::OUT_ADDRESS);
  ServoController.setPin(3, (u32*) Servo4::OUT_ADDRESS);
  ServoController.setPin(4, (u32*) Servo5::OUT_ADDRESS);
  ServoController.setPin(5, (u32*) Servo6::OUT_ADDRESS);
  ServoController.setPin(6, (u32*) Servo7::OUT_ADDRESS);
  ServoController.setPin(7, (u32*) Servo8::OUT_ADDRESS);
  ServoController.setPin(8, (u32*) Servo9::OUT_ADDRESS);
  ServoController.setPin(9, (u32*) Servo10::OUT_ADDRESS);
  ServoController.setPin(10, (u32*) Servo11::OUT_ADDRESS);
  ServoController.setPin(11, (u32*) Servo12::OUT_ADDRESS);
  ServoController.setPin(12, (u32*) Servo13::OUT_ADDRESS);
  ServoController.setPin(13, (u32*) Servo14::OUT_ADDRESS);
//  ServoController.setPin(14, (u32*) Servo15::OUT_ADDRESS);  // Crash!
  ServoController.setPin(14, (u32*) Servo16::OUT_ADDRESS);
  ServoController.setPin(15, (u32*) Servo17::OUT_ADDRESS);
  ServoController.setPin(16, (u32*) Servo18::OUT_ADDRESS);
  ServoController.setPin(17, (u32*) Servo19::OUT_ADDRESS);
  ServoController.setPin(18, (u32*) Servo20::OUT_ADDRESS);

  ServoController.initialize();
}

void initializePeripherals()
{
  initializeGpio();
  initializeUsart();
  initializeI2c();
  initializeTimer();
  initializeDma();
  initializeServoController();
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

    DMA_U3TX::clearTransferCompleteFlag();
    DMA_U3TX::setNumberOfTransactions(sizeof(reading));
    DMA_U3TX::enablePeripheral();
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
  ServoController.onPeriodTimerInterrupt();
}

void interrupt::TIM7()
{
  ServoController.onDutyCycleTimerInterrupt();
}

void interrupt::TIM2()
{
  TIM2::clearUpdateFlag();

  go = true;
}

void interrupt::DMA1_Stream1()
{
  DMA_U3RX::clearTransferCompleteFlag();

  switch (input.command) {
    case 'R': // On/off report
      if (TIM2::isCounting())
        TIM2::stopCounter();
      else
        TIM2::startCounter();
      break;
    case 'S': // On/off ServoController
      if (ServoController.isActive())
        ServoController.stop();
      else
        ServoController.start();
      break;
    case 'M': // Manual control
      ServoController.load(input.angles);
      break;
  }
}
