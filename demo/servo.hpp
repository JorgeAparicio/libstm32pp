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
#include "core/nvic.hpp"
#include "driver/servo.hpp"
#include "peripheral/gpio.hpp"

// TODO Test SERVO demo on STM32F1XX

typedef servo::Functions<
    tim::address::TIM6,
    50,  // Hz
    tim::address::TIM7,
    1500,  // us
    8
> Servo;

template<> u8 Servo::servoIndex = 0;

template<> u32* Servo::pin[8] = {
    (u32*) PA0::OUT_ADDRESS,
    (u32*) PA1::OUT_ADDRESS,
    (u32*) PA2::OUT_ADDRESS,
    (u32*) PA3::OUT_ADDRESS,
    (u32*) PA4::OUT_ADDRESS,
    (u32*) PA5::OUT_ADDRESS,
    (u32*) PA6::OUT_ADDRESS,
    (u32*) PA7::OUT_ADDRESS,
};

template<> s16 Servo::value[8] = {
    -400,
    -300,
    -200,
    -100,
    100,
    200,
    300,
    400
};

template<> u8 Servo::sortedIndices[8] = { };

void mcuSetup()
{
#ifdef STM32F1XX
  RCC::enableClocks<
  rcc::apb2enr::IOPA
  >();

  GPIOA::configureLowerPins<
  gpio::registers::cr::states::GP_PUSH_PULL_2MHZ, /* 0 */
  gpio::registers::cr::states::GP_PUSH_PULL_2MHZ, /* 1 */
  gpio::registers::cr::states::GP_PUSH_PULL_2MHZ, /* 2 */
  gpio::registers::cr::states::GP_PUSH_PULL_2MHZ, /* 3 */
  gpio::registers::cr::states::GP_PUSH_PULL_2MHZ, /* 4 */
  gpio::registers::cr::states::GP_PUSH_PULL_2MHZ, /* 5 */
  gpio::registers::cr::states::GP_PUSH_PULL_2MHZ, /* 6 */
  gpio::registers::cr::states::GP_PUSH_PULL_2MHZ /* 7 */
  >();

#else
  RCC::enableClocks<
      rcc::ahb1enr::GPIOA
  >();

  GPIOA::setModes<
      gpio::registers::moder::states::OUTPUT, /* 0 */
      gpio::registers::moder::states::OUTPUT, /* 1 */
      gpio::registers::moder::states::OUTPUT, /* 2 */
      gpio::registers::moder::states::OUTPUT, /* 3 */
      gpio::registers::moder::states::OUTPUT, /* 4 */
      gpio::registers::moder::states::OUTPUT, /* 5 */
      gpio::registers::moder::states::OUTPUT, /* 6 */
      gpio::registers::moder::states::OUTPUT, /* 7 */
      gpio::registers::moder::states::INPUT, /* 8 */
      gpio::registers::moder::states::INPUT, /* 9 */
      gpio::registers::moder::states::INPUT, /* 10 */
      gpio::registers::moder::states::INPUT, /* 11 */
      gpio::registers::moder::states::INPUT, /* 12 */
      gpio::registers::moder::states::ALTERNATE, /* 13: JTAG PIN! */
      gpio::registers::moder::states::ALTERNATE, /* 14: JTAG PIN! */
      gpio::registers::moder::states::ALTERNATE /* 15: JTAG PIN! */
  >();
#endif

  RCC::enableClocks<
      rcc::apb1enr::TIM6,
      rcc::apb1enr::TIM7
  >();

#if defined VALUE_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX
  NVIC::enableInterrupt<
      nvic::irqn::TIM6_DAC
  >();
#else
  NVIC::enableInterrupt<
  nvic::irqn::TIM6
  >();
#endif

  NVIC::enableInterrupt<
      nvic::irqn::TIM7
  >();

  Servo::initialize();
  Servo::start();
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

#if defined VALUE_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX
void interrupt::TIM6_DAC()
{
  Servo::onPeriodTimerInterrupt();
}
#else
void interrupt::TIM6()
{
  Servo::onPeriodTimerInterrupt();
}
#endif

void interrupt::TIM7()
{
  Servo::onDutyCycleTimerInterrupt();
}
