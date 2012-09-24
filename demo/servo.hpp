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
// Tested on STM32F4DISCOVERY
// Tested on F4Dev
#include "clock.hpp"

#include "interrupt.hpp"

#include "peripheral/gpio.hpp"

typedef PA0 Servo1;
typedef PA1 Servo2;
typedef PA2 Servo3;
typedef PA3 Servo4;

#include "driver/servo.hpp"

servo::Functions<
    tim::TIM6,
    50,  // Hz
    tim::TIM7,
    1500,  // us
    4
> Servo;

void initializeGpio()
{
#ifdef STM32F1XX
  Servo1::enableClock();
  Servo1::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);

  Servo2::enableClock();
  Servo2::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);

  Servo3::enableClock();
  Servo3::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);

  Servo4::enableClock();
  Servo4::setMode(gpio::cr::GP_PUSH_PULL_2MHZ);
#else
  Servo1::enableClock();
  Servo1::setMode(gpio::moder::OUTPUT);

  Servo2::enableClock();
  Servo2::setMode(gpio::moder::OUTPUT);

  Servo3::enableClock();
  Servo3::setMode(gpio::moder::OUTPUT);

  Servo4::enableClock();
  Servo4::setMode(gpio::moder::OUTPUT);
#endif
}

void initializeServoController()
{
  Servo.setPin(0, (u32*) (Servo1::OUT_ADDRESS));
  Servo.setPin(1, (u32*) (Servo2::OUT_ADDRESS));
  Servo.setPin(2, (u32*) (Servo3::OUT_ADDRESS));
  Servo.setPin(3, (u32*) (Servo4::OUT_ADDRESS));

  Servo.initialize();
}

void initializePeripherals()
{
  initializeGpio();
  initializeServoController();

  Servo.start();
}

void loop()
{
  // In debug mode: Modify the Servo.buffer variable.
}

int main(void)
{
  clk::initialize();

  initializePeripherals();

  while (true)
  {
    loop();
  }
}

#if defined VALUE_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX
void interrupt::TIM6_DAC()
#else
void interrupt::TIM6()
#endif
{
  Servo.onPeriodTimerInterrupt();
}

void interrupt::TIM7()
{
  Servo.onDutyCycleTimerInterrupt();
}
