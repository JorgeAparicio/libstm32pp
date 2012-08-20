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

#include "common.hpp"

namespace gpio {
  enum Address {
#ifdef STM32F1XX
    GPIOA = alias::APB2 + 0x0800,
    GPIOB = alias::APB2 + 0x0C00,
    GPIOC = alias::APB2 + 0x1000,
    GPIOD = alias::APB2 + 0x1400,
    GPIOE = alias::APB2 + 0x1800,
    GPIOF = alias::APB2 + 0x1C00,
    GPIOG = alias::APB2 + 0x2000,
#else  // STM32F1XX
    GPIOA = alias::AHB1 + 0x0000,
    GPIOB = alias::AHB1 + 0x0400,
    GPIOC = alias::AHB1 + 0x0800,
    GPIOD = alias::AHB1 + 0x0C00,
    GPIOE = alias::AHB1 + 0x1000,
    GPIOF = alias::AHB1 + 0x1400,
    GPIOG = alias::AHB1 + 0x1800,
    GPIOH = alias::AHB1 + 0x1C00,
    GPIOI = alias::AHB1 + 0x2000,
#endif  // STM32F1XX
  };

  struct Registers {
#ifdef STM32F1XX
      __RW
      u32 CR[2];   // 0x00: Configuration
      __RW
      u32 IDR;// 0x08: Input data
      __RW
      u32 ODR;// 0x0C: Output data
      __RW
      u32 BSRR;// 0x10: Bit set/reset
      __RW
      u32 BRR;// 0x14: Bit reset
      __RW
      u32 LCKR;// 0x18: Configuration lock
#else // STM32F1XX
      __RW
      u32 MODER;   // 0x00: Mode
      __RW
      u32 OTYPER;   // 0x04: Output type
      __RW
      u32 OSPEEDR;   // 0x08: Output speed
      __RW
      u32 PUPDR;   // 0x0C: Pull-up/pull-down
      __RW
      u32 IDR;   // 0x10: Input data
      __RW
      u32 ODR;   // 0x14: Output data
      __RW
      u32 BSRR;   // 0x18: Bit set/reset
      __RW
      u32 LCKR;   // 0x1C: Configuration lock
      __RW
      u32 AFR[2];   // 0x20: Alternate function
#endif  // STM32F1XX
  };

#ifdef STM32F1XX
  namespace crl {
    enum {
      OFFSET = 0x00
    };
  }  // namespace crl

  namespace crh {
    enum {
      OFFSET = 0x04
    };
  }  // namespace crh

  namespace cr {
    enum {
      MASK = 0b1111,
      POSITION = 4
    };
    enum States {
      ANALOG_INPUT = 0,
      GP_PUSH_PULL_10MHZ = 1,
      GP_PUSH_PULL_2MHZ = 2,
      GP_PUSH_PULL_50MHZ = 3,
      FLOATING_INPUT = 4,
      GP_OPEN_DRAIN_10MHZ = 5,
      GP_OPEN_DRAIN_2MHZ = 6,
      GP_OPEN_DRAIN_50MHZ = 7,
      INPUT_PULL_X = 8,
      AF_PUSH_PULL_10MHZ = 9,
      AF_PUSH_PULL_2MHZ = 10,
      AF_PUSH_PULL_50MHZ = 11,
      AF_OPEN_DRAIN_10MHZ = 13,
      AF_OPEN_DRAIN_2MHZ = 14,
      AF_OPEN_DRAIN_50MHZ = 15,
    };
  }  // namespace cr

  namespace idr {
    enum {
      OFFSET = 0x08
    };
  }  // namespace idr

  namespace odr {
    enum {
      OFFSET = 0x0C
    };
  }  // namespace odr

  namespace bsrr {
    enum {
      OFFSET = 0x10
    };
  }  // namespace bsrr

  namespace brr {
    enum {
      OFFSET = 0x14
    };
  }  // namespace brr

  namespace lckr {
    enum {
      OFFSET = 0x18
    };
  }  // namespace lckr
#else  // STM32F1XX
  namespace moder {
    enum {
      OFFSET = 0x00,
      MASK = 0b11
    };
    enum States {
      INPUT = 0,
      OUTPUT = 1,
      ALTERNATE = 2,
      ANALOG = 3,
    };
  }  // namespace moder

  namespace otyper {
    enum {
      OFFSET = 0x04,
      MASK = 0b1
    };
    enum States {
      PUSH_PULL = 0,
      OPEN_DRAIN = 1,
    };
  }  // namespace otyper

  namespace ospeedr {
    enum {
      OFFSET = 0x08,
      MASK = 0b11
    };
    enum States {
      _2MHZ = 0,
      _25MHZ = 1,
      _50MHZ = 2,
      _100MHZ = 3,
    };
  }  // namespace ospeedr

  namespace pupdr {
    enum {
      POSITION = 0x0C,
      MASK = 0b11
    };
    enum States {
      FLOATING = 0,
      PULL_UP = 1,
      PULL_DOWN = 2,
    };
  }  // namespace pupdr

  namespace idr {
    enum {
      OFFSET = 0x10
    };
  }  // namespace idr

  namespace odr {
    enum {
      OFFSET = 0x14
    };
  }  // namespace odr

  namespace bsrr {
    enum {
      OFFSET = 0x18
    };
  }  // namespace bsrr

  namespace lckr {
    enum {
      OFFSET = 0x1C
    };
  }  // namespace lckr

  namespace afrl {
    enum {
      OFFSET = 0x20
    };
  }  // namespace afrl

  namespace afrh {
    enum {
      OFFSET = 0x24
    };
  }  // namespace afrh

  namespace afr {
    enum {
      MASK = 0b1111,
      POSITION = 4
    };
    enum States {
      SYSTEM = 0,
      TIM1_2 = 1,
      TIM3_5 = 2,
      TIM8_11 = 3,
      I2C = 4,
      SPI1_2 = 5,
      SPI3 = 6,
      USART1_3 = 7,
      USART4_6 = 8,
      CAN1_2 = 9,
      TIM12_14 = 9,
      OTG_FS = 10,
      OTG_HS = 10,
      ETH = 11,
      FSMC = 12,
      SDIO = 12,
      DCMI = 13,
      EVENTOUT = 15,
    };
  }  // namespace afr
#endif  // STM32F1XX
}  // namespace gpio
