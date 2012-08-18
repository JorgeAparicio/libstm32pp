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

/*******************************************************************************
 *
 *                      Inter-integrated Circuit Interface
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"
#include "../defs.hpp"

#include "../clock.hpp"
#include "../../memorymap/i2c.hpp"

// Low-level access to the registers
#define _I2C1 reinterpret_cast<i2c::Registers*>(i2c::address::I2C1)
#define _I2C2 reinterpret_cast<i2c::Registers*>(i2c::address::I2C2)
#ifndef STM32F1XX
#define _I2C3 reinterpret_cast<i2c::Registers*>(i2c::address::I2C3)
#endif

// High-level functions
namespace i2c {
  template<address::E I>
  class Standard {
    public:
      enum {
        FREQUENCY = clock::APB1
      };

      template<
          i2c::registers::cr1::bits::pe::states::E,
          i2c::registers::cr1::bits::enpec::states::E,
          i2c::registers::cr1::bits::engc::states::E,
          i2c::registers::cr1::bits::nostretch::states::E,
          i2c::registers::cr2::bits::iterren::states::E,
          i2c::registers::cr2::bits::itevten::states::E,
          i2c::registers::cr2::bits::itbufen::states::E,
          i2c::registers::cr2::bits::dmaen::states::E,
          i2c::registers::cr2::bits::last::states::E
      >
      static inline void configure();

      template<
          i2c::registers::ccr::bits::f_s::states::E,
          i2c::registers::ccr::bits::duty::states::E,
          u32 FREQUENCY_HZ
      >
      static inline void configureClock();

      static inline void enableClock();
      static inline void disableClock();
      static inline void enablePeripheral();
      static inline void disablePeripheral();
      static inline void sendStart();
      static inline void sendStop();
      static inline void sendData(u8 const data);
      static inline u8 getData();

      template<operation::E op>
      static inline void sendAddress(u8 const add);

      static inline void enableACK();
      static inline void disableACK();
      static inline bool hasSentStart();
      static inline bool hasSentStop();
      static inline bool hasAddressTransmitted();
      static inline bool hasReceivedData();
      static inline bool canSendData();
      static inline bool hasTranferFinished();
      static inline bool isTheBusBusy();
      static void writeSlaveRegister(
          u8 const slaveAddress,
          u8 const registerAddress,
          u8 const value);
      static u8 readSlaveRegister(
          u8 const slaveAddress,
          u8 const registerAddress);

      // TODO I2C state machine functions

    private:
      Standard();
  };
}

// High-level access to the peripheral
typedef i2c::Standard<i2c::address::I2C1> I2C1;
typedef i2c::Standard<i2c::address::I2C2> I2C2;

#include "../../bits/i2c.tcc"
