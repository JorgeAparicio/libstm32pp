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
#define I2C1_REGS reinterpret_cast<i2c::Registers*>(i2c::I2C1)
#define I2C2_REGS reinterpret_cast<i2c::Registers*>(i2c::I2C2)
#ifndef STM32F1XX
#define I2C3_REGS reinterpret_cast<i2c::Registers*>(i2c::I2C3)
#endif

// High-level functions
namespace i2c {
  template<Address I>
  class Standard {
    public:
      enum {
        FREQUENCY = clock::APB1
      };

      static inline void configure(
          i2c::cr1::pe::States,
          i2c::cr1::enpec::States,
          i2c::cr1::engc::States,
          i2c::cr1::nostretch::States,
          i2c::cr2::iterren::States,
          i2c::cr2::itevten::States,
          i2c::cr2::itbufen::States,
          i2c::cr2::dmaen::States,
          i2c::cr2::last::States);

      template<
          i2c::ccr::f_s::States,
          i2c::ccr::duty::States,
          u32
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

      static inline void sendAddress(
          u8 const add,
          operation::E op);

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
typedef i2c::Standard<i2c::I2C1> I2C1;
typedef i2c::Standard<i2c::I2C2> I2C2;

#include "../../bits/i2c.tcc"
