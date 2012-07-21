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
      static INLINE void configure();

      template<
          i2c::registers::ccr::bits::f_s::states::E,
          i2c::registers::ccr::bits::duty::states::E,
          u32 FREQUENCY_HZ
      >
      static INLINE void configureClock();

      static INLINE void enable();
      static INLINE void disable();
      static INLINE void sendStart();
      static INLINE void sendStop();
      static INLINE void sendData(u8 const data);
      static INLINE u8 getData();

      template<operation::E op>
      static INLINE void sendAddress(u8 const add);

      static INLINE void enableACK();
      static INLINE void disableACK();
      static INLINE bool hasSentStart();
      static INLINE bool hasSentStop();
      static INLINE bool hasAddressTransmitted();
      static INLINE bool hasReceivedData();
      static INLINE bool canSendData();
      static INLINE bool hasTranferFinished();
      static INLINE bool isTheBusBusy();
      static void writeSlaveRegister(u8 const, u8 const, u8 const);
      static u8 readSlaveRegister(u8 const, u8 const);

      // TODO I2C state machine functions

    private:
      Standard();
  };
}

// High-level access to the peripheral
typedef i2c::Standard<i2c::address::I2C1> I2C1;
typedef i2c::Standard<i2c::address::I2C2> I2C2;

#include "../../bits/i2c.tcc"
