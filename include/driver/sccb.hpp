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

#include "../device_select.hpp"
#include "../defs.hpp"

#include "../peripheral/gpio.hpp"
#include "../peripheral/tim.hpp"

namespace sccb {
  template<
      gpio::Address SDIO_C_PORT,
      u8 SDIO_C_PIN,
      gpio::Address SDIO_D_PORT,
      u8 SDIO_D_PIN,
      tim::Address DELAY_TIMER_ADDRESS,
      u32 FREQUENCY
  >
  class Functions {
      enum {
        DELAY = 1000000 / (2 * FREQUENCY)
      };

      static_assert(SDIO_C_PIN < 16, "Invalid GPIO pin.");
      static_assert(SDIO_D_PIN < 16, "Invalid GPIO pin.");
      static_assert(DELAY != 0, "Can't reach this frequency.");

      typedef gpio::Pin<SDIO_C_PORT, SDIO_C_PIN> SDIO_C;
      typedef gpio::Pin<SDIO_D_PORT, SDIO_D_PIN> SDIO_D;
      typedef tim::Functions<DELAY_TIMER_ADDRESS> TIMER;

    public:
      static inline void initialize();
      static void sendStart();
      static void sendStop();
      static void sendNACK();
      static bool sendByte(u8 const);
      static u8 getByte();
      static bool writeSlaveRegister(
          u8 const slaveAddress,
          u8 const registerAddress,
          u8 const value);
      static bool readSlaveRegister(
          u8 const slaveAddress,
          u8 const registerAddress,
          u8& value);

    private:
      Functions();
  };
}

#include "../../bits/sccb.tcc"
