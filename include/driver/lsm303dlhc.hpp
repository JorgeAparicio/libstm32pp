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
 *   LSM303DLHC: Three axis MEMS accelerometer + three axis MEMS magnetometer
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"
#include "../defs.hpp"

#include "../../memorymap/lsm303dlhc.hpp"

// High-level functions
namespace lsm303dlhc {
  template <
    i2c::address::E I
  >
  class Accelerometer {
    public:
      template <
        lsm303dlhc::registers::ctrl_reg1_a::bits::lpen::states::E,
        lsm303dlhc::registers::ctrl_reg1_a::bits::odr::states::E,
        lsm303dlhc::registers::ctrl_reg1_a::bits::xen::states::E,
        lsm303dlhc::registers::ctrl_reg1_a::bits::yen::states::E,
        lsm303dlhc::registers::ctrl_reg1_a::bits::zen::states::E,
        lsm303dlhc::registers::ctrl_reg4_a::bits::sim::states::E,
        lsm303dlhc::registers::ctrl_reg4_a::bits::hr::states::E,
        lsm303dlhc::registers::ctrl_reg4_a::bits::fs::states::E,
        lsm303dlhc::registers::ctrl_reg4_a::bits::ble::states::E,
        lsm303dlhc::registers::ctrl_reg4_a::bits::bdu::states::E
      >
      static void configure();

      static inline u8 readXLow();
      static inline u8 readXHigh();
      static inline u8 readYLow();
      static inline u8 readYHigh();
      static inline u8 readZLow();
      static inline u8 readZHigh();

    private:
      Accelerometer();
  };

  template <
    i2c::address::E I
  >
  class Magnetometer {
    public:
      template <
        lsm303dlhc::registers::cra_reg_m::bits::do_::states::E
      >
      static void setDataRate();

      template <
        lsm303dlhc::registers::crb_reg_m::bits::gn::states::E
      >
      static void setReadingRange();

      template <
        lsm303dlhc::registers::mr_reg_m::bits::md::states::E
      >
      static void setMode();

      static inline u8 readXLow();
      static inline u8 readXHigh();
      static inline u8 readYLow();
      static inline u8 readYHigh();
      static inline u8 readZLow();
      static inline u8 readZHigh();

    private:
      Magnetometer();
  };

  template <
    i2c::address::E I
  >
  class Thermometer {
    public:
      static void disable();
      static void enable();

    private:
      Thermometer();
  };
}  // namespace lsm303dlhc

// High-level access to the device
// TODO LSM303DLHC high-level access

#include "../../bits/lsm303dlhc.tcc"
