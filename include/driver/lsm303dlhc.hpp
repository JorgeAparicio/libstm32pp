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
#include "../include/peripheral/i2c.hpp"
#include "../../memorymap/lsm303dlhc.hpp"

// High-level functions
namespace lsm303dlhc {
  namespace accelerometer {
    template<i2c::Address I>
    class Functions {
      public:
        static void configure(
            lsm303dlhc::accelerometer::ctrl1::lpen::States,
            lsm303dlhc::accelerometer::ctrl1::odr::States,
            lsm303dlhc::accelerometer::ctrl1::xen::States,
            lsm303dlhc::accelerometer::ctrl1::yen::States,
            lsm303dlhc::accelerometer::ctrl1::zen::States,
            lsm303dlhc::accelerometer::ctrl4::sim::States,
            lsm303dlhc::accelerometer::ctrl4::hr::States,
            lsm303dlhc::accelerometer::ctrl4::fs::States,
            lsm303dlhc::accelerometer::ctrl4::ble::States,
            lsm303dlhc::accelerometer::ctrl4::bdu::States);

        static inline u8 readXLow();
        static inline u8 readXHigh();
        static inline u8 readYLow();
        static inline u8 readYHigh();
        static inline u8 readZLow();
        static inline u8 readZHigh();

      private:
        Functions();
    };
  } // namespace accelerometer

  namespace magnetometer {
    template<i2c::Address>
    class Functions {
      public:
        static void setDataRate(lsm303dlhc::magnetometer::cra::do_::States);
        static void setReadingRange(lsm303dlhc::magnetometer::crb::gn::States);
        static void setMode(lsm303dlhc::magnetometer::mr::md::States);
        static inline u8 readXLow();
        static inline u8 readXHigh();
        static inline u8 readYLow();
        static inline u8 readYHigh();
        static inline u8 readZLow();
        static inline u8 readZHigh();

      private:
        Functions();
    };
  }  // namespace magnetometer

  namespace thermometer {
    template<i2c::Address>
    class Functions {
      public:
        static void disable();
        static void enable();

      private:
        Functions();
    };
  }  // namespace thermometer
}  // namespace lsm303dlhc

#include "../../bits/lsm303dlhc.tcc"
