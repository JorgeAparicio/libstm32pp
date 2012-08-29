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
 *                      L3GD20: Three axis MEMS gyroscope
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"
#include "../defs.hpp"

#include "../../memorymap/l3gd20.hpp"
#include "../include/peripheral/i2c.hpp"

// High-level functions
namespace l3gd20 {
  template<
      i2c::Address I,
      Address A
  >
  class Functions {
    public:
      static void configure(
          l3gd20::ctrl1::xen::States,
          l3gd20::ctrl1::yen::States,
          l3gd20::ctrl1::zen::States,
          l3gd20::ctrl1::pd::States,
          l3gd20::ctrl1::bw_odr::States,
          l3gd20::ctrl4::sim::States,
          l3gd20::ctrl4::fs::States,
          l3gd20::ctrl4::ble::States,
          l3gd20::ctrl4::bdu::States);

      static inline u8 readXLow();
      static inline u8 readXHigh();
      static inline u8 readYLow();
      static inline u8 readYHigh();
      static inline u8 readZLow();
      static inline u8 readZHigh();

    private:
      Functions();
  };
}  // namespace l3gd20

#include "../../bits/l3gd20.tcc"
