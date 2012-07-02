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
      i2c::address::E I,
      address::E A
  >
  class Gyroscope {
    public:

      template<
          l3gd20::registers::ctrl_reg1::bits::xen::states::E,
          l3gd20::registers::ctrl_reg1::bits::yen::states::E,
          l3gd20::registers::ctrl_reg1::bits::zen::states::E,
          l3gd20::registers::ctrl_reg1::bits::pd::states::E,
          l3gd20::registers::ctrl_reg1::bits::bw_odr::states::E,
          l3gd20::registers::ctrl_reg4::bits::sim::states::E,
          l3gd20::registers::ctrl_reg4::bits::fs::states::E,
          l3gd20::registers::ctrl_reg4::bits::ble::states::E,
          l3gd20::registers::ctrl_reg4::bits::bdu::states::E
      >
      static void configure();

      static INLINE u8 readXLow();
      static INLINE u8 readXHigh();
      static INLINE u8 readYLow();
      static INLINE u8 readYHigh();
      static INLINE u8 readZLow();
      static INLINE u8 readZHigh();

    private:
      Gyroscope();
  };
}  // namespace l3gd20

#include "../../bits/l3gd20.tcc"
