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

namespace l3gd20 {
  template<
      i2c::Address I,
      Address A
  >
  void Functions<I, A>::configure(
      l3gd20::ctrl_reg1::xen::States XEN,
      l3gd20::ctrl_reg1::yen::States YEN,
      l3gd20::ctrl_reg1::zen::States ZEN,
      l3gd20::ctrl_reg1::pd::States PD,
      l3gd20::ctrl_reg1::bw_odr::States DR_BW,
      l3gd20::ctrl_reg4::sim::States SIM,
      l3gd20::ctrl_reg4::fs::States FS,
      l3gd20::ctrl_reg4::ble::States BLE,
      l3gd20::ctrl_reg4::bdu::States BDU)
  {
    i2c::Standard<I>::writeSlaveRegister(
        A,
        ctrl_reg1::ADDRESS,
        PD + DR_BW + XEN + YEN + ZEN);
    i2c::Standard<I>::writeSlaveRegister(
        A,
        ctrl_reg4::ADDRESS,
        SIM + FS + BLE + BDU);
  }

  /**
   * @brief Reads the low byte of the X axis magnetic field.
   */
  template<i2c::Address I, Address A>
  u8 Functions<I, A>::readXLow()
  {
    return i2c::Standard<I>::readSlaveRegister(
        A,
        out_x_l::ADDRESS);
  }

  /**
   * @brief Reads the high byte of the X axis magnetic field.
   */
  template<i2c::Address I, Address A>
  u8 Functions<I, A>::readXHigh()
  {
    return i2c::Standard<I>::readSlaveRegister(
        A,
        out_x_h::ADDRESS);
  }

  /**
   * @brief Reads the low byte of the Y axis magnetic field.
   */
  template<i2c::Address I, Address A>
  u8 Functions<I, A>::readYLow()
  {
    return i2c::Standard<I>::readSlaveRegister(
        A,
        out_y_l::ADDRESS);
  }

  /**
   * @brief Reads the high byte of the Y axis magnetic field.
   */
  template<i2c::Address I, Address A>
  u8 Functions<I, A>::readYHigh()
  {
    return i2c::Standard<I>::readSlaveRegister(
        A,
        out_y_h::ADDRESS);
  }

  /**
   * @brief Reads the low byte of the Z axis magnetic field.
   */
  template<i2c::Address I, Address A>
  u8 Functions<I, A>::readZLow()
  {
    return i2c::Standard<I>::readSlaveRegister(
        A,
        out_z_l::ADDRESS);
  }

  /**
   * @brief Reads the high byte of the Z axis magnetic field.
   */
  template<i2c::Address I, Address A>
  u8 Functions<I, A>::readZHigh()
  {
    return i2c::Standard<I>::readSlaveRegister(
        A,
        out_z_h::ADDRESS);
  }

}  // namespace l3gd20
