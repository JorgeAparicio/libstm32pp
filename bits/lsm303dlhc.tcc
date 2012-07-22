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

namespace lsm303dlhc {

  /**
   * @brief Configures the accelerometer.
   */
  template<i2c::address::E I>
  template<
      lsm303dlhc::registers::ctrl_reg1_a::bits::lpen::states::E LPEN,
      lsm303dlhc::registers::ctrl_reg1_a::bits::odr::states::E ODR,
      lsm303dlhc::registers::ctrl_reg1_a::bits::xen::states::E XEN,
      lsm303dlhc::registers::ctrl_reg1_a::bits::yen::states::E YEN,
      lsm303dlhc::registers::ctrl_reg1_a::bits::zen::states::E ZEN,
      lsm303dlhc::registers::ctrl_reg4_a::bits::sim::states::E SIM,
      lsm303dlhc::registers::ctrl_reg4_a::bits::hr::states::E HR,
      lsm303dlhc::registers::ctrl_reg4_a::bits::fs::states::E FS,
      lsm303dlhc::registers::ctrl_reg4_a::bits::ble::states::E BLE,
      lsm303dlhc::registers::ctrl_reg4_a::bits::bdu::states::E BDU
  >
  void Accelerometer<I>::configure()
  {
    i2c::Standard<I>::writeSlaveRegister(
        address::ACCELEROMETER,
        registers::ctrl_reg1_a::ADDRESS,
        LPEN + ODR + XEN + YEN + ZEN);
    i2c::Standard<I>::writeSlaveRegister(
        address::ACCELEROMETER,
        registers::ctrl_reg4_a::ADDRESS,
        SIM + HR + FS + BLE + BDU);
  }

  /**
   * @brief Reads the low byte of the X axis acceleration.
   */
  template<i2c::address::E I>
  u8 Accelerometer<I>::readXLow()
  {
    return i2c::Standard<I>::readSlaveRegister(
        address::ACCELEROMETER,
        registers::out_x_l_a::ADDRESS);
  }

  /**
   * @brief Reads the high byte of the X axis acceleration.
   */
  template<i2c::address::E I>
  u8 Accelerometer<I>::readXHigh()
  {
    return i2c::Standard<I>::readSlaveRegister(
        address::ACCELEROMETER,
        registers::out_x_h_a::ADDRESS);
  }

  /**
   * @brief Reads the low byte of the Y axis acceleration.
   */
  template<i2c::address::E I>
  u8 Accelerometer<I>::readYLow()
  {
    return i2c::Standard<I>::readSlaveRegister(
        address::ACCELEROMETER,
        registers::out_y_l_a::ADDRESS);
  }

  /**
   * @brief Reads the high byte of the Y axis acceleration.
   */
  template<i2c::address::E I>
  u8 Accelerometer<I>::readYHigh()
  {
    return i2c::Standard<I>::readSlaveRegister(
        address::ACCELEROMETER,
        registers::out_y_h_a::ADDRESS);
  }

  /**
   * @brief Reads the low byte of the Z axis acceleration.
   */
  template<i2c::address::E I>
  u8 Accelerometer<I>::readZLow()
  {
    return i2c::Standard<I>::readSlaveRegister(
        address::ACCELEROMETER,
        registers::out_z_l_a::ADDRESS);
  }

  /**
   * @brief Reads the high byte of the Z axis acceleration.
   */
  template<i2c::address::E I>
  u8 Accelerometer<I>::readZHigh()
  {
    return i2c::Standard<I>::readSlaveRegister(
        address::ACCELEROMETER,
        registers::out_z_h_a::ADDRESS);
  }

  /**
   * @brief Sets the magnetometer data rate.
   */
  template<i2c::address::E I>
  template<
      lsm303dlhc::registers::cra_reg_m::bits::do_::states::E DO_
  >
  void Magnetometer<I>::setDataRate()
  {
    u32 tmp = i2c::Standard<I>::readSlaveRegister(
        address::MAGNETOMETER,
        registers::cra_reg_m::ADDRESS);
    tmp = (tmp & ~registers::cra_reg_m::bits::do_::MASK) + DO_;
    i2c::Standard<I>::writeSlaveRegister(
        address::MAGNETOMETER,
        registers::cra_reg_m::ADDRESS,
        tmp);
  }

  /**
   * @brief Sets the magnetometer reading range.
   */
  template<i2c::address::E I>
  template<
      lsm303dlhc::registers::crb_reg_m::bits::gn::states::E GN
  >
  void Magnetometer<I>::setReadingRange()
  {
    i2c::Standard<I>::writeSlaveRegister(
        address::MAGNETOMETER,
        registers::crb_reg_m::ADDRESS,
        GN);
  }

  /**
   * @brief Sets the magnetometer sampling mode.
   */
  template<i2c::address::E I>
  template<
      lsm303dlhc::registers::mr_reg_m::bits::md::states::E MD
  >
  void Magnetometer<I>::setMode()
  {
    i2c::Standard<I>::writeSlaveRegister(
        address::MAGNETOMETER,
        registers::mr_reg_m::ADDRESS,
        MD);
  }

  /**
   * @brief Reads the low byte of the X axis magnetic field.
   */
  template<i2c::address::E I>
  u8 Magnetometer<I>::readXLow()
  {
    return i2c::Standard<I>::readSlaveRegister(
        address::MAGNETOMETER,
        registers::out_x_l_m::ADDRESS);
  }

  /**
   * @brief Reads the high byte of the X axis magnetic field.
   */
  template<i2c::address::E I>
  u8 Magnetometer<I>::readXHigh()
  {
    return i2c::Standard<I>::readSlaveRegister(
        address::MAGNETOMETER,
        registers::out_x_h_m::ADDRESS);
  }

  /**
   * @brief Reads the low byte of the Y axis magnetic field.
   */
  template<i2c::address::E I>
  u8 Magnetometer<I>::readYLow()
  {
    return i2c::Standard<I>::readSlaveRegister(
        address::MAGNETOMETER,
        registers::out_y_l_m::ADDRESS);
  }

  /**
   * @brief Reads the high byte of the Y axis magnetic field.
   */
  template<i2c::address::E I>
  u8 Magnetometer<I>::readYHigh()
  {
    return i2c::Standard<I>::readSlaveRegister(
        address::MAGNETOMETER,
        registers::out_y_h_m::ADDRESS);
  }

  /**
   * @brief Reads the low byte of the Z axis magnetic field.
   */
  template<i2c::address::E I>
  u8 Magnetometer<I>::readZLow()
  {
    return i2c::Standard<I>::readSlaveRegister(
        address::MAGNETOMETER,
        registers::out_z_l_m::ADDRESS);
  }

  /**
   * @brief Reads the high byte of the Z axis magnetic field.
   */
  template<i2c::address::E I>
  u8 Magnetometer<I>::readZHigh()
  {
    return i2c::Standard<I>::readSlaveRegister(
        address::MAGNETOMETER,
        registers::out_z_h_m::ADDRESS);
  }

  /**
   * @brief Disables the thermometer.
   */
  template<i2c::address::E I>
  void Thermometer<I>::disable()
  {
    u8 tmp = i2c::Standard<I>::readSlaveRegister(
        address::MAGNETOMETER,
        registers::cra_reg_m::ADDRESS);
    tmp &=
        ~registers::cra_reg_m::bits::temp_en::states::TEMPERATURE_SENSOR_ENABLED;
    i2c::Standard<I>::writeSlaveRegister(
        address::MAGNETOMETER,
        registers::cra_reg_m::ADDRESS,
        tmp);
  }

  /**
   * @brief Enables the thermometer.
   */
  template<i2c::address::E I>
  void Thermometer<I>::enable()
  {
    u8 tmp = i2c::Standard<I>::readSlaveRegister(
        address::MAGNETOMETER,
        registers::cra_reg_m::ADDRESS);
    tmp |= registers::cra_reg_m::bits::temp_en::states::TEMPERATURE_SENSOR_ENABLED;
    i2c::Standard<I>::writeSlaveRegister(
        address::MAGNETOMETER,
        registers::cra_reg_m::ADDRESS,
        tmp);
  }
}  // namespace lsm303dlhc

