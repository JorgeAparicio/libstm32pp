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
  namespace accelerometer {
    /**
     * @brief Configures the Functions.
     */
    template<i2c::Address I>
    void Functions<I>::configure(
        ctrl1::lpen::States LPEN,
        ctrl1::odr::States ODR,
        ctrl1::xen::States XEN,
        ctrl1::yen::States YEN,
        ctrl1::zen::States ZEN,
        ctrl4::sim::States SIM,
        ctrl4::hr::States HR,
        ctrl4::fs::States FS,
        ctrl4::ble::States BLE,
        ctrl4::bdu::States BDU)
    {
      i2c::Standard<I>::writeSlaveRegister(
          ADDRESS,
          ctrl1::ADDRESS,
          LPEN + ODR + XEN + YEN + ZEN);
      i2c::Standard<I>::writeSlaveRegister(
          ADDRESS,
          ctrl4::ADDRESS,
          SIM + HR + FS + BLE + BDU);
    }

    /**
     * @brief Reads the low byte of the X axis acceleration.
     */
    template<i2c::Address I>
    u8 Functions<I>::readXLow()
    {
      return i2c::Standard<I>::readSlaveRegister(
          ADDRESS,
          out_x_l::ADDRESS);
    }

    /**
     * @brief Reads the high byte of the X axis acceleration.
     */
    template<i2c::Address I>
    u8 Functions<I>::readXHigh()
    {
      return i2c::Standard<I>::readSlaveRegister(
          ADDRESS,
          out_x_h::ADDRESS);
    }

    /**
     * @brief Reads the low byte of the Y axis acceleration.
     */
    template<i2c::Address I>
    u8 Functions<I>::readYLow()
    {
      return i2c::Standard<I>::readSlaveRegister(
          ADDRESS,
          out_y_l::ADDRESS);
    }

    /**
     * @brief Reads the high byte of the Y axis acceleration.
     */
    template<i2c::Address I>
    u8 Functions<I>::readYHigh()
    {
      return i2c::Standard<I>::readSlaveRegister(
          ADDRESS,
          out_y_h::ADDRESS);
    }

    /**
     * @brief Reads the low byte of the Z axis acceleration.
     */
    template<i2c::Address I>
    u8 Functions<I>::readZLow()
    {
      return i2c::Standard<I>::readSlaveRegister(
          ADDRESS,
          out_z_l::ADDRESS);
    }

    /**
     * @brief Reads the high byte of the Z axis acceleration.
     */
    template<i2c::Address I>
    u8 Functions<I>::readZHigh()
    {
      return i2c::Standard<I>::readSlaveRegister(
          ADDRESS,
          out_z_h::ADDRESS);
    }
  }  // namespace accelerometer

  namespace magnetometer {
    /**
     * @brief Sets the magnetometer data rate.
     */
    template<i2c::Address I>
    void Functions<I>::setDataRate(cra::do_::States DO_)
    {
      u32 tmp = i2c::Standard<I>::readSlaveRegister(
          ADDRESS,
          cra::ADDRESS);
      tmp = (tmp & ~cra::do_::MASK) + DO_;
      i2c::Standard<I>::writeSlaveRegister(
          ADDRESS,
          cra::ADDRESS,
          tmp);
    }

    /**
     * @brief Sets the magnetometer reading range.
     */
    template<i2c::Address I>
    void Functions<I>::setReadingRange(crb::gn::States GN)
    {
      i2c::Standard<I>::writeSlaveRegister(
          ADDRESS,
          crb::ADDRESS,
          GN);
    }

    /**
     * @brief Sets the magnetometer sampling mode.
     */
    template<i2c::Address I>
    void Functions<I>::setMode(mr::md::States MD)
    {
      i2c::Standard<I>::writeSlaveRegister(
          ADDRESS,
          mr::ADDRESS,
          MD);
    }

    /**
     * @brief Reads the low byte of the X axis magnetic field.
     */
    template<i2c::Address I>
    u8 Functions<I>::readXLow()
    {
      return i2c::Standard<I>::readSlaveRegister(
          ADDRESS,
          out_x_l::ADDRESS);
    }

    /**
     * @brief Reads the high byte of the X axis magnetic field.
     */
    template<i2c::Address I>
    u8 Functions<I>::readXHigh()
    {
      return i2c::Standard<I>::readSlaveRegister(
          ADDRESS,
          out_x_h::ADDRESS);
    }

    /**
     * @brief Reads the low byte of the Y axis magnetic field.
     */
    template<i2c::Address I>
    u8 Functions<I>::readYLow()
    {
      return i2c::Standard<I>::readSlaveRegister(
          ADDRESS,
          out_y_l::ADDRESS);
    }

    /**
     * @brief Reads the high byte of the Y axis magnetic field.
     */
    template<i2c::Address I>
    u8 Functions<I>::readYHigh()
    {
      return i2c::Standard<I>::readSlaveRegister(
          ADDRESS,
          out_y_h::ADDRESS);
    }

    /**
     * @brief Reads the low byte of the Z axis magnetic field.
     */
    template<i2c::Address I>
    u8 Functions<I>::readZLow()
    {
      return i2c::Standard<I>::readSlaveRegister(
          ADDRESS,
          out_z_l::ADDRESS);
    }

    /**
     * @brief Reads the high byte of the Z axis magnetic field.
     */
    template<i2c::Address I>
    u8 Functions<I>::readZHigh()
    {
      return i2c::Standard<I>::readSlaveRegister(
          ADDRESS,
          out_z_h::ADDRESS);
    }
  }  // namespace magnetometer

  namespace thermometer {
    /**
     * @brief Disables the thermometer.
     */
    template<i2c::Address I>
    void Functions<I>::disable()
    {
      u8 tmp = i2c::Standard<I>::readSlaveRegister(
          magnetometer::ADDRESS,
          magnetometer::cra::ADDRESS);
      tmp &= ~magnetometer::cra::temp_en::TEMPERATURE_SENSOR_ENABLED;
      i2c::Standard<I>::writeSlaveRegister(
          magnetometer::ADDRESS,
          magnetometer::cra::ADDRESS,
          tmp);
    }

    /**
     * @brief Enables the thermometer.
     */
    template<i2c::Address I>
    void Functions<I>::enable()
    {
      u8 tmp = i2c::Standard<I>::readSlaveRegister(
          magnetometer::ADDRESS,
          magnetometer::cra::ADDRESS);
      tmp |= magnetometer::cra::temp_en::TEMPERATURE_SENSOR_ENABLED;
      i2c::Standard<I>::writeSlaveRegister(
          magnetometer::ADDRESS,
          magnetometer::cra::ADDRESS,
          tmp);
    }
  }  // namespace thermometer
}  // namespace lsm303dlhc

