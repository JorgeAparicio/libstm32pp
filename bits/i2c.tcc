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

#include "bitband.hpp"
#include "../include/peripheral/i2c.hpp"

namespace i2c {
  /**
   * @brief Configures the I2C in standard mode.
   * @brief Overrides the old configuration.
   */
  template<address::E I>
  template<
      i2c::registers::cr1::bits::pe::states::E PE,
      i2c::registers::cr1::bits::enpec::states::E ENPEC,
      i2c::registers::cr1::bits::engc::states::E ENGC,
      i2c::registers::cr1::bits::nostretch::states::E NOSTRETCH,
      i2c::registers::cr2::bits::iterren::states::E ITERREN,
      i2c::registers::cr2::bits::itevten::states::E ITVEN,
      i2c::registers::cr2::bits::itbufen::states::E ITBUFEN,
      i2c::registers::cr2::bits::dmaen::states::E DMAEN,
      i2c::registers::cr2::bits::last::states::E LAST
  >
  void Standard<I>::configure()
  {
    static_assert((FREQUENCY >= 2000000) || (FREQUENCY <= 42000000),
        "I2C Frequency must be between 2 MHz and 42 MHz (inclusive)");

    reinterpret_cast<Registers*>(I)->CR1 =
        PE + ENPEC + ENGC + NOSTRETCH;

    reinterpret_cast<Registers*>(I)->CR2 =
        ITERREN + ITVEN + ITBUFEN + DMAEN + LAST +
            ((FREQUENCY / 1000000) << registers::cr2::bits::freq::POSITION);
  }

  /**
   * @brief Configures the I2C clock.
   * @note  In standard mode, CCR >= 4 and in fast mode, CCR  >= 1
   *                           APB1
   * FREQ = -----------------------------------------
   *        CCR *(NORMAL:2, FAST:2 + 1, FAST: 16 + 9)
   */
  template<address::E I>
  template<
      registers::ccr::bits::f_s::states::E F_S,
      registers::ccr::bits::duty::states::E DUTY,
      u32 FREQUENCY_HZ
  >
  void Standard<I>::configureClock()
  {
    enum {
      CCR = FREQUENCY
          / (FREQUENCY_HZ
              *
              (F_S == registers::ccr::bits::f_s::states::STANDARD_MODE ?
                  2 :
                  (DUTY
                      == registers::ccr::bits::duty::states::T_LOW_16_T_HIGH_9 ?
                      25 :
                      3)))
    };

    static_assert(CCR < 2048,
        "This frequency can't be archived with this configuration.");
    static_assert((CCR >= 1) ||
        (F_S == registers::ccr::bits::f_s::states::STANDARD_MODE),
        "This frequency can't be archived with this configuration.");
    static_assert((CCR >= 4) ||
        (F_S == registers::ccr::bits::f_s::states::FAST_MODE),
        "This frequency can't be archived with this configuration.");

    reinterpret_cast<Registers*>(I)->CCR = F_S + DUTY + CCR;
  }

  /**
   * @brief Enables the I2C's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  template<address::E I>
  void Standard<I>::enableClock()
  {
    RCC::enableClocks<
        I == address::I2C1 ?
            rcc::registers::apb1enr::bits::I2C1 :
            (I == address::I2C2 ?
                rcc::registers::apb1enr::bits::I2C2 :
                #ifdef STM32F1XX
                0)
#else // STM32F1XX
                (I == address::I2C3 ?
                                      rcc::registers::apb1enr::bits::I2C3 :
                                      0))
    #endif // STM32F1XX
    >();
  }

  /**
   * @brief Disables the I2C's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  template<address::E I>
  void Standard<I>::disableClock()
  {
    RCC::disableClocks<
        I == address::I2C1 ?
            rcc::registers::apb1enr::bits::I2C1 :
            (I == address::I2C2 ?
                rcc::registers::apb1enr::bits::I2C2 :
                #ifdef STM32F1XX
                0)
#else // STM32F1XX
                (I == address::I2C3 ?
                                      rcc::registers::apb1enr::bits::I2C3 :
                                      0))
    #endif // STM32F1XX
    >();
  }

  /**
   * @brief Turns on the I2C peripheral.
   */
  template<address::E I>
  void Standard<I>::enablePeripheral()
  {
    *(u32*) (bitband::Peripheral<
        I + registers::cr1::OFFSET,
        registers::cr1::bits::pe::POSITION
    >::address) = 1;
  }

  /**
   * @brief Turns off the I2C peripheral.
   */
  template<address::E I>
  void Standard<I>::disablePeripheral()
  {
    *(u32*) (bitband::Peripheral<
        I + registers::cr1::OFFSET,
        registers::cr1::bits::pe::POSITION
    >::address) = 0;
  }

  /**
   * @brief Sends Start condition.
   */
  template<address::E I>
  void Standard<I>::sendStart()
  {
    *(u32*) (bitband::Peripheral<
        I + registers::cr1::OFFSET,
        registers::cr1::bits::start::POSITION
    >::address) = 1;
  }

  /**
   * @brief Sends Stop condition.
   */
  template<address::E I>
  void Standard<I>::sendStop()
  {
    *(u32*) (bitband::Peripheral<
        I + registers::cr1::OFFSET,
        registers::cr1::bits::stop::POSITION
    >::address) = 1;
  }

  /**
   * @brief Sends data.
   */
  template<address::E I>
  void Standard<I>::sendData(u8 const data)
  {
    reinterpret_cast<Registers*>(I)->DR = data;
  }

  /**
   * @brief Sends data.
   */
  template<address::E I>
  u8 Standard<I>::getData()
  {
    return reinterpret_cast<Registers*>(I)->DR;
  }

  /**
   * @brief Sends the slave's address.
   */
  template<address::E I>
  template<operation::E op>
  void Standard<I>::sendAddress(u8 const add)
  {
    reinterpret_cast<Registers*>(I)->DR = (add << 1) + op;
  }

  /**
   * @brief Send acknowledge after byte reception.
   */
  template<address::E I>
  void Standard<I>::enableACK()
  {
    *(u32*) (bitband::Peripheral<
        I + registers::cr1::OFFSET,
        registers::cr1::bits::ack::POSITION
    >::address) = 1;
  }

  /**
   * @brief Don't send acknowledge after byte reception.
   */
  template<address::E I>
  void Standard<I>::disableACK()
  {
    *(u32*) (bitband::Peripheral<
        I + registers::cr1::OFFSET,
        registers::cr1::bits::ack::POSITION
    >::address) = 0;
  }

  /**
   * @brief returns true if a start condition has been sent.
   */
  template<address::E I>
  bool Standard<I>::hasSentStart()
  {
    return *(bool*) bitband::Peripheral<
        I + registers::sr1::OFFSET,
        registers::sr1::bits::sb::POSITION
    >::address;
  }

  /**
   * @brief returns true when a stop condition has been sent.
   */
  template<address::E I>
  bool Standard<I>::hasSentStop()
  {
    return *(bool*) bitband::Peripheral<
        I + registers::sr1::OFFSET,
        registers::sr1::bits::stopf::POSITION
    >::address;
  }

  /**
   * @brief returns true when the slave address has been transmitted.
   */
  template<address::E I>
  bool Standard<I>::hasAddressTransmitted()
  {
    return *(bool*) bitband::Peripheral<
        I + registers::sr1::OFFSET,
        registers::sr1::bits::addr::POSITION
    >::address;
  }

  /**
   * @brief returns true if data has been received.
   */
  template<address::E I>
  bool Standard<I>::hasReceivedData()
  {
    return *(bool*) bitband::Peripheral<
        I + registers::sr1::OFFSET,
        registers::sr1::bits::rxne::POSITION
    >::address;
  }

  /**
   * @brief 0 - Can't send data, 1 - Can send data.
   */
  template<address::E I>
  bool Standard<I>::canSendData()
  {
    return *(bool*) bitband::Peripheral<
        I + registers::sr1::OFFSET,
        registers::sr1::bits::txe::POSITION
    >::address;
  }

  /**
   * @brief 0 - Transfer hasn't finished, 1 - Transfer has finished.
   */
  template<address::E I>
  bool Standard<I>::hasTranferFinished()
  {
    return *(bool*) bitband::Peripheral<
        I + registers::sr1::OFFSET,
        registers::sr1::bits::btf::POSITION
    >::address;
  }

  /**
   * @brief  0 - No communication on the bus,
   *          1 - Communication ongoing on the bus.
   */
  template<address::E I>
  bool Standard<I>::isTheBusBusy()
  {
    return *(bool*) (bitband::Peripheral<
        I + registers::sr2::OFFSET,
        registers::sr2::bits::busy::POSITION
    >::address);
  }

  /**
   * @brief Writes a value to a slave device register.
   */
  template<address::E I>
  void Standard<I>::writeSlaveRegister(
      u8 const slaveAddress,
      u8 const registerAddress,
      u8 const value)
  {
    sendStart();

    while (!hasSentStart()) {
    };

    Standard<I>::sendAddress<i2c::operation::WRITE>(slaveAddress);

    while (!hasAddressTransmitted()) {
    };

    reinterpret_cast<Registers*>(I)->SR2;

    sendData(registerAddress);

    while (!canSendData()) {
    };

    sendData(value);

    while (!hasTranferFinished()) {
    };

    sendStop();

    while (isTheBusBusy()) {
    };
  }

  /**
   * @brief Reads a value from a slave device register.
   */
  template<address::E I>
  u8 Standard<I>::readSlaveRegister(
      u8 const slaveAddress,
      u8 const registerAddress)
  {
    sendStart();

    while (!hasSentStart()) {
    };

    sendAddress<i2c::operation::WRITE>(slaveAddress);

    while (!hasAddressTransmitted()) {
    };

    reinterpret_cast<Registers*>(I)->SR2;

    sendData(registerAddress);

    while (!canSendData()) {
    };

    sendStart();

    while (!hasSentStart()) {
    };

    sendAddress<i2c::operation::READ>(slaveAddress);

    disableACK();

    while (!hasAddressTransmitted()) {
    };

    reinterpret_cast<Registers*>(I)->SR2;

    while (!hasReceivedData()) {
    };

    sendStop();

    while (isTheBusBusy()) {
    };

    return getData();
  }
}  // namespace i2c
