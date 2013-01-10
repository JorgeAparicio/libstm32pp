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
#include "../include/peripheral/rcc.hpp"

namespace gpio {
#ifdef STM32F1XX
  /**
   * @brief Enables the port's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  template<Address P, u8 N>
  void Pin<P, N>::enableClock()
  {
    switch (P) {
      case GPIOA:
      RCC::enableClocks<rcc::apb2enr::IOPA>();
      break;
      case GPIOB:
      RCC::enableClocks<rcc::apb2enr::IOPB>();
      break;
      case GPIOC:
      RCC::enableClocks<rcc::apb2enr::IOPC>();
      break;
      case GPIOD:
      RCC::enableClocks<rcc::apb2enr::IOPD>();
      break;
      case GPIOE:
      RCC::enableClocks<rcc::apb2enr::IOPE>();
      break;
      case GPIOF:
      RCC::enableClocks<rcc::apb2enr::IOPF>();
      break;
      case GPIOG:
      RCC::enableClocks<rcc::apb2enr::IOPG>();
      break;
    }
  }

  /**
   * @brief Sets the pin to 1, if it is in any of the output modes.
   */
  template<Address P, u8 N>
  void Pin<P, N>::setHigh()
  {
    reinterpret_cast<Registers *>(P)->BSRR = 1 << N;
  }

  /**
   * @brief Sets the pin to 0, if it is in any of the output modes.
   */
  template<Address P, u8 N>
  void Pin<P, N>::setLow()
  {
    reinterpret_cast<Registers *>(P)->BRR = 1 << N;
  }

  /**
   * @brief Outputs the desired logic state through the pin.
   */
  template<Address P, u8 N>
  void Pin<P, N>::setOutput(u32 const value)
  {
    *(u32 volatile*) (OUT_ADDRESS) = value;
  }

  /**
   * @brief Gets the logic state of the pin.
   */
  template<Address P, u8 N>
  u32 Pin<P, N>::getInput()
  {
    return *(u32 volatile*) (IN_ADDRESS);
  }

  /**
   * @brief Enables the pull-up circuit on this pin. (MODE_INPUT_PULL_X only).
   */
  template<Address P, u8 N>
  void Pin<P, N>::pullUp()
  {
    setHigh();
  }

  /**
   * @brief Enables the pull-down circuit on this pin. (MODE_INPUT_PULL_X only).
   */
  template<Address P, u8 N>
  void Pin<P, N>::pullDown()
  {
    setLow();
  }

  /**
   * @brief Returns true if the pin is high.
   */
  template<Address P, u8 N>
  bool Pin<P, N>::isHigh()
  {
    return *(bool volatile*) (bitband::peripheral<
        P + idr::OFFSET,
        N
        >());
  }

  /**
   * @brief Changes the I/O mode.
   */
  template<Address P, u8 N>
  void Pin<P, N>::setMode(cr::States Mode)
  {
    reinterpret_cast<Registers*>(P)->CR[N > 7 ? 1 : 0] &=
    ~(cr::MASK <<
        cr::POSITION * (N > 7 ? (N - 8) : N));

    reinterpret_cast<Registers*>(P)->CR[N > 7 ? 1 : 0] |=
    Mode << cr::POSITION * (N > 7 ? (N - 8) : N);
  }

  /**
   * @brief Enables the port's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  template<Address P>
  void Port<P>::enableClock()
  {
    switch (P) {
      case GPIOA:
      RCC::enableClocks<rcc::apb2enr::IOPA>();
      break;
      case GPIOB:
      RCC::enableClocks<rcc::apb2enr::IOPB>();
      break;
      case GPIOC:
      RCC::enableClocks<rcc::apb2enr::IOPC>();
      break;
      case GPIOD:
      RCC::enableClocks<rcc::apb2enr::IOPD>();
      break;
      case GPIOE:
      RCC::enableClocks<rcc::apb2enr::IOPE>();
      break;
      case GPIOF:
      RCC::enableClocks<rcc::apb2enr::IOPF>();
      break;
      case GPIOG:
      RCC::enableClocks<rcc::apb2enr::IOPG>();
      break;
    }
  }

  /**
   * @brief Disables the port's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  template<Address P>
  void Port<P>::disableClock()
  {
    switch (P) {
      case GPIOA:
      RCC::disableClocks<rcc::apb2enr::IOPA>();
      break;
      case GPIOB:
      RCC::disableClocks<rcc::apb2enr::IOPB>();
      break;
      case GPIOC:
      RCC::disableClocks<rcc::apb2enr::IOPC>();
      break;
      case GPIOD:
      RCC::disableClocks<rcc::apb2enr::IOPD>();
      break;
      case GPIOE:
      RCC::disableClocks<rcc::apb2enr::IOPE>();
      break;
      case GPIOF:
      RCC::disableClocks<rcc::apb2enr::IOPF>();
      break;
      case GPIOG:
      RCC::disableClocks<rcc::apb2enr::IOPG>();
      break;
    }
  }

  /**
   * @brief Configures the pins 0-7 of the port.
   * @note  PB3 and PB4 are JTAG pins, they should be configured as ALTERNATE
   *        if using the JTAG.
   */
  template<Address P>
  void Port<P>::configureLowerPins(
      cr::States P0,
      cr::States P1,
      cr::States P2,
      cr::States P3,
      cr::States P4,
      cr::States P5,
      cr::States P6,
      cr::States P7)
  {
    reinterpret_cast<Registers *>(P)->CR[0] =
    (P0 << 0) + (P1 << 4) + (P2 << 8) + (P3 << 12) + (P4 << 16) +
    (P5 << 20) + (P6 << 24) + (P7 << 28);
  }

  /**
   * @brief Configures the pins 8-15 of the port.
   * @note  PA13, PA14 and PA15 are JTAG pins, they should be configured as
   *        ALTERNATE if using the JTAG.
   */
  template<Address P>
  void Port<P>::configureHigherPins(
      cr::States P8,
      cr::States P9,
      cr::States P10,
      cr::States P11,
      cr::States P12,
      cr::States P13,
      cr::States P14,
      cr::States P15)
  {
    reinterpret_cast<Registers *>(P)->CR[1] =
    (P8 << 0) + (P9 << 4) + (P10 << 8) + (P11 << 12) +
    (P12 << 16) + (P13 << 20) + (P14 << 24) + (P15 << 28);
  }

  /**
   * @brief Outputs the desired logic state on the port.
   */
  template<Address P>
  void Port<P>::setValue(u32 const value)
  {
    reinterpret_cast<Registers *>(P)->ODR =
    value;
  }

  /**
   * @brief Gets the logic state of the port.
   */
  template<Address P>
  u32 Port<P>::getValue()
  {
    return reinterpret_cast<Registers *>(P)->IDR;
  }

#else // STM32F1XX
  /**
   * @brief Enables the port's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  template<Address P, u8 N>
  void Pin<P, N>::enableClock()
  {
    switch (P) {
      case GPIOA:
        RCC::enableClocks<rcc::ahb1enr::GPIOA>();
        break;
      case GPIOB:
        RCC::enableClocks<rcc::ahb1enr::GPIOB>();
        break;
      case GPIOC:
        RCC::enableClocks<rcc::ahb1enr::GPIOC>();
        break;
      case GPIOD:
        RCC::enableClocks<rcc::ahb1enr::GPIOD>();
        break;
      case GPIOE:
        RCC::enableClocks<rcc::ahb1enr::GPIOE>();
        break;
      case GPIOF:
        RCC::enableClocks<rcc::ahb1enr::GPIOF>();
        break;
      case GPIOG:
        RCC::enableClocks<rcc::ahb1enr::GPIOG>();
        break;
      case GPIOH:
        RCC::enableClocks<rcc::ahb1enr::GPIOH>();
        break;
      case GPIOI:
        RCC::enableClocks<rcc::ahb1enr::GPIOI>();
        break;
    }
  }

  /**
   * @brief Sets the pin to 1, if it is in output mode.
   */
  template<Address P, u8 N>
  void Pin<P, N>::setHigh()
  {
    reinterpret_cast<Registers *>(P)->BSRR = (1 << N);
  }

  /**
   * @brief Sets the pin to 0, if it is in output mode.
   */
  template<Address P, u8 N>
  void Pin<P, N>::setLow()
  {
    reinterpret_cast<Registers *>(P)->BSRR = (1 << N) << 16;
  }

  /**
   * @brief Outputs the logic state on the pin.
   */
  template<Address P, u8 N>
  void Pin<P, N>::setOutput(u32 const value)
  {
    *(u32 volatile*) (OUT_ADDRESS) = value;
  }

  /**
   * @brief Gets the logic state of the pin.
   */
  template<Address P, u8 N>
  u32 Pin<P, N>::getInput()
  {
    return *(u32 volatile*) (IN_ADDRESS);
  }

  /**
   * @brief Returns true if the pin is high.
   */
  template<Address P, u8 N>
  bool Pin<P, N>::isHigh()
  {
    return *(bool volatile*) (bitband::peripheral<
        P + idr::OFFSET,
        N
    >());
  }

  /**
   * @brief Selects the I/O mode.
   */
  template<Address P, u8 N>
  void Pin<P, N>::setMode(moder::States Mode)
  {
    reinterpret_cast<Registers *>(P)->MODER &=
        ~(moder::MASK << (2 * N));
    reinterpret_cast<Registers *>(P)->MODER |=
        (Mode << (2 * N));
  }

  /**
   * @brief Selects the output mode.
   */
  template<Address P, u8 N>
  void Pin<P, N>::setOutputMode(otyper::States OutputMode)
  {
    *((u32 volatile*) (bitband::peripheral<
        P + otyper::OFFSET,
        N
    >())) = OutputMode;
  }

  /**
   * @brief Selects the output speed.
   */
  template<Address P, u8 N>
  void Pin<P, N>::setSpeed(ospeedr::States Speed)
  {
    reinterpret_cast<Registers *>(P)->OSPEEDR &=
        ~(ospeedr::MASK << (2 * N));
    reinterpret_cast<Registers *>(P)->OSPEEDR |=
        (Speed << (2 * N));
  }

  /**
   * @brief Selects between pull-up, pull-down or no pull-up/pull-down.
   */
  template<Address P, u8 N>
  void Pin<P, N>::setPullMode(pupdr::States PullMode)
  {
    reinterpret_cast<Registers *>(P)->PUPDR &=
        ~(pupdr::MASK << (2 * N));
    reinterpret_cast<Registers *>(P)->PUPDR |=
        (PullMode << (2 * N));
  }

  /**
   * @brief Selects the alternate function.
   */
  template<Address P, u8 N>
  void Pin<P, N>::setAlternateFunction(afr::States Alternate)
  {
    reinterpret_cast<Registers*>(P)->AFR[N > 7 ? 1 : 0] &=
        ~(afr::MASK <<
            afr::POSITION * (N > 7 ? (N - 8) : N));

    reinterpret_cast<Registers*>(P)->AFR[N > 7 ? 1 : 0] |=
        Alternate << (afr::POSITION * (N > 7 ? (N - 8) : N));
  }

  /**
   * @brief Enables the port's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  template<Address P>
  void Port<P>::enableClock()
  {
    switch (P) {
      case GPIOA:
        RCC::enableClocks<rcc::ahb1enr::GPIOA>();
        break;
      case GPIOB:
        RCC::enableClocks<rcc::ahb1enr::GPIOB>();
        break;
      case GPIOC:
        RCC::enableClocks<rcc::ahb1enr::GPIOC>();
        break;
      case GPIOD:
        RCC::enableClocks<rcc::ahb1enr::GPIOD>();
        break;
      case GPIOE:
        RCC::enableClocks<rcc::ahb1enr::GPIOE>();
        break;
      case GPIOF:
        RCC::enableClocks<rcc::ahb1enr::GPIOF>();
        break;
      case GPIOG:
        RCC::enableClocks<rcc::ahb1enr::GPIOG>();
        break;
      case GPIOH:
        RCC::enableClocks<rcc::ahb1enr::GPIOH>();
        break;
      case GPIOI:
        RCC::enableClocks<rcc::ahb1enr::GPIOI>();
        break;
    }
  }

  /**
   * @brief Disables the port's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  template<Address P>
  void Port<P>::disableClock()
  {
    switch (P) {
      case GPIOA:
        RCC::disableClocks<rcc::ahb1enr::GPIOA>();
        break;
      case GPIOB:
        RCC::disableClocks<rcc::ahb1enr::GPIOB>();
        break;
      case GPIOC:
        RCC::disableClocks<rcc::ahb1enr::GPIOC>();
        break;
      case GPIOD:
        RCC::disableClocks<rcc::ahb1enr::GPIOD>();
        break;
      case GPIOE:
        RCC::disableClocks<rcc::ahb1enr::GPIOE>();
        break;
      case GPIOF:
        RCC::disableClocks<rcc::ahb1enr::GPIOF>();
        break;
      case GPIOG:
        RCC::disableClocks<rcc::ahb1enr::GPIOG>();
        break;
      case GPIOH:
        RCC::enableClocks<rcc::ahb1enr::GPIOH>();
        break;
      case GPIOI:
        RCC::enableClocks<rcc::ahb1enr::GPIOI>();
        break;
    }
  }

  /**
   * @brief Outputs the logic states on the port.
   */
  template<Address P>
  void Port<P>::setOutput(u16 const value)
  {
    reinterpret_cast<Registers*>(P)->ODR = value;
  }

  /**
   * @brief Gets the logic state of the port.
   */
  template<Address P>
  u16 Port<P>::getInput()
  {
    return reinterpret_cast<Registers*>(P)->IDR;
  }

  /**
   * @brief Configures the mode of the port.
   * @note  PB3, PB4, PA13, PA14 and PA15 are JTAG pins, they should be
   *        configured as ALTERNATE if using the JTAG.
   */
  template<Address P>
  void Port<P>::setModes(
      moder::States M0,
      moder::States M1,
      moder::States M2,
      moder::States M3,
      moder::States M4,
      moder::States M5,
      moder::States M6,
      moder::States M7,
      moder::States M8,
      moder::States M9,
      moder::States M10,
      moder::States M11,
      moder::States M12,
      moder::States M13,
      moder::States M14,
      moder::States M15)
  {
    reinterpret_cast<Registers *>(P)->MODER =
        (M0 << 0) + (M1 << 2) + (M2 << 4) + (M3 << 6) + (M4 << 8) +
            (M5 << 10) + (M6 << 12) + (M7 << 14) + (M8 << 16) + (M9 << 18) +
            (M10 << 20) + (M11 << 22) + (M12 << 24) + (M13 << 26) +
            (M14 << 28) + (M15 << 30);
  }

  /**
   * @brief Configures the output mode of the port.
   */
  template<Address P>
  void Port<P>::setOutputTypes(
      otyper::States M0,
      otyper::States M1,
      otyper::States M2,
      otyper::States M3,
      otyper::States M4,
      otyper::States M5,
      otyper::States M6,
      otyper::States M7,
      otyper::States M8,
      otyper::States M9,
      otyper::States M10,
      otyper::States M11,
      otyper::States M12,
      otyper::States M13,
      otyper::States M14,
      otyper::States M15)
  {
    reinterpret_cast<Registers *>(P)->OTYPER =
        (M0 << 0) + (M1 << 1) + (M2 << 2) + (M3 << 3) + (M4 << 4) +
            (M5 << 5) + (M6 << 6) + (M7 << 7) + (M8 << 8) + (M9 << 9) +
            (M10 << 10) + (M11 << 11) + (M12 << 12) + (M13 << 13) +
            (M14 << 14) + (M15 << 15);
  }

  /**
   * @brief Configures the output speed of the port.
   */
  template<Address P>
  void Port<P>::setOutputSpeeds(
      ospeedr::States M0,
      ospeedr::States M1,
      ospeedr::States M2,
      ospeedr::States M3,
      ospeedr::States M4,
      ospeedr::States M5,
      ospeedr::States M6,
      ospeedr::States M7,
      ospeedr::States M8,
      ospeedr::States M9,
      ospeedr::States M10,
      ospeedr::States M11,
      ospeedr::States M12,
      ospeedr::States M13,
      ospeedr::States M14,
      ospeedr::States M15)
  {
    reinterpret_cast<Registers *>(P)->OSPEEDR =
        (M0 << 0) + (M1 << 2) + (M2 << 4) + (M3 << 6) + (M4 << 8) +
            (M5 << 10) + (M6 << 12) + (M7 << 14) + (M8 << 16) + (M9 << 18) +
            (M10 << 20) + (M11 << 22) + (M12 << 24) + (M13 << 26) +
            (M14 << 28) + (M15 << 30);
  }

  /**
   * @brief Configures the input mode of the port.
   */
  template<Address P>
  void Port<P>::setPullModes(
      pupdr::States M0,
      pupdr::States M1,
      pupdr::States M2,
      pupdr::States M3,
      pupdr::States M4,
      pupdr::States M5,
      pupdr::States M6,
      pupdr::States M7,
      pupdr::States M8,
      pupdr::States M9,
      pupdr::States M10,
      pupdr::States M11,
      pupdr::States M12,
      pupdr::States M13,
      pupdr::States M14,
      pupdr::States M15)
  {
    reinterpret_cast<Registers *>(P)->PUPDR =
        (M0 << 0) + (M1 << 2) + (M2 << 4) + (M3 << 6) + (M4 << 8) +
            (M5 << 10) + (M6 << 12) + (M7 << 14) + (M8 << 16) + (M9 << 18) +
            (M10 << 20) + (M11 << 22) + (M12 << 24) + (M13 << 26) +
            (M14 << 28) + (M15 << 30);
  }

#endif // STM32F1XX
}
// namespace gpio

