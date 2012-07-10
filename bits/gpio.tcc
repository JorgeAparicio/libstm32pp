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

namespace gpio {
#ifdef STM32F1XX
   /**
    * @brief Sets the pin to 1, if it is in any of the output modes.
    */
   template<address::E P, u8 N>
   void Pin<P, N>::setHigh()
   {
     reinterpret_cast<Registers *>(P)->BSRR = 1 << N;
   }

   /**
    * @brief Sets the pin to 0, if it is in any of the output modes.
    */
   template<address::E P, u8 N>
   void Pin<P, N>::setLow()
   {
     reinterpret_cast<Registers *>(P)->BRR = 1 << N;
   }

   /**
    * @brief Outputs the desired logic state through the pin.
    */
   template<address::E P, u8 N>
   void Pin<P, N>::output(u32 const value)
   {
     *(u32*) (OUT_ADDRESS) = value;
   }

   /**
    * @brief Gets the logic state of the pin.
    */
   template<address::E P, u8 N>
   u32 Pin<P, N>::input()
   {
     return *(u32*) (IN_ADDRESS);
   }

   /**
    * @brief Enables the pull-up circuit on this pin. (MODE_INPUT_PULL_X only).
    */
   template<address::E P, u8 N>
   void Pin<P, N>::pullUp()
   {
     setHigh();
   }

   /**
    * @brief Enables the pull-down circuit on this pin. (MODE_INPUT_PULL_X only).
    */
   template<address::E P, u8 N>
   void Pin<P, N>::pullDown()
   {
     setLow();
   }

   /**
    * @brief Returns true if the pin is high.
    */
   template<address::E P, u8 N>
   bool Pin<P, N>::isHigh()
   {
     return *(bool*) (bitband::Peripheral<
         P + registers::idr::OFFSET,
         N
         >::address);
   }

  /**
   * @brief Changes the I/O mode.
   */
  template<address::E P, u8 N>
  template<registers::cr::states::E CR>
  void Pin<P, N>::setMode()
  {
    reinterpret_cast<Registers*>(P)->CR[N > 7 ? 1 : 0] &=
    ~(registers::cr::MASK <<
        registers::cr::POSITION * (N > 7 ? (N - 8) : N));

    reinterpret_cast<Registers*>(P)->CR[N > 7 ? 1 : 0] |=
    CR << registers::cr::POSITION * (N > 7 ? (N - 8) : N);
  }

  /**
   * @brief Configures the pins 0-7 of the port.
   * @note  PB3 and PB4 are JTAG pins, they should be configured as ALTERNATE
   *        if using the JTAG.
   */
  template<address::E P>
  template<
  registers::cr::states::E P0,
  registers::cr::states::E P1,
  registers::cr::states::E P2,
  registers::cr::states::E P3,
  registers::cr::states::E P4,
  registers::cr::states::E P5,
  registers::cr::states::E P6,
  registers::cr::states::E P7
  >
  void Port<P>::configureLowerPins()
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
  template<address::E P>
  template<
  registers::cr::states::E P8,
  registers::cr::states::E P9,
  registers::cr::states::E P10,
  registers::cr::states::E P11,
  registers::cr::states::E P12,
  registers::cr::states::E P13,
  registers::cr::states::E P14,
  registers::cr::states::E P15
  >
  void Port<P>::configureHigherPins()
  {
    reinterpret_cast<Registers *>(P)->CR[1] =
    (P8 << 0) + (P9 << 4) + (P10 << 8) + (P11 << 12) +
    (P12 << 16) + (P13 << 20) + (P14 << 24) + (P15 << 28);
  }

  /**
   * @brief Outputs the desired logic state on the port.
   */
  template<address::E P>
  void Port<P>::setValue(u32 const value)
  {
    reinterpret_cast<Registers *>(P)->ODR =
    value;
  }

  /**
   * @brief Gets the logic state of the port.
   */
  template<address::E P>
  u32 Port<P>::getValue()
  {
    return reinterpret_cast<Registers *>(P)->IDR;
  }

#else // STM32F1XX
  /**
   * @brief Sets the pin to 1, if it is in output mode.
   */
  template<address::E P, u8 N>
  void Pin<P, N>::setHigh()
  {
    reinterpret_cast<Registers *>(P)->BSRR = (1 << N);
  }

  /**
   * @brief Sets the pin to 0, if it is in output mode.
   */
  template<address::E P, u8 N>
  void Pin<P, N>::setLow()
  {
    reinterpret_cast<Registers *>(P)->BSRR = (1 << N) << 16;
  }

  /**
   * @brief Outputs the logic state on the pin.
   */
  template<address::E P, u8 N>
  void Pin<P, N>::output(u32 const value)
  {
    *(u32*) (OUT_ADDRESS) = value;
  }

  /**
   * @brief Gets the logic state of the pin.
   */
  template<address::E P, u8 N>
  u32 Pin<P, N>::input()
  {
    return *(u32*) (IN_ADDRESS);
  }

  /**
   * @brief Returns true if the pin is high.
   */
  template<address::E P, u8 N>
  bool Pin<P, N>::isHigh()
  {
    return *(bool*) (bitband::Peripheral<
        P + registers::idr::OFFSET,
        N
    >::address);
  }

  /**
   * @brief Selects the I/O mode.
   */
  template<address::E P, u8 N>
  template<registers::moder::states::E MODER>
  void Pin<P, N>::setMode()
  {
    reinterpret_cast<Registers *>(P)->MODER &=
        ~(registers::moder::MASK << (2 * N));
    reinterpret_cast<Registers *>(P)->MODER |=
        (MODER << (2 * N));
  }

  /**
   * @brief Selects the output mode.
   */
  template<address::E P, u8 N>
  template<registers::otyper::states::E OTYPER>
  void Pin<P, N>::setOutputMode()
  {
    *((u32*) (bitband::Peripheral<
        P + registers::otyper::OFFSET,
        N
    >::address)) = OTYPER;
  }

  /**
   * @brief Selects the output speed.
   */
  template<address::E P, u8 N>
  template<registers::ospeedr::states::E OSPEEDR>
  void Pin<P, N>::setSpeed()
  {
    reinterpret_cast<Registers *>(P)->OSPEEDR &=
        ~(registers::ospeedr::MASK << (2 * N));
    reinterpret_cast<Registers *>(P)->OSPEEDR |=
        (OSPEEDR << (2 * N));
  }

  /**
   * @brief Selects between pull-up, pull-down or no pull-up/pull-down.
   */
  template<address::E P, u8 N>
  template<registers::pupdr::states::E PUPDR>
  void Pin<P, N>::setPullMode()
  {
    reinterpret_cast<Registers *>(P)->PUPDR &=
        ~(registers::pupdr::MASK << (2 * N));
    reinterpret_cast<Registers *>(P)->PUPDR |=
        (PUPDR << (2 * N));
  }

  /**
   * @brief Selects the alternate function.
   */
  template<address::E P, u8 N>
  template<registers::afr::states::E AFR>
  void Pin<P, N>::setAlternateFunction()
  {
    reinterpret_cast<Registers*>(P)->AFR[N > 7 ? 1 : 0] &=
        ~(registers::afr::MASK <<
            registers::afr::POSITION * (N > 7 ? (N - 8) : N));

    reinterpret_cast<Registers*>(P)->AFR[N > 7 ? 1 : 0] |=
        AFR << (registers::afr::POSITION * (N > 7 ? (N - 8) : N));
  }

  /**
   * @brief Outputs the logic states on the port.
   */
  template<address::E P>
  void Port<P>::output(u16 const value)
  {
    reinterpret_cast<Registers*>(P)->ODR = value;
  }

  /**
   * @brief Gets the logic state of the port.
   */
  template<address::E P>
  u16 Port<P>::input()
  {
    return reinterpret_cast<Registers*>(P)->IDR;
  }

  /**
   * @brief Configures the mode of the port.
   * @note  PB3, PB4, PA13, PA14 and PA15 are JTAG pins, they should be
   *        configured as ALTERNATE if using the JTAG.
   */
  template<address::E P>
  template<
      registers::moder::states::E M0,
      registers::moder::states::E M1,
      registers::moder::states::E M2,
      registers::moder::states::E M3,
      registers::moder::states::E M4,
      registers::moder::states::E M5,
      registers::moder::states::E M6,
      registers::moder::states::E M7,
      registers::moder::states::E M8,
      registers::moder::states::E M9,
      registers::moder::states::E M10,
      registers::moder::states::E M11,
      registers::moder::states::E M12,
      registers::moder::states::E M13,
      registers::moder::states::E M14,
      registers::moder::states::E M15
  >
  void Port<P>::setModes()
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
  template<address::E P>
  template<
      registers::otyper::states::E M0,
      registers::otyper::states::E M1,
      registers::otyper::states::E M2,
      registers::otyper::states::E M3,
      registers::otyper::states::E M4,
      registers::otyper::states::E M5,
      registers::otyper::states::E M6,
      registers::otyper::states::E M7,
      registers::otyper::states::E M8,
      registers::otyper::states::E M9,
      registers::otyper::states::E M10,
      registers::otyper::states::E M11,
      registers::otyper::states::E M12,
      registers::otyper::states::E M13,
      registers::otyper::states::E M14,
      registers::otyper::states::E M15
  >
  void Port<P>::setOutputTypes()
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
  template<address::E P>
  template<
      registers::ospeedr::states::E M0,
      registers::ospeedr::states::E M1,
      registers::ospeedr::states::E M2,
      registers::ospeedr::states::E M3,
      registers::ospeedr::states::E M4,
      registers::ospeedr::states::E M5,
      registers::ospeedr::states::E M6,
      registers::ospeedr::states::E M7,
      registers::ospeedr::states::E M8,
      registers::ospeedr::states::E M9,
      registers::ospeedr::states::E M10,
      registers::ospeedr::states::E M11,
      registers::ospeedr::states::E M12,
      registers::ospeedr::states::E M13,
      registers::ospeedr::states::E M14,
      registers::ospeedr::states::E M15
  >
  void Port<P>::setOutputSpeeds()
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
  template<address::E P>
  template<
      registers::pupdr::states::E M0,
      registers::pupdr::states::E M1,
      registers::pupdr::states::E M2,
      registers::pupdr::states::E M3,
      registers::pupdr::states::E M4,
      registers::pupdr::states::E M5,
      registers::pupdr::states::E M6,
      registers::pupdr::states::E M7,
      registers::pupdr::states::E M8,
      registers::pupdr::states::E M9,
      registers::pupdr::states::E M10,
      registers::pupdr::states::E M11,
      registers::pupdr::states::E M12,
      registers::pupdr::states::E M13,
      registers::pupdr::states::E M14,
      registers::pupdr::states::E M15
  >
  void Port<P>::setPullModes()
  {
    reinterpret_cast<Registers *>(P)->PUPDR =
        (M0 << 0) + (M1 << 2) + (M2 << 4) + (M3 << 6) + (M4 << 8) +
            (M5 << 10) + (M6 << 12) + (M7 << 14) + (M8 << 16) + (M9 << 18) +
            (M10 << 20) + (M11 << 22) + (M12 << 24) + (M13 << 26) +
            (M14 << 28) + (M15 << 30);
  }

#endif // STM32F1XX
}  // namespace gpio

