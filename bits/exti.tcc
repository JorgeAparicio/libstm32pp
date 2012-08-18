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

namespace exti {
  /**
   * @brief Clears the pending interrupt/event flag.
   */
  template<u8 N>
  void Functions<N>::clearPendingFlag()
  {
    static_assert(N < 23, "There are only 23 (0-22) external interrupt lines");

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::pr::OFFSET,
        N
    >()) = 1;
  }

  /**
   * @brief Disables both the events and the interrupts.
   */
  template<u8 N>
  void Functions<N>::disableAll()
  {
    static_assert(N < 23, "There are only 23 (0-22) external interrupt lines");

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::emr::OFFSET,
        N
    >()) = 0;

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::imr::OFFSET,
        N
    >()) = 0;

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::ftsr::OFFSET,
        N
    >()) = 0;

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::rtsr::OFFSET,
        N
    >()) = 0;

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::swier::OFFSET,
        N
    >()) = 0;
  }

  /**
   * @brief Disables the event.
   */
  template<u8 N>
  void Functions<N>::disableEvent()
  {
    static_assert(N < 23, "There are only 23 (0-22) external interrupt lines");

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::emr::OFFSET,
        N
    >()) = 0;

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::ftsr::OFFSET,
        N
    >()) = 0;

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::rtsr::OFFSET,
        N
    >()) = 0;

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::swier::OFFSET,
        N
    >()) = 0;
  }

  /**
   * @brief Disables the interrupt.
   */
  template<u8 N>
  void Functions<N>::disableInterrupt()
  {
    static_assert(N < 23, "There are only 23 (0-22) external interrupt lines");

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::imr::OFFSET,
        N
    >()) = 0;

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::ftsr::OFFSET,
        N
    >()) = 0;

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::rtsr::OFFSET,
        N
    >()) = 0;

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::swier::OFFSET,
        N
    >()) = 0;
  }

  /**
   * @brief Enables the hardware event by falling edge.
   */
  template<u8 N>
  void Functions<N>::enableHardwareEventByFallingEdge()
  {
    static_assert(N < 23, "There are only 23 (0-22) external interrupt lines");

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::emr::OFFSET,
        N
    >()) = 1;

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::ftsr::OFFSET,
        N
    >()) = 1;
  }

  /**
   * @brief Enables the hardware event by rising edge.
   */
  template<u8 N>
  void Functions<N>::enableHardwareEventByRisingEdge()
  {
    static_assert(N < 23, "There are only 23 (0-22) external interrupt lines");

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::emr::OFFSET,
        N
    >()) = 1;

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::rtsr::OFFSET,
        N
    >()) = 1;
  }

  /**
   * @brief Enables the hardware interrupt by falling edge.
   */
  template<u8 N>
  void Functions<N>::enableHardwareInterruptByFallingEdge()
  {
    static_assert(N < 23, "There are only 23 (0-22) external interrupt lines");

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::imr::OFFSET,
        N
    >()) = 1;

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::ftsr::OFFSET,
        N
    >()) = 1;
  }

  /**
   * @brief Enables the hardware interrupt by rising edge.
   */
  template<u8 N>
  void Functions<N>::enableHardwareInterruptByRisingEdge()
  {
    static_assert(N < 23, "There are only 23 (0-22) external interrupt lines");

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::imr::OFFSET,
        N
    >()) = 1;

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::rtsr::OFFSET,
        N
    >()) = 1;
  }

  /**
   * @brief Enables the software event.
   */
  template<u8 N>
  void Functions<N>::enableSoftwareEvent()
  {
    static_assert(N < 23, "There are only 23 (0-22) external interrupt lines");

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::emr::OFFSET,
        N
    >()) = 1;

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::swier::OFFSET,
        N
    >()) = 1;
  }

  /**
   * @brief Enables the software interrupt.
   */
  template<u8 N>
  void Functions<N>::enableSoftwareInterrupt()
  {
    static_assert(N < 23, "There are only 23 (0-22) external interrupt lines");

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::emr::OFFSET,
        N
    >()) = 1;

    *(u32*) (bitband::peripheral<
        ADDRESS + registers::swier::OFFSET,
        N
    >()) = 1;
  }
}  // namespace exti
