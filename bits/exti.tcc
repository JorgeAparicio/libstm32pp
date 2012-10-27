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

#include "../include/core/nvic.hpp"

namespace exti {
  /**
   * @brief Clears the pending interrupt/event flag.
   */
  template<u8 N>
  void Functions<N>::clearPendingFlag()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + pr::OFFSET,
        N
    >()) = 1;
  }

  /**
   * @brief Disables both the events and the interrupts.
   */
  template<u8 N>
  void Functions<N>::disableAll()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + emr::OFFSET,
        N
    >()) = 0;

    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + imr::OFFSET,
        N
    >()) = 0;

    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + ftsr::OFFSET,
        N
    >()) = 0;

    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + rtsr::OFFSET,
        N
    >()) = 0;

    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + swier::OFFSET,
        N
    >()) = 0;
  }

  /**
   * @brief Disables the event.
   */
  template<u8 N>
  void Functions<N>::disableEvent()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + emr::OFFSET,
        N
    >()) = 0;

    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + ftsr::OFFSET,
        N
    >()) = 0;

    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + rtsr::OFFSET,
        N
    >()) = 0;

    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + swier::OFFSET,
        N
    >()) = 0;
  }

  /**
   * @brief Disables the interrupt.
   */
  template<u8 N>
  void Functions<N>::disableInterrupt()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + imr::OFFSET,
        N
    >()) = 0;

    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + ftsr::OFFSET,
        N
    >()) = 0;

    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + rtsr::OFFSET,
        N
    >()) = 0;

    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + swier::OFFSET,
        N
    >()) = 0;
  }

  template<u8 N>
  void Functions<N>::unmaskInterrupt()
  {
    static_assert(N < 5, "Warning: EXTI5-9 are grouped and EXTI10-15 are also "
        "grouped. Unmasking one of the member will unmask the whole group. "
        "You can comment this static_assert now, you have been warned.");
    switch (N) {
      case 0:
        NVIC::enableIrq<nvic::irqn::EXTI0>();
        break;
      case 1:
        NVIC::enableIrq<nvic::irqn::EXTI1>();
        break;
      case 2:
        NVIC::enableIrq<nvic::irqn::EXTI2>();
        break;
      case 3:
        NVIC::enableIrq<nvic::irqn::EXTI3>();
        break;
      case 4:
        NVIC::enableIrq<nvic::irqn::EXTI4>();
        break;
      case 5:
      case 6:
      case 7:
      case 8:
      case 9:
        NVIC::enableIrq<nvic::irqn::EXTI9_5>();
        break;
      case 10:
      case 11:
      case 12:
      case 13:
      case 14:
      case 15:
        NVIC::enableIrq<nvic::irqn::EXTI15_10>();
        break;
    }
  }

  template<u8 N>
  void Functions<N>::maskInterrupt()
  {
    static_assert(N < 5, "Warning: EXTI5-9 are grouped and EXTI10-15 are also "
        "grouped. Masking one of the member will mask the whole group. "
        "You can comment this static_assert now, you have been warned.");
    switch (N) {
      case 0:
        NVIC::disableIrq<nvic::irqn::EXTI0>();
        break;
      case 1:
        NVIC::disableIrq<nvic::irqn::EXTI1>();
        break;
      case 2:
        NVIC::disableIrq<nvic::irqn::EXTI2>();
        break;
      case 3:
        NVIC::disableIrq<nvic::irqn::EXTI3>();
        break;
      case 4:
        NVIC::disableIrq<nvic::irqn::EXTI4>();
        break;
      case 5:
      case 6:
      case 7:
      case 8:
      case 9:
        NVIC::disableIrq<nvic::irqn::EXTI9_5>();
        break;
      case 10:
      case 11:
      case 12:
      case 13:
      case 14:
      case 15:
        NVIC::disableIrq<nvic::irqn::EXTI15_10>();
        break;
    }
  }

  /**
   * @brief Enables the hardware event by falling edge.
   */
  template<u8 N>
  void Functions<N>::enableHardwareEventByFallingEdge()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + emr::OFFSET,
        N
    >()) = 1;

    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + ftsr::OFFSET,
        N
    >()) = 1;
  }

  /**
   * @brief Enables the hardware event by rising edge.
   */
  template<u8 N>
  void Functions<N>::enableHardwareEventByRisingEdge()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + emr::OFFSET,
        N
    >()) = 1;

    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + rtsr::OFFSET,
        N
    >()) = 1;
  }

  /**
   * @brief Enables the hardware interrupt by falling edge.
   */
  template<u8 N>
  void Functions<N>::enableHardwareInterruptByFallingEdge()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + imr::OFFSET,
        N
    >()) = 1;

    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + ftsr::OFFSET,
        N
    >()) = 1;
  }

  /**
   * @brief Enables the hardware interrupt by rising edge.
   */
  template<u8 N>
  void Functions<N>::enableHardwareInterruptByRisingEdge()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + imr::OFFSET,
        N
    >()) = 1;

    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + rtsr::OFFSET,
        N
    >()) = 1;
  }

  /**
   * @brief Enables the software event.
   */
  template<u8 N>
  void Functions<N>::enableSoftwareEvent()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + emr::OFFSET,
        N
    >()) = 1;

    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + swier::OFFSET,
        N
    >()) = 1;
  }

  /**
   * @brief Enables the software interrupt.
   */
  template<u8 N>
  void Functions<N>::enableSoftwareInterrupt()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + emr::OFFSET,
        N
    >()) = 1;

    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + swier::OFFSET,
        N
    >()) = 1;
  }
}  // namespace exti
