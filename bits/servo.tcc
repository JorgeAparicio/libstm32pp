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

namespace servo {
  /**
   * @brief Constructor
   */
  template<
      tim::address::E P,
      u32 F,
      tim::address::E D,
      u16 M,
      u8 N
  >
  Functions<P, F, D, M, N>::Functions()
  {
    for (u8 i = 0; i < N; i++)
      value[i] = 0;
  }

  /**
   * @brief Constructor
   */
  template<
      tim::address::E P,
      u32 F,
      tim::address::E D,
      u16 M,
      u8 N
  >
  void Functions<P, F, D, M, N>::setPin(u8 const index, u32* const pinAddres)
  {
    pin[index] = pinAddres;
  }

  /**
   * @brief Configures the controller for operation.
   * @note  Only call this function once.
   */
  template<
      tim::address::E P,
      u32 F,
      tim::address::E D,
      u16 M,
      u8 N
  >
  void Functions<P, F, D, M, N>::initialize()
  {
    PeriodTimer::template configurePeriodicInterrupt<F>();

    DutyCycleTimer::template configureBasicCounter<
        tim::registers::cr1::bits::cen::states::
        COUNTER_DISABLED,
        tim::registers::cr1::bits::udis::states::
        UPDATE_EVENT_ENABLED,
        tim::registers::cr1::bits::urs::states::
        UPDATE_REQUEST_SOURCE_OVERFLOW_UNDERFLOW,
        tim::registers::cr1::bits::opm::states::
        DONT_STOP_COUNTER_AT_NEXT_UPDATE_EVENT,
        tim::registers::cr1::bits::arpe::states::
        AUTO_RELOAD_UNBUFFERED
    >();
    // Timer resolution: 1us
    DutyCycleTimer::setPrescaler((DutyCycleTimer::FREQUENCY / 1000000) - 1);
    DutyCycleTimer::enableUpdateInterrupt();
    DutyCycleTimer::generateUpdate();
  }

  /**
   * @brief Starts the controller.
   */
  template<
      tim::address::E P,
      u32 F,
      tim::address::E D,
      u16 M,
      u8 N
  >
  void Functions<P, F, D, M, N>::start()
  {
    servoIndex = 0;

    for (u8 i = 0; i < N; i++) {
      if (pin[i] == 0)
        return;

      sortedIndices[i] = i;
    }

    PeriodTimer::startCounter();
  }

  /**
   * @brief Stops the controller.
   */
  template<
      tim::address::E P,
      u32 F,
      tim::address::E D,
      u16 M,
      u8 N
  >
  void Functions<P, F, D, M, N>::stop()
  {
    PeriodTimer::stopCounter();
  }

  /**
   * @brief Loads values into the servo controller buffer
   */
  template<
      tim::address::E P,
      u32 F,
      tim::address::E D,
      u16 M,
      u8 N
  >
  void Functions<P, F, D, M, N>::load(s16 const (&newValues)[N])
  {
    for (int i = 0; i < N; i++)
      buffer[i] = newValues[i];
  }

  /**
   * @brief Call this function on the PeriodTimer interrupt.
   * @note  This function clears the interrupt flag.
   */
  template<
      tim::address::E P,
      u32 F,
      tim::address::E D,
      u16 M,
      u8 N
  >
  void Functions<P, F, D, M, N>::onPeriodTimerInterrupt()
  {
    PeriodTimer::clearUpdateFlag();

    // Sort
    u8 n = N;
    while (n != 0) {
      u8 newN = 0;
      for (u8 i = 1; i < n; i++) {
        if (value[sortedIndices[i - 1]] > value[sortedIndices[i]]) {
          u8 temp;
          temp = sortedIndices[i - 1];
          sortedIndices[i - 1] = sortedIndices[i];
          sortedIndices[i] = temp;
          newN = i;
        }
      }
      n = newN;
    }

    DutyCycleTimer::setAutoReload(M + value[sortedIndices[0]]);

    for (u8 i = 0; i < N; i++)
      *(pin[i]) = 1;

    DutyCycleTimer::startCounter();

    servoIndex = 0;
  }

  /**
   * @brief Call this function on the DutyCycleTimer interrupt.
   * @note  This function clears the interrupt flag.
   */
  template<
      tim::address::E P,
      u32 F,
      tim::address::E D,
      u16 M,
      u8 N
  >
  void Functions<P, F, D, M, N>::onDutyCycleTimerInterrupt()
  {
    DutyCycleTimer::stopCounter();
    DutyCycleTimer::clearUpdateFlag();

    do {
      *(pin[sortedIndices[servoIndex]]) = 0;
      servoIndex++;
    } while (((value[sortedIndices[servoIndex]] -
        value[sortedIndices[servoIndex - 1]]) == 0) &&
        servoIndex != N);

    if (servoIndex == N) {
      for (int i = 0; i < N; i++)
        value[i] = buffer[i];
    } else {
      DutyCycleTimer::setAutoReload(value[sortedIndices[servoIndex]] -
          value[sortedIndices[servoIndex - 1]]);

      DutyCycleTimer::startCounter();
    }
  }
}  // namespace servo
