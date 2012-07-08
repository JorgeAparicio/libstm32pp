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

namespace tim {
  /**
   * @brief Starts the counter.
   */
  template<address::E T>
  void Functions<T>::startCounter()
  {
    *(u32*) (bitband::Peripheral<
        T + registers::cr1::OFFSET,
        registers::cr1::bits::cen::POSITION
    >::address) = 1;
  }

  /**
   * @brief Stops the counter.
   */
  template<address::E T>
  void Functions<T>::stopCounter()
  {
    *(u32*) (bitband::Peripheral<
        T + registers::cr1::OFFSET,
        registers::cr1::bits::cen::POSITION
    >::address) = 0;
  }

  /**
   * @brief Sets the prescaler value of the counter.
   * @note  A value of 0 indicates no prescaler, a value of 1 indicates
   *        prescaling by 2, and so on.
   */
  template<address::E T>
  void Functions<T>::setPrescaler(u16 const psc)
  {
    reinterpret_cast<Registers*>(T)->PSC = psc;
  }

  /**
   * @brief Sets the auto-reload value of the counter.
   * @note  A value of 0 blocks the counter.
   */
  template<address::E T>
  void Functions<T>::setAutoReload(u16 const rld)
  {
    reinterpret_cast<Registers*>(T)->ARR = rld;
  }

  /**
   * @brief Sets the value of the counter.
   */
  template<address::E T>
  void Functions<T>::setCounter(u16 const cnt)
  {
    reinterpret_cast<Registers*>(T)->CNT = cnt;
  }

  /**
   * @brief Returns the current value of the counter.
   */
  template<address::E T>
  u16 Functions<T>::getCounter()
  {
    return reinterpret_cast<Registers*>(T)->CNT;
  }

  /**
   * @brief Generates an update event.
   */
  template<address::E T>
  void Functions<T>::generateUpdate()
  {
    *(u32*) (bitband::Peripheral<
        T + registers::egr::OFFSET,
        registers::egr::bits::ug::POSITION
    >::address) = 1;
  }

  /**
   * @brief Enables the update interrupt.
   */
  template<address::E T>
  void Functions<T>::enableUpdateInterrupt()
  {
    *(u32*) (bitband::Peripheral<
        T + registers::dier::OFFSET,
        registers::dier::bits::uie::POSITION
    >::address) = 1;
  }

  /**
   * @brief Disables the update interrupt.
   */
  template<address::E T>
  void Functions<T>::disableUpdateInterrupt()
  {
    *(u32*) (bitband::Peripheral<
        T + registers::dier::OFFSET,
        registers::dier::bits::uie::POSITION
    >::address) = 0;
  }

  /**
   * @brief Clears the update interrupt flag.
   */
  template<address::E T>
  void Functions<T>::clearUpdateFlag()
  {
    *(u32*) (bitband::Peripheral<
        T + registers::sr::OFFSET,
        registers::sr::bits::uif::POSITION
    >::address) = 0;
  }

  /**
   * @brief Enables the update DMA request.
   */
  template<address::E T>
  void Functions<T>::enableUpdateDma()
  {
    *(u32*) (bitband::Peripheral<
        T + registers::dier::OFFSET,
        registers::dier::bits::ude::POSITION
    >::address) = 1;
  }

  /**
   * @brief Disables the update DMA request.
   */
  template<address::E T>
  void Functions<T>::disableUpdateDma()
  {
    *(u32*) (bitband::Peripheral<
        T + registers::dier::OFFSET,
        registers::dier::bits::ude::POSITION
    >::address) = 0;
  }

  /**
   * @brief Configures the timer to generate a periodic interrupt.
   * @note  This functions doesn't starts the counter.
   */
  template<address::E T>
  template<u32 Frequency>
  void Functions<T>::configurePeriodicInterrupt()
  {
    configureBasicCounter<
        registers::cr1::bits::cen::states::COUNTER_DISABLED,
        registers::cr1::bits::udis::states::UPDATE_EVENT_ENABLED,
        registers::cr1::bits::urs::states::UPDATE_REQUEST_SOURCE_OVERFLOW_UNDERFLOW,
        registers::cr1::bits::opm::states::DONT_STOP_COUNTER_AT_NEXT_UPDATE_EVENT,
        registers::cr1::bits::arpe::states::AUTO_RELOAD_UNBUFFERED
    >();

    setAutoReload(
        (FREQUENCY / Frequency < 65536 ?
            FREQUENCY / Frequency :
            (FREQUENCY / (10 * Frequency) < 65536 ?
                FREQUENCY / (10 * Frequency) :
                (FREQUENCY / (100 * Frequency) < 65536 ?
                    FREQUENCY / (100 * Frequency) :
                    (FREQUENCY / (1000 * Frequency) < 65536 ?
                        FREQUENCY / (1000 * Frequency) :
                        (FREQUENCY / (10000 * Frequency) < 65536 ?
                            FREQUENCY / (10000 * Frequency) :
                            0))))));

    setPrescaler(
        (FREQUENCY / Frequency < 65536 ?
            1 - 1 :
            (FREQUENCY / (10 * Frequency) < 65536 ?
                10 - 1 :
                (FREQUENCY / (100 * Frequency) < 65536 ?
                    100 - 1 :
                    (FREQUENCY / (1000 * Frequency) < 65536 ?
                        1000 - 1 :
                        (FREQUENCY / (10000 * Frequency) < 65536 ?
                                                                   10000 - 1 :
                                                                   0))))));

    enableUpdateInterrupt();
    generateUpdate();
  }

  /**
   * @brief Configures the master mode.
   */
  template<address::E T>
  template<
      registers::cr2::bits::mms::states::E MMS
  >
  void Functions<T>::setMasterMode()
  {
    reinterpret_cast<Registers*>(T)->CR2 &= registers::cr2::bits::mms::MASK;
    reinterpret_cast<Registers*>(T)->CR2 |= MMS;
  }

  /**
   * @brief Configures the timer as a basic counter.
   */
  template<address::E T>
  template<
      registers::cr1::bits::cen::states::E CEN,
      registers::cr1::bits::udis::states::E UDIS,
      registers::cr1::bits::urs::states::E URS,
      registers::cr1::bits::opm::states::E OPM,
      registers::cr1::bits::arpe::states::E ARPE
  >
  void Functions<T>::configureBasicCounter()
  {
    reinterpret_cast<Registers*>(T)->CR1 = CEN + UDIS + URS + OPM + ARPE;
  }

}  // namespace tim
