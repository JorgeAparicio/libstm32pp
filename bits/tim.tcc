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

#include "../include/bitband.hpp"
#include "../include/peripheral/rcc.hpp"

namespace tim {
  /**
   * @brief Enables the timer's clock.
   */
  template<Address T>
  void Functions<T>::enableClock()
  {
    switch (T) {
      case TIM1:
        RCC::enableClocks<rcc::apb2enr::TIM1>();
        break;
      case TIM2:
        RCC::enableClocks<rcc::apb1enr::TIM2>();
        break;
      case TIM3:
        RCC::enableClocks<rcc::apb1enr::TIM3>();
        break;
      case TIM4:
        RCC::enableClocks<rcc::apb1enr::TIM4>();
        break;
      case TIM5:
        RCC::enableClocks<rcc::apb1enr::TIM5>();
        break;
      case TIM6:
        RCC::enableClocks<rcc::apb1enr::TIM6>();
        break;
      case TIM7:
        RCC::enableClocks<rcc::apb1enr::TIM7>();
        break;
      case TIM8:
        RCC::enableClocks<rcc::apb2enr::TIM8>();
        break;
      case TIM9:
        RCC::enableClocks<rcc::apb2enr::TIM9>();
        break;
      case TIM10:
        RCC::enableClocks<rcc::apb2enr::TIM10>();
        break;
      case TIM11:
        RCC::enableClocks<rcc::apb2enr::TIM11>();
        break;
      case TIM12:
        RCC::enableClocks<rcc::apb1enr::TIM12>();
        break;
      case TIM13:
        RCC::enableClocks<rcc::apb1enr::TIM13>();
        break;
      case TIM14:
        RCC::enableClocks<rcc::apb1enr::TIM14>();
        break;
#ifdef VALUE_LINE
        case TIM15:
        RCC::enableClocks<rcc::apb2enr::TIM15>();
        break;
        case TIM16:
        RCC::enableClocks<rcc::apb2enr::TIM16>();
        break;
        case TIM17:
        RCC::enableClocks<rcc::apb2enr::TIM17>();
        break;
#endif // VALUE_LINE
    }
  }

  /**
   * @brief Disables the timer's clock.
   */
  template<Address T>
  void Functions<T>::disableClock()
  {
    switch (T) {
      case TIM1:
        RCC::disableClocks<rcc::apb2enr::TIM1>();
        break;
      case TIM2:
        RCC::disableClocks<rcc::apb1enr::TIM2>();
        break;
      case TIM3:
        RCC::disableClocks<rcc::apb1enr::TIM3>();
        break;
      case TIM4:
        RCC::disableClocks<rcc::apb1enr::TIM4>();
        break;
      case TIM5:
        RCC::disableClocks<rcc::apb1enr::TIM5>();
        break;
      case TIM6:
        RCC::disableClocks<rcc::apb1enr::TIM6>();
        break;
      case TIM7:
        RCC::disableClocks<rcc::apb1enr::TIM7>();
        break;
      case TIM8:
        RCC::disableClocks<rcc::apb2enr::TIM8>();
        break;
      case TIM9:
        RCC::disableClocks<rcc::apb2enr::TIM9>();
        break;
      case TIM10:
        RCC::disableClocks<rcc::apb2enr::TIM10>();
        break;
      case TIM11:
        RCC::disableClocks<rcc::apb2enr::TIM11>();
        break;
      case TIM12:
        RCC::disableClocks<rcc::apb1enr::TIM12>();
        break;
      case TIM13:
        RCC::disableClocks<rcc::apb1enr::TIM13>();
        break;
      case TIM14:
        RCC::disableClocks<rcc::apb1enr::TIM14>();
        break;
#ifdef VALUE_LINE
        case TIM15:
        RCC::disableClocks<rcc::apb2enr::TIM15>();
        break;
        case TIM16:
        RCC::disableClocks<rcc::apb2enr::TIM16>();
        break;
        case TIM17:
        RCC::disableClocks<rcc::apb2enr::TIM17>();
        break;
#endif // VALUE_LINE
    }
  }

  /**
   * @brief Starts the counter.
   */
  template<Address T>
  void Functions<T>::startCounter()
  {
    *(volatile u32*) (bitband::peripheral<
        T + cr1::OFFSET,
        cr1::cen::POSITION
    >()) = 1;
  }

  /**
   * @brief Stops the counter.
   */
  template<Address T>
  void Functions<T>::stopCounter()
  {
    *(volatile u32*) (bitband::peripheral<
        T + cr1::OFFSET,
        cr1::cen::POSITION
    >()) = 0;
  }

  /**
   * @brief Configures the prescaler, so the counter counts in microseconds.
   */
  template<Address T>
  void Functions<T>::setMicroSecondResolution()
  {
    enum {
      PSC = (FREQUENCY / 1000000) - 1
    };

    static_assert(PSC < 65536,
        "Can't configure the timer to count in microseconds.");

    setPrescaler(PSC);
  }

  /**
   * @brief Configures the prescaler, so the counter counts in miliseconds.
   */
  template<Address T>
  void Functions<T>::setMiliSecondResolution()
  {
    enum {
      PSC = (FREQUENCY / 1000) - 1
    };

    static_assert(PSC < 65536,
        "Can't configure the timer to count in miliseconds.");

    setPrescaler(PSC);
  }

  /**
   * @brief Waits for <N> counts.
   * @note  Timer must be configured to generate an update request only on
   *        overflow or underflow.
   * @note  If N = 0, the processor will be trapped in an infinite loop.
   */
  template<Address T>
  void Functions<T>::delay(u16 const N)
  {
    setAutoReload(N);
    generateUpdate();
    clearUpdateFlag();
    startCounter();
    while (!hasUpdateEventOccurred()) {
    }
    stopCounter();
  }

  /**
   * @brief Sets the prescaler value of the counter.
   * @note  A value of 0 indicates no prescaler, a value of 1 indicates
   *        prescaling by 2, and so on.
   */
  template<Address T>
  void Functions<T>::setPrescaler(u16 const psc)
  {
    reinterpret_cast<Registers*>(T)->PSC = psc;
  }

  /**
   * @brief Sets the auto-reload value of the counter.
   * @note  A value of 0 blocks the counter.
   */
  template<Address T>
  void Functions<T>::setAutoReload(u16 const rld)
  {
    reinterpret_cast<Registers*>(T)->ARR = rld;
  }

  /**
   * @brief Sets the value of the counter.
   */
  template<Address T>
  void Functions<T>::setCounter(u16 const cnt)
  {
    reinterpret_cast<Registers*>(T)->CNT = cnt;
  }

  /**
   * @brief Returns the current value of the counter.
   */
  template<Address T>
  u16 Functions<T>::getCounter()
  {
    return reinterpret_cast<Registers*>(T)->CNT;
  }

  /**
   * @brief Generates an update event.
   */
  template<Address T>
  void Functions<T>::generateUpdate()
  {
    *(volatile u32*) (bitband::peripheral<
        T + egr::OFFSET,
        egr::ug::POSITION
    >()) = 1;
  }

  /**
   * @brief Enables the update interrupt.
   */
  template<Address T>
  void Functions<T>::enableUpdateInterrupt()
  {
    *(volatile u32*) (bitband::peripheral<
        T + dier::OFFSET,
        dier::uie::POSITION
    >()) = 1;
  }

  /**
   * @brief Disables the update interrupt.
   */
  template<Address T>
  void Functions<T>::disableUpdateInterrupt()
  {
    *(volatile u32*) (bitband::peripheral<
        T + dier::OFFSET,
        dier::uie::POSITION
    >()) = 0;
  }

  /**
   * @brief Clears the update interrupt flag.
   */
  template<Address T>
  void Functions<T>::clearUpdateFlag()
  {
    *(volatile u32*) (bitband::peripheral<
        T + sr::OFFSET,
        sr::uif::POSITION
    >()) = 0;
  }

  /**
   * @brief Enables the update DMA request.
   */
  template<Address T>
  void Functions<T>::enableUpdateDma()
  {
    *(volatile u32*) (bitband::peripheral<
        T + dier::OFFSET,
        dier::ude::POSITION
    >()) = 1;
  }

  /**
   * @brief Disables the update DMA request.
   */
  template<Address T>
  void Functions<T>::disableUpdateDma()
  {
    *(volatile u32*) (bitband::peripheral<
        T + dier::OFFSET,
        dier::ude::POSITION
    >()) = 0;
  }

  /**
   * @brief Returns true if an update event has occurred.
   */
  template<Address T>
  bool Functions<T>::hasUpdateEventOccurred()
  {
    return *(volatile bool*) (bitband::peripheral<
        T + sr::OFFSET,
        sr::uif::POSITION
    >());
  }

  /**
   * @brief Configures the timer to generate a periodic interrupt.
   * @note  This functions doesn't starts the counter.
   */
  template<Address T>
  template<u32 Frequency>
  void Functions<T>::configurePeriodicInterrupt()
  {
    configureBasicCounter(
        cr1::cen::COUNTER_DISABLED,
        cr1::udis::UPDATE_EVENT_ENABLED,
        cr1::urs::UPDATE_REQUEST_SOURCE_OVERFLOW_UNDERFLOW,
        cr1::opm::DONT_STOP_COUNTER_AT_NEXT_UPDATE_EVENT,
        cr1::arpe::AUTO_RELOAD_UNBUFFERED);

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
  template<Address T>
  void Functions<T>::setMasterMode(cr2::mms::States MMS)
  {
    reinterpret_cast<Registers*>(T)->CR2 &= cr2::mms::MASK;
    reinterpret_cast<Registers*>(T)->CR2 |= MMS;
  }

  /**
   * @brief Configures the timer as a basic counter.
   */
  template<Address T>
  void Functions<T>::configureBasicCounter(
      cr1::cen::States CEN,
      cr1::udis::States UDIS,
      cr1::urs::States URS,
      cr1::opm::States OPM,
      cr1::arpe::States ARPE)
  {
    reinterpret_cast<Registers*>(T)->CR1 = CEN + UDIS + URS + OPM + ARPE;
  }

}  // namespace tim
