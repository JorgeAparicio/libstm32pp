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
#include "../include/core/nvic.hpp"

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
    *(u32 volatile*) (bitband::peripheral<
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
    *(u32 volatile*) (bitband::peripheral<
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
    *(u32 volatile*) (bitband::peripheral<
        T + egr::OFFSET,
        egr::ug::POSITION
    >()) = 1;
  }

  /**
   * @brief Enables all the timer interrupts.
   */
  template<Address T>
  void Functions<T>::enableGlobalInterrupt()
  {
    switch (T) {
#if defined XL_DENSITY || \
    defined STM32F2XX || \
    defined STM32F4XX
      case TIM1:
      NVIC::enableInterrupt<
      nvic::irqn::TIM1_BRK_TIM9
      >();
      NVIC::enableInterrupt<
      nvic::irqn::TIM1_CC
      >();
      NVIC::enableInterrupt<
      nvic::irqn::TIM1_TRG_COM_TIM11
      >();
      NVIC::enableInterrupt<
      nvic::irqn::TIM1_UP_TIM10
      >();
      break;
#elif defined VALUE_LINE
      case TIM1:
      NVIC::enableInterrupt<
      nvic::irqn::TIM1_BRK_TIM15
      >();
      NVIC::enableInterrupt<
      nvic::irqn::TIM1_CC
      >();
      NVIC::enableInterrupt<
      nvic::irqn::TIM1_TRG_COM_TIM17
      >();
      NVIC::enableInterrupt<
      nvic::irqn::TIM1_UP_TIM16
      >();
      break;
#else // XL_DENSITY || STM32F2XX || STM32F4XX
      case TIM1:
        NVIC::enableInterrupt<
            nvic::irqn::TIM1_BRK
        >();
        NVIC::enableInterrupt<
            nvic::irqn::TIM1_CC
        >();
        NVIC::enableInterrupt<
            nvic::irqn::TIM1_TRG_COM
        >();
        NVIC::enableInterrupt<
            nvic::irqn::TIM1_UP
        >();
        break;
#endif // XL_DENSITY || STM32F2XX || STM32F4XX
      case TIM2:
        NVIC::enableInterrupt<
            nvic::irqn::TIM2
        >();
        break;
      case TIM3:
        NVIC::enableInterrupt<
            nvic::irqn::TIM3
        >();
        break;
      case TIM4:
        NVIC::enableInterrupt<
            nvic::irqn::TIM4
        >();
        break;
      case TIM5:
        NVIC::enableInterrupt<
            nvic::irqn::TIM5
        >();
        break;
#if defined VALUE_LINE || \
        defined STM32F2XX || \
        defined STM32F4XX
        case TIM6:
        NVIC::enableInterrupt<
        nvic::irqn::TIM6_DAC
        >();
        break;
#else // !STM32F1XX
      case TIM6:
        NVIC::enableInterrupt<
            nvic::irqn::TIM6
        >();
        break;
#endif // !STM32F1XX
      case TIM7:
        NVIC::enableInterrupt<
            nvic::irqn::TIM7
        >();
        break;
#if defined XL_DENSITY || \
  defined STM32F2XX || \
  defined STM32F4XX
        case TIM8:
        NVIC::enableInterrupt<
        nvic::irqn::TIM8_BRK_TIM12
        >();
        NVIC::enableInterrupt<
        nvic::irqn::TIM8_CC
        >();
        NVIC::enableInterrupt<
        nvic::irqn::TIM8_TRG_COM_TIM14
        >();
        NVIC::enableInterrupt<
        nvic::irqn::TIM8_UP_TIM13
        >();
        break;
#elif not defined CONNECTIVITY_LINE && \
 not defined VALUE_LINE
        case TIM8:
        NVIC::enableInterrupt<
        nvic::irqn::TIM8_BRK
        >();
        NVIC::enableInterrupt<
        nvic::irqn::TIM8_CC
        >();
        NVIC::enableInterrupt<
        nvic::irqn::TIM8_TRG_COM
        >();
        NVIC::enableInterrupt<
        nvic::irqn::TIM8_UP
        >();
        break;
#endif // XL_DENSITY || STM32F2XX || STM32F4XX
#ifndef STM32F1XX
        case TIM9:
        NVIC::enableInterrupt<
        nvic::irqn::TIM1_BRK_TIM9
        >();
        break;
        case TIM10:
        NVIC::enableInterrupt<
        nvic::irqn::TIM1_UP_TIM10
        >();
        break;
        case TIM11:
        NVIC::enableInterrupt<
        nvic::irqn::TIM1_TRG_COM_TIM11
        >();
        break;
#endif // !STM32F1XX
#ifndef STM32F1XX
        case TIM12:
        NVIC::enableInterrupt<
        nvic::irqn::TIM8_BRK_TIM12
        >();
        break;
        case TIM13:
        NVIC::enableInterrupt<
        nvic::irqn::TIM8_UP_TIM13
        >();
        break;
        case TIM14:
        NVIC::enableInterrupt<
        nvic::irqn::TIM8_TRG_COM_TIM14
        >();
        break;
#elif defined VALUE_LINE
        case TIM12:
        NVIC::enableInterrupt<
        nvic::irqn::TIM12
        >();
        break;
        case TIM13:
        NVIC::enableInterrupt<
        nvic::irqn::TIM13
        >();
        break;
        case TIM14:
        NVIC::enableInterrupt<
        nvic::irqn::TIM14
        >();
        break;
#endif // !STM32F1XX
#ifdef VALUE_LINE
        case TIM15:
        NVIC::enableInterrupt<
        nvic::irqn::TIM1_BRK_TIM15
        >();
        break;
        case TIM16:
        NVIC::enableInterrupt<
        nvic::irqn::TIM1_UP_TIM16
        >();
        break;
        case TIM17:
        NVIC::enableInterrupt<
        nvic::irqn::TIM1_TRG_COM_TIM17
        >();
        break;
#endif // VALUE_LINE
    }
  }

  /**
   * @brief Disables all the timer interrupts.
   */
  template<Address T>
  void Functions<T>::disableGlobalInterrupt()
  {
    switch (T) {

#if defined XL_DENSITY || \
    defined STM32F2XX || \
    defined STM32F4XX || \
    defined VALUE_LINE
      case TIM1:
      static_assert(T != TIM1,
          "Don't use this function, use NVIC::disableInterrupt() instead.");
      break;
#else
      case TIM1:
        NVIC::disableInterrupt<
            nvic::irqn::TIM1_BRK
        >();
        NVIC::disableInterrupt<
            nvic::irqn::TIM1_CC
        >();
        NVIC::disableInterrupt<
            nvic::irqn::TIM1_TRG_COM
        >();
        NVIC::disableInterrupt<
            nvic::irqn::TIM1_UP
        >();
        break;
#endif
      case TIM2:
        NVIC::disableInterrupt<
            nvic::irqn::TIM2
        >();
        break;
      case TIM3:
        NVIC::disableInterrupt<
            nvic::irqn::TIM3
        >();
        break;
      case TIM4:
        NVIC::disableInterrupt<
            nvic::irqn::TIM4
        >();
        break;
      case TIM5:
        NVIC::disableInterrupt<
            nvic::irqn::TIM5
        >();
        break;
#if defined VALUE_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX
        case TIM6:
        static_assert(T != TIM6,
            "Don't use this function, use NVIC::disableInterrupt() instead.");
        break;
#else // !STM32F1XX
      case TIM6:
        NVIC::disableInterrupt<
            nvic::irqn::TIM6
        >();
        break;
#endif // !STM32F1XX
      case TIM7:
        NVIC::disableInterrupt<
            nvic::irqn::TIM7
        >();
        break;
#if defined XL_DENSITY || \
  defined STM32F2XX || \
  defined STM32F4XX
        case TIM8:
        static_assert(T != TIM8,
            "Don't use this function, use NVIC::disableInterrupt() instead.");
        break;
#elif not defined CONNECTIVITY_LINE && \
 not defined VALUE_LINE
        case TIM8:
        NVIC::disableInterrupt<
        nvic::irqn::TIM8_BRK
        >();
        NVIC::disableInterrupt<
        nvic::irqn::TIM8_CC
        >();
        NVIC::disableInterrupt<
        nvic::irqn::TIM8_TRG_COM
        >();
        NVIC::disableInterrupt<
        nvic::irqn::TIM8_UP
        >();
        break;
#endif // XL_DENSITY || STM32F2XX || STM32F4XX
      case TIM9:
        static_assert(T != TIM9,
            "Don't use this function, use NVIC::disableInterrupt() instead.");
        break;
      case TIM10:
        static_assert(T != TIM10,
            "Don't use this function, use NVIC::disableInterrupt() instead.");
        break;
      case TIM11:
        static_assert(T != TIM11,
            "Don't use this function, use NVIC::disableInterrupt() instead.");
        break;
#ifndef STM32F1XX
        case TIM12:
        static_assert(T != TIM12,
            "Don't use this function, use NVIC::disableInterrupt() instead.");
        break;
        case TIM13:
        static_assert(T != TIM13,
            "Don't use this function, use NVIC::disableInterrupt() instead.");
        break;
        case TIM14:
        static_assert(T != TIM14,
            "Don't use this function, use NVIC::disableInterrupt() instead.");
        break;
#elif defined VALUE_LINE
        case TIM12:
        NVIC::disableInterrupt<
        nvic::irqn::TIM12
        >();
        break;
        case TIM13:
        NVIC::disableInterrupt<
        nvic::irqn::TIM13
        >();
        break;
        case TIM14:
        NVIC::disableInterrupt<
        nvic::irqn::TIM14
        >();
        break;
#endif // !STM32F1XX
#ifdef VALUE_LINE
        case TIM15:
        static_assert(T != TIM15,
            "Don't use this function, use NVIC::disableInterrupt() instead.");
        break;
        case TIM16:
        static_assert(T != TIM16,
            "Don't use this function, use NVIC::disableInterrupt() instead.");
        break;
        case TIM17:
        static_assert(T != TIM17,
            "Don't use this function, use NVIC::disableInterrupt() instead.");
        break;
#endif // VALUE_LINE
    }
  }

  /**
   * @brief Enables the update interrupt.
   */
  template<Address T>
  void Functions<T>::enableUpdateInterrupt()
  {
    *(u32 volatile*) (bitband::peripheral<
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
    *(u32 volatile*) (bitband::peripheral<
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
    *(u32 volatile*) (bitband::peripheral<
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
    *(u32 volatile*) (bitband::peripheral<
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
    *(u32 volatile*) (bitband::peripheral<
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
    return *(bool volatile*) (bitband::peripheral<
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
