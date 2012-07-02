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
  namespace basic {
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
     */
    template<address::E T>
    void Functions<T>::setPrescaler(u16 const psc)
    {
      reinterpret_cast<Registers*>(T)->PSC = psc;
    }

    /**
     * @brief Sets the autoreload value of the counter.
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
    void Functions<T>::enableInterrupt()
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
    void Functions<T>::disableInterrupt()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::uie::POSITION
      >::address) = 0;
    }

    /**
     * @brief Clears the interrupt flag.
     */
    template<address::E T>
    void Functions<T>::clearFlag()
    {
//      *(u32*) (bitband::Peripheral<
//          T + registers::sr::OFFSET,
//          registers::sr::bits::uif::POSITION
//      >::address) = 0;
      reinterpret_cast<Registers*>(T)->SR = 0;
    }

    /**
     * @brief Enables the DMA request.
     */
    template<address::E T>
    void Functions<T>::enableDMA()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::ude::POSITION
      >::address) = 1;
    }

    /**
     * @brief Disables the DMA request.
     */
    template<address::E T>
    void Functions<T>::disableDMA()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::ude::POSITION
      >::address) = 0;
    }

    /**
     * @brief Configures the timer.
     */
    template<address::E T>
    template<
        tim::registers::cr1::bits::cen::states::E CEN,
        tim::registers::cr1::bits::udis::states::E UDIS,
        tim::registers::cr1::bits::urs::states::E URS,
        tim::registers::cr1::bits::opm::states::E OPM,
        tim::registers::cr1::bits::arpe::states::E ARPE
    >
    void Functions<T>::configureCounter()
    {
      reinterpret_cast<Registers*>(T)->CR1 = CEN + UDIS + URS + OPM + ARPE;
    }

    /**
     * @brief Configures the master mode.
     */
    template<address::E T>
    template<
        tim::registers::cr2::bits::mms::states::E MMS
    >
    void Functions<T>::setMasterMode()
    {
      reinterpret_cast<basic::Registers*>(T)->CR2 = MMS;
    }
  }  // namespace basic

  namespace generalPurpose1 {
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
     */
    template<address::E T>
    void Functions<T>::setPrescaler(u16 const psc)
    {
      reinterpret_cast<Registers*>(T)->PSC = psc;
    }

    /**
     * @brief Sets the autoreload value of the counter.
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
    void Functions<T>::enableInterrupt()
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
    void Functions<T>::disableInterrupt()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::uie::POSITION
      >::address) = 0;
    }

    /**
     * @brief Clears the interrupt flag.
     */
    template<address::E T>
    void Functions<T>::clearFlag()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::sr::OFFSET,
          registers::sr::bits::uif::POSITION
      >::address) = 0;
    }

    /**
     * @brief Enables the DMA request.
     */
    template<address::E T>
    void Functions<T>::enableDMA()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::ude::POSITION
      >::address) = 1;
    }

    /**
     * @brief Disables the DMA request.
     */
    template<address::E T>
    void Functions<T>::disableDMA()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::ude::POSITION
      >::address) = 0;
    }

    /**
     * @brief Configures the timer.
     */
    template<address::E T>
    template<
        tim::registers::cr1::bits::cen::states::E CEN,
        tim::registers::cr1::bits::udis::states::E UDIS,
        tim::registers::cr1::bits::urs::states::E URS,
        tim::registers::cr1::bits::opm::states::E OPM,
        tim::registers::cr1::bits::arpe::states::E ARPE
    >
    void Functions<T>::configureCounter()
    {
      reinterpret_cast<Registers*>(T)->CR1 = CEN + UDIS + URS + OPM + ARPE;
    }

    /**
     * @brief Configures the master mode.
     */
    template<address::E T>
    template<
        tim::registers::cr2::bits::mms::states::E MMS
    >
    void Functions<T>::setMasterMode()
    {
      reinterpret_cast<basic::Registers*>(T)->CR2 = MMS;
    }

  // TODO TIM General-Purpose 1 function implementations
  }// namespace generalPurpose1

  namespace generalPurpose2 {
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
     */
    template<address::E T>
    void Functions<T>::setPrescaler(u16 const psc)
    {
      reinterpret_cast<Registers*>(T)->PSC = psc;
    }

    /**
     * @brief Sets the autoreload value of the counter.
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
    void Functions<T>::enableInterrupt()
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
    void Functions<T>::disableInterrupt()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::uie::POSITION
      >::address) = 0;
    }

    /**
     * @brief Clears the interrupt flag.
     */
    template<address::E T>
    void Functions<T>::clearFlag()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::sr::OFFSET,
          registers::sr::bits::uif::POSITION
      >::address) = 0;
    }

    /**
     * @brief Enables the DMA request.
     */
    template<address::E T>
    void Functions<T>::enableDMA()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::ude::POSITION
      >::address) = 1;
    }

    /**
     * @brief Disables the DMA request.
     */
    template<address::E T>
    void Functions<T>::disableDMA()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::ude::POSITION
      >::address) = 0;
    }

    /**
     * @brief Configures the timer.
     */
    template<address::E T>
    template<
        tim::registers::cr1::bits::cen::states::E CEN,
        tim::registers::cr1::bits::udis::states::E UDIS,
        tim::registers::cr1::bits::urs::states::E URS,
        tim::registers::cr1::bits::opm::states::E OPM,
        tim::registers::cr1::bits::arpe::states::E ARPE
    >
    void Functions<T>::configureCounter()
    {
      reinterpret_cast<Registers*>(T)->CR1 = CEN + UDIS + URS + OPM + ARPE;
    }

    /**
     * @brief Configures the master mode.
     */
    template<address::E T>
    template<
        tim::registers::cr2::bits::mms::states::E MMS
    >
    void Functions<T>::setMasterMode()
    {
      reinterpret_cast<basic::Registers*>(T)->CR2 = MMS;
    }

  // TODO TIM General-Purpose 2 function implementations
  }// namespace generalPurpose2

  namespace generalPurpose3 {
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
     */
    template<address::E T>
    void Functions<T>::setPrescaler(u16 const psc)
    {
      reinterpret_cast<Registers*>(T)->PSC = psc;
    }

    /**
     * @brief Sets the autoreload value of the counter.
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
    void Functions<T>::enableInterrupt()
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
    void Functions<T>::disableInterrupt()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::uie::POSITION
      >::address) = 0;
    }

    /**
     * @brief Clears the interrupt flag.
     */
    template<address::E T>
    void Functions<T>::clearFlag()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::sr::OFFSET,
          registers::sr::bits::uif::POSITION
      >::address) = 0;
    }

    /**
     * @brief Enables the DMA request.
     */
    template<address::E T>
    void Functions<T>::enableDMA()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::ude::POSITION
      >::address) = 1;
    }

    /**
     * @brief Disables the DMA request.
     */
    template<address::E T>
    void Functions<T>::disableDMA()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::ude::POSITION
      >::address) = 0;
    }

    /**
     * @brief Configures the timer.
     */
    template<address::E T>
    template<
        tim::registers::cr1::bits::cen::states::E CEN,
        tim::registers::cr1::bits::udis::states::E UDIS,
        tim::registers::cr1::bits::urs::states::E URS,
        tim::registers::cr1::bits::opm::states::E OPM,
        tim::registers::cr1::bits::arpe::states::E ARPE
    >
    void Functions<T>::configureCounter()
    {
      reinterpret_cast<Registers*>(T)->CR1 = CEN + UDIS + URS + OPM + ARPE;
    }

    /**
     * @brief Configures the master mode.
     */
    template<address::E T>
    template<
        tim::registers::cr2::bits::mms::states::E MMS
    >
    void Functions<T>::setMasterMode()
    {
      reinterpret_cast<basic::Registers*>(T)->CR2 = MMS;
    }

  // TODO TIM General-Purpose 3 function implementations
  }// namespace generalPurpose3

#ifdef VALUE_LINE
  namespace generalPurpose4 {
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
     */
    template<address::E T>
    void Functions<T>::setPrescaler(u16 const psc)
    {
      reinterpret_cast<Registers*>(T)->PSC = psc;
    }

    /**
     * @brief Sets the autoreload value of the counter.
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
    void Functions<T>::enableInterrupt()
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
    void Functions<T>::disableInterrupt()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::uie::POSITION
      >::address) = 0;
    }

    /**
     * @brief Clears the interrupt flag.
     */
    template<address::E T>
    void Functions<T>::clearFlag()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::sr::OFFSET,
          registers::sr::bits::uif::POSITION
      >::address) = 0;
    }

    /**
     * @brief Enables the DMA request.
     */
    template<address::E T>
    void Functions<T>::enableDMA()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::ude::POSITION
      >::address) = 1;
    }

    /**
     * @brief Disables the DMA request.
     */
    template<address::E T>
    void Functions<T>::disableDMA()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::ude::POSITION
      >::address) = 0;
    }

    /**
     * @brief Configures the timer.
     */
    template<address::E T>
    template<
        tim::registers::cr1::bits::cen::states::E CEN,
        tim::registers::cr1::bits::udis::states::E UDIS,
        tim::registers::cr1::bits::urs::states::E URS,
        tim::registers::cr1::bits::opm::states::E OPM,
        tim::registers::cr1::bits::arpe::states::E ARPE
    >
    void Functions<T>::configureCounter()
    {
      reinterpret_cast<Registers*>(T)->CR1 = CEN + UDIS + URS + OPM + ARPE;
    }

    /**
     * @brief Configures the master mode.
     */
    template<address::E T>
    template<
        tim::registers::cr2::bits::mms::states::E MMS
    >
    void Functions<T>::setMasterMode()
    {
      reinterpret_cast<basic::Registers*>(T)->CR2 = MMS;
    }

  // TODO TIM General-Purpose 4 function implementations
  }// namespace generalPurpose4
#endif

  namespace advancedControl {
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
     */
    template<address::E T>
    void Functions<T>::setPrescaler(u16 const psc)
    {
      reinterpret_cast<Registers*>(T)->PSC = psc;
    }

    /**
     * @brief Sets the autoreload value of the counter.
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
    void Functions<T>::enableInterrupt()
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
    void Functions<T>::disableInterrupt()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::uie::POSITION
      >::address) = 0;
    }

    /**
     * @brief Clears the interrupt flag.
     */
    template<address::E T>
    void Functions<T>::clearFlag()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::sr::OFFSET,
          registers::sr::bits::uif::POSITION
      >::address) = 0;
    }

    /**
     * @brief Enables the DMA request.
     */
    template<address::E T>
    void Functions<T>::enableDMA()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::ude::POSITION
      >::address) = 1;
    }

    /**
     * @brief Disables the DMA request.
     */
    template<address::E T>
    void Functions<T>::disableDMA()
    {
      *(u32*) (bitband::Peripheral<
          T + registers::dier::OFFSET,
          registers::dier::bits::ude::POSITION
      >::address) = 0;
    }

    /**
     * @brief Configures the timer.
     */
    template<address::E T>
    template<
        tim::registers::cr1::bits::cen::states::E CEN,
        tim::registers::cr1::bits::udis::states::E UDIS,
        tim::registers::cr1::bits::urs::states::E URS,
        tim::registers::cr1::bits::opm::states::E OPM,
        tim::registers::cr1::bits::arpe::states::E ARPE
    >
    void Functions<T>::configureCounter()
    {
      reinterpret_cast<Registers*>(T)->CR1 = CEN + UDIS + URS + OPM + ARPE;
    }

    /**
     * @brief Configures the master mode.
     */
    template<address::E T>
    template<
        tim::registers::cr2::bits::mms::states::E MMS
    >
    void Functions<T>::setMasterMode()
    {
      reinterpret_cast<basic::Registers*>(T)->CR2 = MMS;
    }

  // TODO TIM Advanced-Control function implementations
  }// namespace advancedControl
}  // namespace tim

