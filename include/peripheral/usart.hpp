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

/*******************************************************************************
 *
 *         Universal Synchronous Asynchronous Receiver Transmitter
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"

#include "../defs.hpp"

#include "../clock.hpp"
#include "../../memorymap/usart.hpp"

// Low-level access to the registers
#define _USART1 reinterpret_cast<usart::Registers*>(usart::address::USART1)
#define _USART2 reinterpret_cast<usart::Registers*>(usart::address::USART2)
#define _USART3 reinterpret_cast<usart::Registers*>(usart::address::USART3)
#define _UART4  reinterpret_cast<usart::Registers*>(usart::address::UART4)
#define _UART5  reinterpret_cast<usart::Registers*>(usart::address::UART5)
#ifndef STM32F1XX
#define _USART6 reinterpret_cast<usart::Registers*>(usart::address::USART6)
#endif

// High-level functions
namespace usart {
  template<address::E U>
  class Asynchronous {
    public:
      enum {
        FREQUENCY = u32(U) > u32(alias::APB2) ?
                                                         clock::APB2 :
                                                         clock::APB1
      };

      static INLINE void enableClock();
      static INLINE void disableClock();
      static INLINE void sendData(u8 const data);
      static INLINE u8 getData(void);
      static INLINE bool canSendDataYet(void);
      static INLINE bool isThereDataAvailable(void);
      template<u32 BAUD_RATE>
      static INLINE void setBaudRate();

      /**
       * @brief Configures the USART for asynchronous operation.
       */
      template<
          usart::registers::cr1::bits::rwu::states::E,
          usart::registers::cr1::bits::re::states::E,
          usart::registers::cr1::bits::te::states::E,
          usart::registers::cr1::bits::idleie::states::E,
          usart::registers::cr1::bits::rxneie::states::E,
          usart::registers::cr1::bits::tcie::states::E,
          usart::registers::cr1::bits::txeie::states::E,
          usart::registers::cr1::bits::peie::states::E,
          usart::registers::cr1::bits::ps::states::E,
          usart::registers::cr1::bits::pce::states::E,
          usart::registers::cr1::bits::wake::states::E,
          usart::registers::cr1::bits::m::states::E,
          usart::registers::cr1::bits::ue::states::E,
          usart::registers::cr1::bits::over8::states::E,
          usart::registers::cr2::bits::stop::states::E,
          usart::registers::cr3::bits::eie::states::E,
          usart::registers::cr3::bits::hdsel::states::E,
          usart::registers::cr3::bits::dmar::states::E,
          usart::registers::cr3::bits::dmat::states::E,
          usart::registers::cr3::bits::rtse::states::E,
          usart::registers::cr3::bits::ctse::states::E,
          usart::registers::cr3::bits::ctsie::states::E,
          usart::registers::cr3::bits::onebit::states::E
      >
      static INLINE void configure(void);

    private:
      Asynchronous();
  };

  template<address::E I>
  class Synchronous {
    public:
      // TODO USART (synchronous) function implementation
    private:
      Synchronous();
  };

  template<address::E I>
  class IRDA {
    public:
      // TODO USART (IRDA) function implementation
    private:
      IRDA();
  };

  template<address::E L>
  class LIN {
    public:
      // TODO USART (LIN) function implementation
    private:
      LIN();
  };
}  // namespace usart

// High-level access to the peripherals
typedef usart::Asynchronous<usart::address::USART1> USART1;
typedef usart::Asynchronous<usart::address::USART2> USART2;
typedef usart::Asynchronous<usart::address::USART3> USART3;
typedef usart::Asynchronous<usart::address::UART4> UART4;
typedef usart::Asynchronous<usart::address::UART5> UART5;
#ifndef STM32F1XX
typedef usart::Asynchronous<usart::address::USART6> USART6;
#endif

#include "../../bits/usart.tcc"
