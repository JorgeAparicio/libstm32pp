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
#define USART1_REGS reinterpret_cast<usart::Registers*>(usart::USART1)
#define USART2_REGS reinterpret_cast<usart::Registers*>(usart::USART2)
#define USART3_REGS reinterpret_cast<usart::Registers*>(usart::USART3)
#define UART4_REGS  reinterpret_cast<usart::Registers*>(usart::UART4)
#define UART5_REGS  reinterpret_cast<usart::Registers*>(usart::UART5)
#ifndef STM32F1XX
#define USART6_REGS reinterpret_cast<usart::Registers*>(usart::USART6)
#endif

// High-level functions
namespace usart {
  template<Address U>
  class Asynchronous {
    public:
      enum {
        FREQUENCY =
        u32(U) > u32(alias::APB2) ?
                                    clock::APB2 :
                                    clock::APB1
      };

      static inline void enableClock();
      static inline void disableClock();
      static inline void sendData(u8 const data);
      static inline u8 getData();
      static inline bool canSendDataYet();
      static inline bool isThereDataAvailable();
      template<u32 BAUD_RATE>
      static inline void setBaudRate();

      /**
       * @brief Configures the USART for asynchronous operation.
       */
      static inline void configure(
          usart::cr1::rwu::States,
          usart::cr1::re::States,
          usart::cr1::te::States,
          usart::cr1::idleie::States,
          usart::cr1::rxneie::States,
          usart::cr1::tcie::States,
          usart::cr1::txeie::States,
          usart::cr1::peie::States,
          usart::cr1::ps::States,
          usart::cr1::pce::States,
          usart::cr1::wake::States,
          usart::cr1::m::States,
          usart::cr1::ue::States,
          usart::cr1::over8::States,
          usart::cr2::stop::States,
          usart::cr3::eie::States,
          usart::cr3::hdsel::States,
          usart::cr3::dmar::States,
          usart::cr3::dmat::States,
          usart::cr3::rtse::States,
          usart::cr3::ctse::States,
          usart::cr3::ctsie::States,
          usart::cr3::onebit::States);

    private:
      Asynchronous();
  };

  template<Address I>
  class Synchronous {
    public:
      // TODO USART (synchronous) function implementation
    private:
      Synchronous();
  };

  template<Address I>
  class IRDA {
    public:
      // TODO USART (IRDA) function implementation
    private:
      IRDA();
  };

  template<Address L>
  class LIN {
    public:
      // TODO USART (LIN) function implementation
    private:
      LIN();
  };
}  // namespace usart

// High-level access to the peripherals
typedef usart::Asynchronous<usart::USART1> USART1;
typedef usart::Asynchronous<usart::USART2> USART2;
typedef usart::Asynchronous<usart::USART3> USART3;
typedef usart::Asynchronous<usart::UART4> UART4;
typedef usart::Asynchronous<usart::UART5> UART5;
#ifndef STM32F1XX
typedef usart::Asynchronous<usart::USART6> USART6;
#endif

#include "../../bits/usart.tcc"
