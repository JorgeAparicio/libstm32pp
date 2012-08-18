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
 *                            General Purpose I/O
 *
 ******************************************************************************/

#pragma once

#include "../device_select.hpp"
#include "../defs.hpp"
#include "bitband.hpp"

#include "../../memorymap/gpio.hpp"

// Low-level access to the registers
#define _GPIOA  ((gpio::Registers *) gpio::address::E::GPIOA)
#define _GPIOB  ((gpio::Registers *) gpio::address::E::GPIOB)
#define _GPIOC  ((gpio::Registers *) gpio::address::E::GPIOC)
#define _GPIOD  ((gpio::Registers *) gpio::address::E::GPIOD)
#define _GPIOE  ((gpio::Registers *) gpio::address::E::GPIOE)
#define _GPIOF  ((gpio::Registers *) gpio::address::E::GPIOF)
#define _GPIOG  ((gpio::Registers *) gpio::address::E::GPIOG)
#ifdef STM32F4XX
#define _GPIOH  ((gpio::Registers *) gpio::address::E::GPIOH)
#define _GPIOI  ((gpio::Registers *) gpio::address::E::GPIOI)
#endif

// High-level functions
namespace gpio {
#ifdef STM32F1XX
  template<address::E P, u8 N>
  class Pin {
    public:
      enum {
        OUT_ADDRESS = bitband::peripheral<
            P + registers::odr::OFFSET,
            N
        >(),
        IN_ADDRESS = bitband::peripheral<
            P + registers::idr::OFFSET,
            N
        >(),
      };

      static inline void enableClock();
      static inline void setHigh();
      static inline void setLow();
      static inline void setOutput(u32 const value);
      static inline u32 getInput();
      static inline void pullUp();
      static inline void pullDown();
      static inline bool isHigh();

      template<
          gpio::registers::cr::states::E CR
      >
      static inline void setMode();

    private:
      Pin();
  };

  template<address::E P>
  class Port {
    public:
      static inline void enableClock();
      static inline void disableClock();

      template<
          gpio::registers::cr::states::E, /* 0 */
          gpio::registers::cr::states::E, /* 1 */
          gpio::registers::cr::states::E, /* 2 */
          gpio::registers::cr::states::E, /* 3 */
          gpio::registers::cr::states::E, /* 4 */
          gpio::registers::cr::states::E, /* 5 */
          gpio::registers::cr::states::E, /* 6 */
          gpio::registers::cr::states::E /* 7 */
      >
      static inline void configureLowerPins();

      template<
          gpio::registers::cr::states::E, /* 8 */
          gpio::registers::cr::states::E, /* 9 */
          gpio::registers::cr::states::E, /* 10 */
          gpio::registers::cr::states::E, /* 11 */
          gpio::registers::cr::states::E, /* 12 */
          gpio::registers::cr::states::E, /* 13 */
          gpio::registers::cr::states::E, /* 14 */
          gpio::registers::cr::states::E /* 15 */
      >
      static inline void configureHigherPins();

      static inline void setValue(u32 const value);
      static inline u32 getValue();

    private:
      Port();

  };
// class Port

#else // STM32F1XX
template<address::E P, u8 N>
class Pin {
  public:
  enum {
    OUT_ADDRESS = bitband::peripheral<
    P + registers::odr::OFFSET,
    N
    >(),
    IN_ADDRESS = bitband::peripheral<
    P + registers::idr::OFFSET,
    N
    >(),
  };

  static inline void enableClock();
  static inline void setHigh();
  static inline void setLow();
  static inline void setOutput(u32 const value);
  static inline u32 getInput();
  static inline bool isHigh();

  template<
  gpio::registers::moder::states::E
  >
  static inline void setMode();

  template<
  gpio::registers::otyper::states::E
  >
  static inline void setOutputMode();

  template<
  gpio::registers::ospeedr::states::E
  >
  static inline void setSpeed();

  template<
  gpio::registers::pupdr::states::E
  >
  static inline void setPullMode();

  template<
  gpio::registers::afr::states::E
  >
  static inline void setAlternateFunction();

  private:
  Pin();
};
// class Pin

template<address::E P>
class Port {
  public:
  static inline void enableClock();
  static inline void disableClock();
  static inline void setOutput(u16 const);
  static inline u16 getInput();

  template<
  gpio::registers::moder::states::E, /* 0 */
  gpio::registers::moder::states::E, /* 1 */
  gpio::registers::moder::states::E, /* 2 */
  gpio::registers::moder::states::E, /* 3 */
  gpio::registers::moder::states::E, /* 4 */
  gpio::registers::moder::states::E, /* 5 */
  gpio::registers::moder::states::E, /* 6 */
  gpio::registers::moder::states::E, /* 7 */
  gpio::registers::moder::states::E, /* 8 */
  gpio::registers::moder::states::E, /* 9 */
  gpio::registers::moder::states::E, /* 10 */
  gpio::registers::moder::states::E, /* 11 */
  gpio::registers::moder::states::E, /* 12 */
  gpio::registers::moder::states::E, /* 13 */
  gpio::registers::moder::states::E, /* 14 */
  gpio::registers::moder::states::E /* 15 */
  >
  static inline void setModes();

  template<
  gpio::registers::otyper::states::E, /* 0 */
  gpio::registers::otyper::states::E, /* 1 */
  gpio::registers::otyper::states::E, /* 2 */
  gpio::registers::otyper::states::E, /* 3 */
  gpio::registers::otyper::states::E, /* 4 */
  gpio::registers::otyper::states::E, /* 5 */
  gpio::registers::otyper::states::E, /* 6 */
  gpio::registers::otyper::states::E, /* 7 */
  gpio::registers::otyper::states::E, /* 8 */
  gpio::registers::otyper::states::E, /* 9 */
  gpio::registers::otyper::states::E, /* 10 */
  gpio::registers::otyper::states::E, /* 11 */
  gpio::registers::otyper::states::E, /* 12 */
  gpio::registers::otyper::states::E, /* 13 */
  gpio::registers::otyper::states::E, /* 14 */
  gpio::registers::otyper::states::E /* 15 */
  >
  static inline void setOutputTypes();

  template<
  gpio::registers::ospeedr::states::E, /* 0 */
  gpio::registers::ospeedr::states::E, /* 1 */
  gpio::registers::ospeedr::states::E, /* 2 */
  gpio::registers::ospeedr::states::E, /* 3 */
  gpio::registers::ospeedr::states::E, /* 4 */
  gpio::registers::ospeedr::states::E, /* 5 */
  gpio::registers::ospeedr::states::E, /* 6 */
  gpio::registers::ospeedr::states::E, /* 7 */
  gpio::registers::ospeedr::states::E, /* 8 */
  gpio::registers::ospeedr::states::E, /* 9 */
  gpio::registers::ospeedr::states::E, /* 10 */
  gpio::registers::ospeedr::states::E, /* 11 */
  gpio::registers::ospeedr::states::E, /* 12 */
  gpio::registers::ospeedr::states::E, /* 13 */
  gpio::registers::ospeedr::states::E, /* 14 */
  gpio::registers::ospeedr::states::E /* 15 */
  >
  static inline void setOutputSpeeds();

  template<
  gpio::registers::pupdr::states::E, /* 0 */
  gpio::registers::pupdr::states::E, /* 1 */
  gpio::registers::pupdr::states::E, /* 2 */
  gpio::registers::pupdr::states::E, /* 3 */
  gpio::registers::pupdr::states::E, /* 4 */
  gpio::registers::pupdr::states::E, /* 5 */
  gpio::registers::pupdr::states::E, /* 6 */
  gpio::registers::pupdr::states::E, /* 7 */
  gpio::registers::pupdr::states::E, /* 8 */
  gpio::registers::pupdr::states::E, /* 9 */
  gpio::registers::pupdr::states::E, /* 10 */
  gpio::registers::pupdr::states::E, /* 11 */
  gpio::registers::pupdr::states::E, /* 12 */
  gpio::registers::pupdr::states::E, /* 13 */
  gpio::registers::pupdr::states::E, /* 14 */
  gpio::registers::pupdr::states::E /* 15 */
  >
  static inline void setPullModes();

  private:
  Port();
};
// class Port
#endif // STM32F1XX
}
 // namespace gpio

// High-level access to the peripheral
typedef gpio::Port<gpio::address::GPIOA> GPIOA;
typedef gpio::Port<gpio::address::GPIOB> GPIOB;
typedef gpio::Port<gpio::address::GPIOC> GPIOC;
typedef gpio::Port<gpio::address::GPIOD> GPIOD;
typedef gpio::Port<gpio::address::GPIOE> GPIOE;
typedef gpio::Port<gpio::address::GPIOF> GPIOF;
typedef gpio::Port<gpio::address::GPIOG> GPIOG;
#ifdef STM32F4XX
typedef gpio::Port<gpio::address::GPIOH> GPIOH;
typedef gpio::Port<gpio::address::GPIOI> GPIOI;
#endif

typedef gpio::Pin<gpio::address::GPIOA, 0> PA0;
typedef gpio::Pin<gpio::address::GPIOA, 1> PA1;
typedef gpio::Pin<gpio::address::GPIOA, 2> PA2;
typedef gpio::Pin<gpio::address::GPIOA, 3> PA3;
typedef gpio::Pin<gpio::address::GPIOA, 4> PA4;
typedef gpio::Pin<gpio::address::GPIOA, 5> PA5;
typedef gpio::Pin<gpio::address::GPIOA, 6> PA6;
typedef gpio::Pin<gpio::address::GPIOA, 7> PA7;
typedef gpio::Pin<gpio::address::GPIOA, 8> PA8;
typedef gpio::Pin<gpio::address::GPIOA, 9> PA9;
typedef gpio::Pin<gpio::address::GPIOA, 10> PA10;
typedef gpio::Pin<gpio::address::GPIOA, 11> PA11;
typedef gpio::Pin<gpio::address::GPIOA, 12> PA12;
typedef gpio::Pin<gpio::address::GPIOA, 13> PA13;
typedef gpio::Pin<gpio::address::GPIOA, 14> PA14;
typedef gpio::Pin<gpio::address::GPIOA, 15> PA15;

typedef gpio::Pin<gpio::address::GPIOB, 0> PB0;
typedef gpio::Pin<gpio::address::GPIOB, 1> PB1;
typedef gpio::Pin<gpio::address::GPIOB, 2> PB2;
typedef gpio::Pin<gpio::address::GPIOB, 3> PB3;
typedef gpio::Pin<gpio::address::GPIOB, 4> PB4;
typedef gpio::Pin<gpio::address::GPIOB, 5> PB5;
typedef gpio::Pin<gpio::address::GPIOB, 6> PB6;
typedef gpio::Pin<gpio::address::GPIOB, 7> PB7;
typedef gpio::Pin<gpio::address::GPIOB, 8> PB8;
typedef gpio::Pin<gpio::address::GPIOB, 9> PB9;
typedef gpio::Pin<gpio::address::GPIOB, 10> PB10;
typedef gpio::Pin<gpio::address::GPIOB, 11> PB11;
typedef gpio::Pin<gpio::address::GPIOB, 12> PB12;
typedef gpio::Pin<gpio::address::GPIOB, 13> PB13;
typedef gpio::Pin<gpio::address::GPIOB, 14> PB14;
typedef gpio::Pin<gpio::address::GPIOB, 15> PB15;

typedef gpio::Pin<gpio::address::GPIOC, 0> PC0;
typedef gpio::Pin<gpio::address::GPIOC, 1> PC1;
typedef gpio::Pin<gpio::address::GPIOC, 2> PC2;
typedef gpio::Pin<gpio::address::GPIOC, 3> PC3;
typedef gpio::Pin<gpio::address::GPIOC, 4> PC4;
typedef gpio::Pin<gpio::address::GPIOC, 5> PC5;
typedef gpio::Pin<gpio::address::GPIOC, 6> PC6;
typedef gpio::Pin<gpio::address::GPIOC, 7> PC7;
typedef gpio::Pin<gpio::address::GPIOC, 8> PC8;
typedef gpio::Pin<gpio::address::GPIOC, 9> PC9;
typedef gpio::Pin<gpio::address::GPIOC, 10> PC10;
typedef gpio::Pin<gpio::address::GPIOC, 11> PC11;
typedef gpio::Pin<gpio::address::GPIOC, 12> PC12;
typedef gpio::Pin<gpio::address::GPIOC, 13> PC13;
typedef gpio::Pin<gpio::address::GPIOC, 14> PC14;
typedef gpio::Pin<gpio::address::GPIOC, 15> PC15;

typedef gpio::Pin<gpio::address::GPIOD, 0> PD0;
typedef gpio::Pin<gpio::address::GPIOD, 1> PD1;
typedef gpio::Pin<gpio::address::GPIOD, 2> PD2;
typedef gpio::Pin<gpio::address::GPIOD, 3> PD3;
typedef gpio::Pin<gpio::address::GPIOD, 4> PD4;
typedef gpio::Pin<gpio::address::GPIOD, 5> PD5;
typedef gpio::Pin<gpio::address::GPIOD, 6> PD6;
typedef gpio::Pin<gpio::address::GPIOD, 7> PD7;
typedef gpio::Pin<gpio::address::GPIOD, 8> PD8;
typedef gpio::Pin<gpio::address::GPIOD, 9> PD9;
typedef gpio::Pin<gpio::address::GPIOD, 10> PD10;
typedef gpio::Pin<gpio::address::GPIOD, 11> PD11;
typedef gpio::Pin<gpio::address::GPIOD, 12> PD12;
typedef gpio::Pin<gpio::address::GPIOD, 13> PD13;
typedef gpio::Pin<gpio::address::GPIOD, 14> PD14;
typedef gpio::Pin<gpio::address::GPIOD, 15> PD15;

typedef gpio::Pin<gpio::address::GPIOE, 0> PE0;
typedef gpio::Pin<gpio::address::GPIOE, 1> PE1;
typedef gpio::Pin<gpio::address::GPIOE, 2> PE2;
typedef gpio::Pin<gpio::address::GPIOE, 3> PE3;
typedef gpio::Pin<gpio::address::GPIOE, 4> PE4;
typedef gpio::Pin<gpio::address::GPIOE, 5> PE5;
typedef gpio::Pin<gpio::address::GPIOE, 6> PE6;
typedef gpio::Pin<gpio::address::GPIOE, 7> PE7;
typedef gpio::Pin<gpio::address::GPIOE, 8> PE8;
typedef gpio::Pin<gpio::address::GPIOE, 9> PE9;
typedef gpio::Pin<gpio::address::GPIOE, 10> PE10;
typedef gpio::Pin<gpio::address::GPIOE, 11> PE11;
typedef gpio::Pin<gpio::address::GPIOE, 12> PE12;
typedef gpio::Pin<gpio::address::GPIOE, 13> PE13;
typedef gpio::Pin<gpio::address::GPIOE, 14> PE14;
typedef gpio::Pin<gpio::address::GPIOE, 15> PE15;

typedef gpio::Pin<gpio::address::GPIOF, 0> PF0;
typedef gpio::Pin<gpio::address::GPIOF, 1> PF1;
typedef gpio::Pin<gpio::address::GPIOF, 2> PF2;
typedef gpio::Pin<gpio::address::GPIOF, 3> PF3;
typedef gpio::Pin<gpio::address::GPIOF, 4> PF4;
typedef gpio::Pin<gpio::address::GPIOF, 5> PF5;
typedef gpio::Pin<gpio::address::GPIOF, 6> PF6;
typedef gpio::Pin<gpio::address::GPIOF, 7> PF7;
typedef gpio::Pin<gpio::address::GPIOF, 8> PF8;
typedef gpio::Pin<gpio::address::GPIOF, 9> PF9;
typedef gpio::Pin<gpio::address::GPIOF, 10> PF10;
typedef gpio::Pin<gpio::address::GPIOF, 11> PF11;
typedef gpio::Pin<gpio::address::GPIOF, 12> PF12;
typedef gpio::Pin<gpio::address::GPIOF, 13> PF13;
typedef gpio::Pin<gpio::address::GPIOF, 14> PF14;
typedef gpio::Pin<gpio::address::GPIOF, 15> PF15;

typedef gpio::Pin<gpio::address::GPIOG, 0> PG0;
typedef gpio::Pin<gpio::address::GPIOG, 1> PG1;
typedef gpio::Pin<gpio::address::GPIOG, 2> PG2;
typedef gpio::Pin<gpio::address::GPIOG, 3> PG3;
typedef gpio::Pin<gpio::address::GPIOG, 4> PG4;
typedef gpio::Pin<gpio::address::GPIOG, 5> PG5;
typedef gpio::Pin<gpio::address::GPIOG, 6> PG6;
typedef gpio::Pin<gpio::address::GPIOG, 7> PG7;
typedef gpio::Pin<gpio::address::GPIOG, 8> PG8;
typedef gpio::Pin<gpio::address::GPIOG, 9> PG9;
typedef gpio::Pin<gpio::address::GPIOG, 10> PG10;
typedef gpio::Pin<gpio::address::GPIOG, 11> PG11;
typedef gpio::Pin<gpio::address::GPIOG, 12> PG12;
typedef gpio::Pin<gpio::address::GPIOG, 13> PG13;
typedef gpio::Pin<gpio::address::GPIOG, 14> PG14;
typedef gpio::Pin<gpio::address::GPIOG, 15> PG15;

#ifdef STM32F4XX
typedef gpio::Pin<gpio::address::GPIOH, 0> PH0;
typedef gpio::Pin<gpio::address::GPIOH, 1> PH1;
typedef gpio::Pin<gpio::address::GPIOH, 2> PH2;
typedef gpio::Pin<gpio::address::GPIOH, 3> PH3;
typedef gpio::Pin<gpio::address::GPIOH, 4> PH4;
typedef gpio::Pin<gpio::address::GPIOH, 5> PH5;
typedef gpio::Pin<gpio::address::GPIOH, 6> PH6;
typedef gpio::Pin<gpio::address::GPIOH, 7> PH7;
typedef gpio::Pin<gpio::address::GPIOH, 8> PH8;
typedef gpio::Pin<gpio::address::GPIOH, 9> PH9;
typedef gpio::Pin<gpio::address::GPIOH, 10> PH10;
typedef gpio::Pin<gpio::address::GPIOH, 11> PH11;
typedef gpio::Pin<gpio::address::GPIOH, 12> PH12;
typedef gpio::Pin<gpio::address::GPIOH, 13> PH13;
typedef gpio::Pin<gpio::address::GPIOH, 14> PH14;
typedef gpio::Pin<gpio::address::GPIOH, 15> PH15;

typedef gpio::Pin<gpio::address::GPIOI, 0> PI0;
typedef gpio::Pin<gpio::address::GPIOI, 1> PI1;
typedef gpio::Pin<gpio::address::GPIOI, 2> PI2;
typedef gpio::Pin<gpio::address::GPIOI, 3> PI3;
typedef gpio::Pin<gpio::address::GPIOI, 4> PI4;
typedef gpio::Pin<gpio::address::GPIOI, 5> PI5;
typedef gpio::Pin<gpio::address::GPIOI, 6> PI6;
typedef gpio::Pin<gpio::address::GPIOI, 7> PI7;
typedef gpio::Pin<gpio::address::GPIOI, 8> PI8;
typedef gpio::Pin<gpio::address::GPIOI, 9> PI9;
typedef gpio::Pin<gpio::address::GPIOI, 10> PI10;
typedef gpio::Pin<gpio::address::GPIOI, 11> PI11;
#endif
#include "../../bits/gpio.tcc"
