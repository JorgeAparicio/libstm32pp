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
#define GPIOA_REGS  ((gpio::Registers *) gpio::GPIOA)
#define GPIOB_REGS  ((gpio::Registers *) gpio::GPIOB)
#define GPIOC_REGS  ((gpio::Registers *) gpio::GPIOC)
#define GPIOD_REGS  ((gpio::Registers *) gpio::GPIOD)
#define GPIOE_REGS  ((gpio::Registers *) gpio::GPIOE)
#define GPIOF_REGS  ((gpio::Registers *) gpio::GPIOF)
#define GPIOG_REGS  ((gpio::Registers *) gpio::GPIOG)
#if defined(STM32F4XX) || defined(STM32F2XX)
#define GPIOH_REGS  ((gpio::Registers *) gpio::GPIOH)
#define GPIOI_REGS  ((gpio::Registers *) gpio::GPIOI)
#endif

// High-level functions
namespace gpio {
#ifdef STM32F1XX
  template<Address P, u8 N>
  class Pin {
    public:
    enum {
      OUT_ADDRESS = bitband::peripheral<
      P + odr::OFFSET,
      N
      >(),
      IN_ADDRESS = bitband::peripheral<
      P + idr::OFFSET,
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
    static inline void setMode(gpio::cr::States);

    private:
    Pin();
  };

  template<Address P>
  class Port {
    public:
    static inline void enableClock();
    static inline void disableClock();

    static inline void configureLowerPins(
        gpio::cr::States,
        gpio::cr::States,
        gpio::cr::States,
        gpio::cr::States,
        gpio::cr::States,
        gpio::cr::States,
        gpio::cr::States,
        gpio::cr::States);

    static inline void configureHigherPins(
        gpio::cr::States,
        gpio::cr::States,
        gpio::cr::States,
        gpio::cr::States,
        gpio::cr::States,
        gpio::cr::States,
        gpio::cr::States,
        gpio::cr::States);

    static inline void setValue(u32 const value);
    static inline u32 getValue();

    private:
    Port();
  };

#else // STM32F1XX
  template<Address P, u8 N>
  class Pin {
    public:
      enum {
        OUT_ADDRESS = bitband::peripheral<
            P + odr::OFFSET,
            N
        >(),
        IN_ADDRESS = bitband::peripheral<
            P + idr::OFFSET,
            N
        >(),
      };

      static inline void enableClock();
      static inline void setHigh();
      static inline void setLow();
      static inline void setOutput(u32 const value);
      static inline u32 getInput();
      static inline bool isHigh();
      static inline void setMode(gpio::moder::States);
      static inline void setOutputMode(gpio::otyper::States);
      static inline void setSpeed(gpio::ospeedr::States);
      static inline void setPullMode(gpio::pupdr::States);
      static inline void setAlternateFunction(gpio::afr::States);

    private:
      Pin();
  };

  template<Address P>
  class Port {
    public:
      static inline void enableClock();
      static inline void disableClock();
      static inline void setOutput(u16 const);
      static inline u16 getInput();

      static inline void setModes(
          gpio::moder::States,
          gpio::moder::States,
          gpio::moder::States,
          gpio::moder::States,
          gpio::moder::States,
          gpio::moder::States,
          gpio::moder::States,
          gpio::moder::States,
          gpio::moder::States,
          gpio::moder::States,
          gpio::moder::States,
          gpio::moder::States,
          gpio::moder::States,
          gpio::moder::States,
          gpio::moder::States,
          gpio::moder::States);

      static inline void setOutputTypes(
          gpio::otyper::States,
          gpio::otyper::States,
          gpio::otyper::States,
          gpio::otyper::States,
          gpio::otyper::States,
          gpio::otyper::States,
          gpio::otyper::States,
          gpio::otyper::States,
          gpio::otyper::States,
          gpio::otyper::States,
          gpio::otyper::States,
          gpio::otyper::States,
          gpio::otyper::States,
          gpio::otyper::States,
          gpio::otyper::States,
          gpio::otyper::States);

      static inline void setOutputSpeeds(
          gpio::ospeedr::States,
          gpio::ospeedr::States,
          gpio::ospeedr::States,
          gpio::ospeedr::States,
          gpio::ospeedr::States,
          gpio::ospeedr::States,
          gpio::ospeedr::States,
          gpio::ospeedr::States,
          gpio::ospeedr::States,
          gpio::ospeedr::States,
          gpio::ospeedr::States,
          gpio::ospeedr::States,
          gpio::ospeedr::States,
          gpio::ospeedr::States,
          gpio::ospeedr::States,
          gpio::ospeedr::States);

      static inline void setPullModes(
          gpio::pupdr::States,
          gpio::pupdr::States,
          gpio::pupdr::States,
          gpio::pupdr::States,
          gpio::pupdr::States,
          gpio::pupdr::States,
          gpio::pupdr::States,
          gpio::pupdr::States,
          gpio::pupdr::States,
          gpio::pupdr::States,
          gpio::pupdr::States,
          gpio::pupdr::States,
          gpio::pupdr::States,
          gpio::pupdr::States,
          gpio::pupdr::States,
          gpio::pupdr::States);

    private:
      Port();
  };
#endif // STM32F1XX
}
// namespace gpio

// High-level access to the peripheral
typedef gpio::Port<gpio::GPIOA> GPIOA;
typedef gpio::Port<gpio::GPIOB> GPIOB;
typedef gpio::Port<gpio::GPIOC> GPIOC;
typedef gpio::Port<gpio::GPIOD> GPIOD;
typedef gpio::Port<gpio::GPIOE> GPIOE;
typedef gpio::Port<gpio::GPIOF> GPIOF;
typedef gpio::Port<gpio::GPIOG> GPIOG;
#if defined(STM32F4XX) || defined(STM32F2XX)
typedef gpio::Port<gpio::GPIOH> GPIOH;
typedef gpio::Port<gpio::GPIOI> GPIOI;
#endif

typedef gpio::Pin<gpio::GPIOA, 0> PA0;
typedef gpio::Pin<gpio::GPIOA, 1> PA1;
typedef gpio::Pin<gpio::GPIOA, 2> PA2;
typedef gpio::Pin<gpio::GPIOA, 3> PA3;
typedef gpio::Pin<gpio::GPIOA, 4> PA4;
typedef gpio::Pin<gpio::GPIOA, 5> PA5;
typedef gpio::Pin<gpio::GPIOA, 6> PA6;
typedef gpio::Pin<gpio::GPIOA, 7> PA7;
typedef gpio::Pin<gpio::GPIOA, 8> PA8;
typedef gpio::Pin<gpio::GPIOA, 9> PA9;
typedef gpio::Pin<gpio::GPIOA, 10> PA10;
typedef gpio::Pin<gpio::GPIOA, 11> PA11;
typedef gpio::Pin<gpio::GPIOA, 12> PA12;
typedef gpio::Pin<gpio::GPIOA, 13> PA13;
typedef gpio::Pin<gpio::GPIOA, 14> PA14;
typedef gpio::Pin<gpio::GPIOA, 15> PA15;

typedef gpio::Pin<gpio::GPIOB, 0> PB0;
typedef gpio::Pin<gpio::GPIOB, 1> PB1;
typedef gpio::Pin<gpio::GPIOB, 2> PB2;
typedef gpio::Pin<gpio::GPIOB, 3> PB3;
typedef gpio::Pin<gpio::GPIOB, 4> PB4;
typedef gpio::Pin<gpio::GPIOB, 5> PB5;
typedef gpio::Pin<gpio::GPIOB, 6> PB6;
typedef gpio::Pin<gpio::GPIOB, 7> PB7;
typedef gpio::Pin<gpio::GPIOB, 8> PB8;
typedef gpio::Pin<gpio::GPIOB, 9> PB9;
typedef gpio::Pin<gpio::GPIOB, 10> PB10;
typedef gpio::Pin<gpio::GPIOB, 11> PB11;
typedef gpio::Pin<gpio::GPIOB, 12> PB12;
typedef gpio::Pin<gpio::GPIOB, 13> PB13;
typedef gpio::Pin<gpio::GPIOB, 14> PB14;
typedef gpio::Pin<gpio::GPIOB, 15> PB15;

typedef gpio::Pin<gpio::GPIOC, 0> PC0;
typedef gpio::Pin<gpio::GPIOC, 1> PC1;
typedef gpio::Pin<gpio::GPIOC, 2> PC2;
typedef gpio::Pin<gpio::GPIOC, 3> PC3;
typedef gpio::Pin<gpio::GPIOC, 4> PC4;
typedef gpio::Pin<gpio::GPIOC, 5> PC5;
typedef gpio::Pin<gpio::GPIOC, 6> PC6;
typedef gpio::Pin<gpio::GPIOC, 7> PC7;
typedef gpio::Pin<gpio::GPIOC, 8> PC8;
typedef gpio::Pin<gpio::GPIOC, 9> PC9;
typedef gpio::Pin<gpio::GPIOC, 10> PC10;
typedef gpio::Pin<gpio::GPIOC, 11> PC11;
typedef gpio::Pin<gpio::GPIOC, 12> PC12;
typedef gpio::Pin<gpio::GPIOC, 13> PC13;
typedef gpio::Pin<gpio::GPIOC, 14> PC14;
typedef gpio::Pin<gpio::GPIOC, 15> PC15;

typedef gpio::Pin<gpio::GPIOD, 0> PD0;
typedef gpio::Pin<gpio::GPIOD, 1> PD1;
typedef gpio::Pin<gpio::GPIOD, 2> PD2;
typedef gpio::Pin<gpio::GPIOD, 3> PD3;
typedef gpio::Pin<gpio::GPIOD, 4> PD4;
typedef gpio::Pin<gpio::GPIOD, 5> PD5;
typedef gpio::Pin<gpio::GPIOD, 6> PD6;
typedef gpio::Pin<gpio::GPIOD, 7> PD7;
typedef gpio::Pin<gpio::GPIOD, 8> PD8;
typedef gpio::Pin<gpio::GPIOD, 9> PD9;
typedef gpio::Pin<gpio::GPIOD, 10> PD10;
typedef gpio::Pin<gpio::GPIOD, 11> PD11;
typedef gpio::Pin<gpio::GPIOD, 12> PD12;
typedef gpio::Pin<gpio::GPIOD, 13> PD13;
typedef gpio::Pin<gpio::GPIOD, 14> PD14;
typedef gpio::Pin<gpio::GPIOD, 15> PD15;

typedef gpio::Pin<gpio::GPIOE, 0> PE0;
typedef gpio::Pin<gpio::GPIOE, 1> PE1;
typedef gpio::Pin<gpio::GPIOE, 2> PE2;
typedef gpio::Pin<gpio::GPIOE, 3> PE3;
typedef gpio::Pin<gpio::GPIOE, 4> PE4;
typedef gpio::Pin<gpio::GPIOE, 5> PE5;
typedef gpio::Pin<gpio::GPIOE, 6> PE6;
typedef gpio::Pin<gpio::GPIOE, 7> PE7;
typedef gpio::Pin<gpio::GPIOE, 8> PE8;
typedef gpio::Pin<gpio::GPIOE, 9> PE9;
typedef gpio::Pin<gpio::GPIOE, 10> PE10;
typedef gpio::Pin<gpio::GPIOE, 11> PE11;
typedef gpio::Pin<gpio::GPIOE, 12> PE12;
typedef gpio::Pin<gpio::GPIOE, 13> PE13;
typedef gpio::Pin<gpio::GPIOE, 14> PE14;
typedef gpio::Pin<gpio::GPIOE, 15> PE15;

typedef gpio::Pin<gpio::GPIOF, 0> PF0;
typedef gpio::Pin<gpio::GPIOF, 1> PF1;
typedef gpio::Pin<gpio::GPIOF, 2> PF2;
typedef gpio::Pin<gpio::GPIOF, 3> PF3;
typedef gpio::Pin<gpio::GPIOF, 4> PF4;
typedef gpio::Pin<gpio::GPIOF, 5> PF5;
typedef gpio::Pin<gpio::GPIOF, 6> PF6;
typedef gpio::Pin<gpio::GPIOF, 7> PF7;
typedef gpio::Pin<gpio::GPIOF, 8> PF8;
typedef gpio::Pin<gpio::GPIOF, 9> PF9;
typedef gpio::Pin<gpio::GPIOF, 10> PF10;
typedef gpio::Pin<gpio::GPIOF, 11> PF11;
typedef gpio::Pin<gpio::GPIOF, 12> PF12;
typedef gpio::Pin<gpio::GPIOF, 13> PF13;
typedef gpio::Pin<gpio::GPIOF, 14> PF14;
typedef gpio::Pin<gpio::GPIOF, 15> PF15;

typedef gpio::Pin<gpio::GPIOG, 0> PG0;
typedef gpio::Pin<gpio::GPIOG, 1> PG1;
typedef gpio::Pin<gpio::GPIOG, 2> PG2;
typedef gpio::Pin<gpio::GPIOG, 3> PG3;
typedef gpio::Pin<gpio::GPIOG, 4> PG4;
typedef gpio::Pin<gpio::GPIOG, 5> PG5;
typedef gpio::Pin<gpio::GPIOG, 6> PG6;
typedef gpio::Pin<gpio::GPIOG, 7> PG7;
typedef gpio::Pin<gpio::GPIOG, 8> PG8;
typedef gpio::Pin<gpio::GPIOG, 9> PG9;
typedef gpio::Pin<gpio::GPIOG, 10> PG10;
typedef gpio::Pin<gpio::GPIOG, 11> PG11;
typedef gpio::Pin<gpio::GPIOG, 12> PG12;
typedef gpio::Pin<gpio::GPIOG, 13> PG13;
typedef gpio::Pin<gpio::GPIOG, 14> PG14;
typedef gpio::Pin<gpio::GPIOG, 15> PG15;

#if defined(STM32F4XX) || defined(STM32F2XX)
typedef gpio::Pin<gpio::GPIOH, 0> PH0;
typedef gpio::Pin<gpio::GPIOH, 1> PH1;
typedef gpio::Pin<gpio::GPIOH, 2> PH2;
typedef gpio::Pin<gpio::GPIOH, 3> PH3;
typedef gpio::Pin<gpio::GPIOH, 4> PH4;
typedef gpio::Pin<gpio::GPIOH, 5> PH5;
typedef gpio::Pin<gpio::GPIOH, 6> PH6;
typedef gpio::Pin<gpio::GPIOH, 7> PH7;
typedef gpio::Pin<gpio::GPIOH, 8> PH8;
typedef gpio::Pin<gpio::GPIOH, 9> PH9;
typedef gpio::Pin<gpio::GPIOH, 10> PH10;
typedef gpio::Pin<gpio::GPIOH, 11> PH11;
typedef gpio::Pin<gpio::GPIOH, 12> PH12;
typedef gpio::Pin<gpio::GPIOH, 13> PH13;
typedef gpio::Pin<gpio::GPIOH, 14> PH14;
typedef gpio::Pin<gpio::GPIOH, 15> PH15;

typedef gpio::Pin<gpio::GPIOI, 0> PI0;
typedef gpio::Pin<gpio::GPIOI, 1> PI1;
typedef gpio::Pin<gpio::GPIOI, 2> PI2;
typedef gpio::Pin<gpio::GPIOI, 3> PI3;
typedef gpio::Pin<gpio::GPIOI, 4> PI4;
typedef gpio::Pin<gpio::GPIOI, 5> PI5;
typedef gpio::Pin<gpio::GPIOI, 6> PI6;
typedef gpio::Pin<gpio::GPIOI, 7> PI7;
typedef gpio::Pin<gpio::GPIOI, 8> PI8;
typedef gpio::Pin<gpio::GPIOI, 9> PI9;
typedef gpio::Pin<gpio::GPIOI, 10> PI10;
typedef gpio::Pin<gpio::GPIOI, 11> PI11;
#endif
#include "../../bits/gpio.tcc"
