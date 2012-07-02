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

#include "device_select.hpp"
#include "cfunctions.hpp"

#pragma once

/* Enter your clock configuration below, ignore the grayed out areas **********/

namespace clock {

  /* Are you using an external crystal, resonator, or RC oscillator? **********/
#define USING_EXTERNAL_CRYSTAL
  /***** Comment the macro above to answer no, otherwise your answer is yes ***/

#ifndef USING_EXTERNAL_CRYSTAL

  /* Are you using an external clock? *****************************************/
//#define USING_EXTERNAL_CLOCK
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#endif /* USING_EXTERNAL_CRYSTAL */

#if defined USING_EXTERNAL_CRYSTAL  || \
    defined USING_EXTERNAL_CLOCK

  /* Insert the external clock frequency (in Hz), in the following line *******/
  enum {
    _SOURCE = 8000000
  };
  /******** Insert the external clock frequency (in Hz), in the previous line */

#endif /* USING_EXTERNAL_CRYSTAL || USING_EXTERNAL_CLOCK */

  /* Do you want to use the PLL? **********************************************/
#define USING_PLL
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#ifdef USING_PLL
#if defined STM32F2XX || \
    defined STM32F4XX || \
    (defined CONNECTIVITY_LINE && \
     (defined USING_EXTERNAL_CRYSTAL || \
      defined USING_EXTERNAL_CLOCK))

  /* Do you want to use the PLL for the I2S module? ***************************/
#define USING_I2S_PLL
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#endif
#if defined STM32F2XX || \
    defined STM32F4XX || \
    (not defined VALUE_LINE && \
     (defined EXTERNAL_CLOCK || \
      defined EXTERNAL_CRYSTAL))

  /* Are you going to use the USB module? *************************************/
#define USING_USB
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#endif /* STM32F2XX || STM32F4XX || ~VALUE_LINE */
#endif /* USING_PLL */

#ifdef STM32F1XX

  /****************************************************************************
   *                              STM32F1 FAMILY                              *
   ****************************************************************************/

#ifdef USING_PLL
#if defined USING_EXTERNAL_CLOCK || \
    defined USING_EXTERNAL_CRYSTAL
#ifdef VALUE_LINE

  /*****************************************************************************
   * The PLL clocking scheme is as shown below:
   *
   *      x(2 + PLLMUL)/(PREDIV1 + 1)
   * HSE -----------------------------> SYSTEM
   *
   * Define the PLL parameters below  *****************************************/
  enum {
    _PREDIV1 = 0,
    _PLLMUL = 1,
  };
  /******************************************* Define the PLL paramters above */

#elif defined CONNECTIVITY_LINE
#ifdef USING_I2S_PLL

  /*****************************************************************************
   * The PLL clocking scheme is as shown below:
   *
   *                        x (PLL2MUL + 2)   x (PLLMUL + 2)  / (PREDIV1 + 1)
   *                      +---------------------------------------------------+
   *      1/(PREDIV2 + 1) |x(PLL2MUL=15 + 5) x(PLLMUL=13 / 2)                 |
   * HSE -----------------+                                          SYSTEM <-+
   *                      | x (PLL3MUL + 2)
   *                      +-----------------> I2S
   *                       x(PLL3MUL=15 + 5)
   *
   * Define the PLL parameters below  *****************************************/
  enum {
    _PREDIV1 = 4,
    _PREDIV2 = 4,
    _PLLMUL = 7,
    _PLL2MUL = 6,
    _PLL3MUL = 15,
  };
  /******************************************* Define the PLL paramters above */

#else /* USING_I2S_PLL */

  /*****************************************************************************
   * The PLL clocking scheme is as shown below:
   *
   *      1/(PREDIV2 + 1)  x (PLL2MUL + 2)   x (PLLMUL + 2)  / (PREDIV1 + 1)
   * HSE --------------------------------------------------------------------+
   *                      x(PLL2MUL=15 + 5) x(PLLMUL=13 / 2)                 |
   *                                                                         |
   *                                                                 SYSTEM<-+
   *
   * Define the PLL parameters below  *****************************************/
  enum {
    _PREDIV1 = 0,
    _PREDIV2 = 0,
    _PLLMUL = 0,
    _PLLMUL2 = 0,
  };
  /******************************************* Define the PLL paramters above */

#endif /* USING_I2S_PLL */
#else /* VALUE_LINE */

  /*****************************************************************************
   * The PLL clocking scheme is as shown below:
   *
   *      x(2 + PLLMUL) / (1 + PLLXTPRE)
   * HSE --------------------------------> SYSTEM / I2S
   *
   * Define the PLL parameters below  *****************************************/
  enum {
    _PLLXTPRE = 0,
    _PLLMUL = 0,
  };
  /******************************************* Define the PLL paramters above */

#endif /* VALUE_LINE */
#else /* EXTERNAL_CLOCK || EXTERNAL_CRYSTAL */

  /*****************************************************************************
   * The PLL clocking scheme is as shown below:
   *
   *             x (2 + PLLMUL)
   * (HSI / 2) -----------------> SYSTEM
   *            x(PLLMUL=13 / 2)
   * Define the PLL parameters below  *****************************************/
  enum {
    _PLLMUL = 7,
  };
  /******************************************* Define the PLL paramters above */

#endif /* EXTERNAL_CLOCK || EXTERNAL_CRYSTAL */
#endif /* USING_PLL */
#ifdef USING_USB

  /*****************************************************************************
   * The USB bus clocking scheme is as shown below:
   *
   *         x 2 / (3 - USBPRE)
   * SYSTEM --------------------> USB
   *
   * Define the USB prescaler below  ******************************************/
  enum {
    _USBPRE = 0,
  };
  /******************************************* Define the USB prescaler above */

#endif

  /*****************************************************************************
   * The bus clocking scheme is shown below:
   *                            1/2^PPRE1
   *                          +----------->APB1CLK
   *        1/2^HPRE          |
   * SYSCLK---------->AHBCLK--+
   *                          | 1/2^PPRE2          1/(2*ADCPRE + 2)
   *                          +----------->APB2CLK------------------>ADCCLK
   * Define the prescaler parameters below ************************************/
  enum {
    _HPRE = 0,
    _PPRE1 = 0,
    _PPRE2 = 0,
    _ADCPRE = 0,
  };
  /************************************ Define the prescaler parameters above */

#else /* STM32F1XX */

  /****************************************************************************
   *                        STM32F2 AND STM32F4 FAMILY                        *
   ****************************************************************************/

#ifdef USING_PLL

  /*****************************************************************************
   * The PLL clocking scheme is as shown below:
   *
   *                                        1/PLLP
   * (HSE 1/PLLM         xPLLN             +------> SYSTEM
   *  or -------->VCO_IN------->VCO_OUT----|
   * HSI)                                  +------> USB/SDIO/RNG
   *                                        1/PLLQ
   *
   * Define the PLL parameters below  *****************************************/
  enum {
    _PLLM = 4,
    _PLLN = 168,
    _PLLP = 2,
    _PLLQ = 7,
  };
  /****************************************** Define the PLL parameters above */

#endif /* USING_PLL */

#ifdef USING_I2S_PLL

  /*****************************************************************************
   * The PLL clocking scheme is shown below:
   *
   * (HSE 1/PLLM         xPLLI2SN              1/PLLI2SR
   *  or -------->VCO_IN---------->VCO_I2S_OUT----------->I2SCLK
   * HSI)
   *
   * Define the I2S PLL parameters below  *************************************/
  enum {
    _PLLI2SN = 192,
    _PLLI2SR = 4,
  };
  /************************************** Define the I2S PLL parameters above */

#endif /* USING_I2S_PLL */

  /*****************************************************************************
   * The bus clocking scheme is shown below:
   *                            1/2^PPRE1
   *                          +----------->APB1CLK
   *        1/2^HPRE          |
   * SYSCLK---------->AHBCLK--+
   *                          | 1/2^PPRE2
   *                          +----------->APB2CLK
   * Define the prescaler parameters below ************************************/
  enum {
    _HPRE = 0,
    _PPRE1 = 2,
    _PPRE2 = 1,
  };
  /************************************ Define the prescaler parameters above */

#if defined USING_EXTERNAL_CRYSTAL || \
  defined USING_EXTERNAL_CLOCK

  /* Do you want to use the external clock to drive the RTC? ******************/
//#define USING_HSE_FOR_RTC
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#endif /* USING_EXTERNAL_CRYSTAL || USING_EXTERNAL_CLOCK */
#ifdef USING_HSE_FOR_RTC

  /* Insert the RC prescaler value in the following line **********************/
  enum {_RTCPRE = 8};
  /********************** Insert the RTC prescaler value in the previous line */

#endif /* USING_HSE_FOR_RTC */
#endif /* STM32F1XX */

  /* Insert the flash access latency (in wait states) in the following line ***/
  enum {
    _LATENCY = 3
  };
  /****** Insert the flash access latency (in wait states) in the previous line */
  /* IMPORTANT: USING A LOW LATENCY AT HIGH CLOCK FREQUENCIES MIGHT RESULT IN
   *            FLASH ACCESS ERRORS AT RUN TIME. ******************************/

  /**
   * @brief Configures the clock, using the configuration inputted in clock.hpp
   * @note  Call this function as early as possible in you program.
   */
  static INLINE void initialize();

#if defined USING_EXTERNAL_CRYSTAL || defined USING_EXTERNAL_CLOCK
  /**
   * @brief This function handles the failure of the HSE oscillator.
   * @note  The user must define this function.
   */
  static INLINE void hseFailureHandler(void);
#endif
}  // namespace clock

#include "../bits/clock.tcc"
