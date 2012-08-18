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
#include "peripheral/rcc.hpp"
#include "peripheral/flash.hpp"

#pragma once

/* Enter your clock configuration below, ignore the grayed out areas **********/

namespace clock {

  /****************************************************************************
   *                                                                          *
   *                               CLOCK SOURCES                              *
   *                                                                          *
   ****************************************************************************/

  /* Are you using an external high speed crystal, resonator or oscillator? ***/
//#define USING_HSE_CRYSTAL
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#ifndef USING_HSE_CRYSTAL

  /* Are you using a high speed external clock? *******************************/
//#define USING_HSE_CLOCK
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#endif // USING_HSE_CRYSTAL
#if defined USING_HSE_CRYSTAL  || \
    defined USING_HSE_CLOCK

  /* Insert the HSE clock frequency (in Hz) ***********************************/
  enum {
    HSE = 8000000
  };
  /*********************************** Insert the HSE clock frequency (in Hz) */

  /* Insert the allowed HSE clock stabilization's time (in cycles) ************/
  enum {
    _HSE_TIMEOUT = 0x800
  };
  /************ Insert the allowed HSE clock stabilization's time (in cycles) */

  /**
   * @brief This function is called when the external clock fails.
   * @note  The user must define this function.
   */
  void hseFailureHandler(void);

#endif // USING_HSE_CRYSTAL || USING_HSE_CLOCK
  /* Are you using an external low speed crystal, resonator or oscillator? ****/
//#define USING_LSE_CRYSTAL
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#ifndef USING_LSE_CRYSTAL

  /* Are you using a low speed external clock? *******************************/
//#define USING_LSE_CLOCK
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#endif // USING_LSE_CRYSTAL
#if defined USING_LSE_CRYSTAL  || \
  defined USING_LSE_CLOCK

  /* Insert the LSE clock frequency (in Hz) ***********************************/
  enum {
    LSE = 32768
  };
  /*********************************** Insert the LSE clock frequency (in Hz) */

  /* Insert the allowed LSE clock stabilization's time (in cycles) ************/
  enum {
    _LSE_TIMEOUT = 0x800
  };
  /************ Insert the allowed LSE clock stabilization's time (in cycles) */

  /**
   * @brief This function is called when the external clock fails.
   * @note  The user must define this function.
   */
  void lseFailureHandler(void);

#endif // USING_LSE_CRYSTAL || USING_LSE_CLOCK
  /* Do you want to use the LSI? **********************************************/
//#define USING_LSI
  /******* Comment the macro above to answer no, otherwise your answer is yes */

  /****************************************************************************
   *                                                                          *
   *                       REAL TIME CLOCK CONFIGURATION                      *
   *                                                                          *
   ****************************************************************************/

  /* Are you going to use the Real time clock module? *************************/
//#define USING_RTC
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#ifdef USING_RTC
#ifdef STM32F1XX
  /*****************************************************************************
   * The RTC bus clocking scheme is shown below:
   *
   *     1 / 128
   * HSE---------+
   *             |
   * LSE---------+----> RTC
   *             |
   * LSI---------+
   *
   * Define the prescaler parameters ******************************************/
  enum {
    _RTCSEL = rcc::bdcr::bits::rtcsel::states::
    LSE_CLOCK_AS_RTC_SOURCE
  };
  /****************************************** Define the prescaler parameters */
#else // STM32F1XX
  /*****************************************************************************
   * The RTC bus clocking scheme is shown below:
   *
   *     1 / RTCPRE
   * HSE------------+
   *                |
   * LSE------------+----> RTC
   *                |
   * LSI------------+
   *
   * Define the prescaler parameters ******************************************/
  enum {
    _RTCPRE = 8,
    _RTCSEL = rcc::bdcr::bits::rtcsel::states::
    LSE_CLOCK_AS_RTC_SOURCE
  };
  /****************************************** Define the prescaler parameters */
#endif // STM32F1XX
#endif // USING_RTC
  /****************************************************************************
   *                                                                          *
   *                             PLL CONFIGURATION                            *
   *                                                                          *
   ****************************************************************************/

  /* Do you want to use the PLL? **********************************************/
//#define USING_PLL
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#ifdef USING_PLL

#ifdef STM32F1XX
  /*****************************************************************************
   *
   * PREDIV1_OUTPUT------+
   *                     |
   *                     +----> PLLSRC
   *                     |
   * (HSI / 2)-----------+
   *
   * Select the PLL source ****************************************************/
  enum {
    __PLLSRC = rcc::cfgr::pllsrc::states::
    USE_PREDIV1_OUTPUT_AS_PLL_SOURCE
  };
  /**************************************************** Select the PLL source */
#ifndef CONNECTIVITY_LINE
#ifdef VALUE_LINE
  /*****************************************************************************
   *
   *      1 / PREDIV1
   * HSE -------------> PREDIV1_OUTPUT
   *
   * Select the PREDIV1 source ************************************************/
  enum {
    _PREDIV1 = 1
  };
  /************************************************ Select the PREDIV1 source */
#else // VALUE_LINE
  /*****************************************************************************
   *
   *      1 / (PLLXTPRE + 1)
   * HSE --------------------> PREDIV1_OUTPUT
   *
   * Select the PREDIV1 source ************************************************/
  enum {
    _PLLXTPRE = 0
  };
  /************************************************ Select the PREDIV1 source */
#endif // VALUE_LINE
#else // !CONNECTIVITY_LINE
  /*****************************************************************************
   *
   *
   *
   *     1 / PREDIV2    x (PLL2MUL + 2)
   * HSE---------------------------------> PLL2
   *                   x(PLL2MUL=15 + 5)
   *
   * Configure the PLL parameters *********************************************/
  enum {
    _PREDIV2 = 2,
    _PLL2MUL = 7,
  };
  /********************************************* Configure the PLL parameters */

  /*****************************************************************************
   *
   * HSE-----+
   *         | 1 / PREDIV1
   *         +-------------> PREDIV1_OUTPUT
   *         |
   * PLL2----+
   *
   * Select the PREDIV1 source ************************************************/
  enum {
    _PREDIV1SRC = rcc::cfgr2::bits::prediv1src::states::
    USE_PLL2_AS_PREDIV1_INPUT,
    _PREDIV1 = 1
  };
  /************************************************ Select the PREDIV1 source */
#endif // !CONNECTIVITY_LINE
  /*****************************************************************************
   *
   *         x PLLMUL
   * PLLSRC ----------> PLL
   *
   * Select the PLL parameters ************************************************/
  enum {
    _PLLMUL = 2
  };
  /************************************************ Select the PLL paraneters */
#else // STM32F1XX
  /****************************************************************************
   *
   * HSE----+
   *        |
   *        +----> PLLSRC
   *        |
   * HSI----+
   *
   * Select the PLL source  ***************************************************/
  enum {
    __PLLSRC = rcc::pllcfgr::pllsrc::states::
    USE_HSE_CLOCK_AS_PLL_CLOCK_SOURCE
  };
  /*************************************************** Select the PLL source  */

  /*****************************************************************************
   * The PLL clocking scheme is as shown below:
   *
   *                                             1/PLLP
   *          1/PLLM         xPLLN             +--------> SYSTEM
   * PLL_SRC -------->VCO_IN------->VCO_OUT----|
   *                                           +--------> USB/SDIO/RNG
   *                                             1/PLLQ
   *
   * Define the PLL parameters below  *****************************************/
  enum {
    _PLLM = 4,
    _PLLN = 144,
    _PLLP = 8,
    _PLLQ = 6,
  };
  /****************************************** Define the PLL parameters above */

#endif // STM32F1XX
#endif // USING_PLL
  /****************************************************************************
   *                                                                          *
   *                          SYSTEM CLOCK SELECTION                          *
   *                                                                          *
   ****************************************************************************/

  /* Select the system clock source *******************************************/
  enum {
    _SW = rcc::cfgr::sw::HSI_OSCILLATOR_SELECTED_AS_SYSTEM_CLOCK
  };
  /******************************************* Select the system clock source */
  /****************************************************************************
   *                                                                          *
   *                             PERIPHERAL CLOCKS                            *
   *                                                                          *
   ****************************************************************************/

#if defined USING_PLL && \
    (defined STM32F2XX || \
     defined STM32F4XX || \
     (not defined VALUE_LINE && \
      not defined CONNECTIVITY_LINE))

  /* Are you going to use the USB module? *************************************/
//#define USING_USB
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#if defined USING_USB && \
    not defined STM32F2XX && \
    not defined STM32F4XX

  /*****************************************************************************
   *
   *      2 / (USBPRE + 2)
   * PLL ------------------> USB
   *
   * Define the USB prescaler *************************************************/
  enum {
    _USBPRE = 0
  };
  /************************************************* Define the USB prescaler */

#endif

#endif // STM32F2XX || STM32F4XX || (!VALUE_LINE && !CONNECTIVITY_LINE)
#if defined CONNECTIVITY_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX

  /* Are you going to use the Ethernet module? ********************************/
//#define USING_ETHERNET
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#endif // CONNECTIVITY_LINE || STM32F2XX || STM32F4XX
#ifndef VALUE_LINE

  /* Are you going to use the I2S module? *************************************/
//#define USING_I2S
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#ifdef USING_I2S

#ifdef STM32F1XX
#ifdef CONNECTIVITY_LINE

  /* Do you want to use the I2S PLL? ******************************************/
#define USING_I2S_PLL
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#ifdef USING_I2S_PLL
  /*****************************************************************************
   *
   *
   *
   *     1 / PREDIV2    x (PLL3MUL + 2)
   * HSE---------------------------------> PLL3
   *                   x(PLL3MUL=15 + 5)
   *
   * Configure the PLL parameters *********************************************/
  enum {
#ifndef USING_PLL
    _PREDIV2 = 2,
#endif // USING_PLL
    _PLL3MUL = 7,
  };
  /********************************************* Configure the PLL parameters */
#endif // USING_I2S_PL
  /*****************************************************************************
   *
   * PLL3------+
   *           |
   *           +----> I2S2/I2S3
   *           |
   * SYSTEM----+
   *
   * Define the I2S sources below  ********************************************/
  enum {
    _I2S2SRC = rcc::cfgr2::bits::i2s2src::states::
    USE_SYSTEM_CLOCK_AS_I2S2_CLOCK,
    _I2S3SRC = rcc::cfgr2::bits::i2s3src::states::
    USE_SYSTEM_CLOCK_AS_I2S3_CLOCK,
  };
  /********************************************* Define the I2S sources below */
#endif // CONNECTIVITY_LINE
#else // STM32F1XX
  /* Do you want to use the I2S PLL? ******************************************/
#define USING_I2S_PLL
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#ifdef USING_I2S_PLL
#ifndef USING_PLL
  /****************************************************************************
   *
   * HSE----+
   *        |
   *        +----> PLLSRC
   *        |
   * HSI----+
   *
   * Select the PLL source  ***************************************************/
  enum {
    __PLLSRC = rcc::pllcfgr::pllsrc::states::
    USE_HSE_CLOCK_AS_PLL_CLOCK_SOURCE
  };
  /*************************************************** Select the PLL source  */
#endif // USING_PLL
  /*****************************************************************************
   * The PLL clocking scheme is as shown below:
   *
   *          1 / PLLM         xPLLI2SN              1 / PLLI2SR
   * PLL_SRC ---------->VCO_IN---------->VCO_I2S_OUT------------->PLLI2S----+
   *                                                                        |
   *                                                               I2S <----+
   *
   * Define the PLL parameters below  *****************************************/
  enum {
#ifndef USING_PLL
    _PLLM = 4,
#endif
    _PLLI2SN = 192,
    _PLLI2SR = 2,
  };
  /****************************************** Define the PLL parameters above */
#else // USING_I2S_PLL
  /*****************************************************************************
   *
   * I2S_CKIN---->I2S
   *
   ****************************************************************************/
#endif // USING_I2S_PLL
#endif // STM32F1XX
#endif // USING_I2S
#endif // !VALUE_LINE
  /****************************************************************************
   *                                                                          *
   *                              BUS PRESCALERS                              *
   *                                                                          *
   ****************************************************************************/

  /*****************************************************************************
   * The bus clocking scheme is shown below:
   *
   *                            1/2^PPRE1
   *                          +----------->APB1CLK
   *        1/2^HPRE          |
   * SYSCLK---------->AHBCLK--+
   *                          | 1/2^PPRE2
   *                          +----------->APB2CLK
   *
   * Define the prescaler parameters below ************************************/
  enum {
    _HPRE = 0,
    _PPRE1 = 0,
    _PPRE2 = 0,
  };
  /************************************ Define the prescaler parameters above */

#ifdef STM32F1XX
  /*****************************************************************************
   * The ADC bus clocking scheme is shown below:
   *
   *         1 / (2 *ADCPRE + 2)
   * APB2CLK-------------------->ADCCLK
   *
   * Define the prescaler parameters below ************************************/
  enum {
    _ADCPRE = 1,
  };
  /************************************ Define the prescaler parameters above */
#endif

  /****************************************************************************
   *                                                                          *
   *                       MICROCONTROLLER CLOCK OUTPUT                       *
   *                                                                          *
   ****************************************************************************/

  /* Do you want to use the Microcontroller Clock Output? *********************/
//#define USING_MCO
  /******* Comment the macro above to answer no, otherwise your answer is yes */

#ifdef USING_MCO

#ifdef STM32F1XX
  /*****************************************************************************
   * The microcontroller clock output scheme is shown bellow:
   *
   * (SYSTEM | HSI | HSE | PLL | XT1) ----> MCO
   *
   ****************************************************************************/
  enum {
    _MCO = rcc::cfgr::mco::states::OUTPUT_HSE_CLOCK
  };
#else // STM32F1XX
  /*****************************************************************************
   * The microcontroller clock output scheme is shown bellow:
   *
   *                          1 / MCOPRE1
   * (HSI | LSE | HSE | PLL) -------------> MCO1
   *
   *                                1 / MCOPRE2
   * (SYSTEM | PLLI2S | HSE | PLL) -------------> MCO2
   *
   *
   ****************************************************************************/
  enum {
    _MCO1 = rcc::cfgr::mco1::states::OUTPUT_HSI_CLOCK,
    _MCO2 = rcc::cfgr::mco2::states::OUTPUT_HSE_CLOCK,
    _MCO1PRE = 1,
    _MCO2PRE = 1,
  };
#endif // STM32F1XX
#endif // USING_MCO
#ifndef VALUE_LINE
  /****************************************************************************
   *                                                                          *
   *                        FLASH MEMORY ACCESS LATENCY                       *
   *                                                                          *
   ****************************************************************************/

  /* Select the flash memory access latency ***********************************/
  enum {
    _LATENCY = flash::registers::acr::bits::latency::states::ZERO_WAIT_STATE
  };
  /*********************************** Select the flash memory access latency */
  /* IMPORTANT: USING A LOW LATENCY AT HIGH CORE'S FREQUENCY MIGHT RESULT IN
   *            FLASH MEMORY ACCESS ERRORS AT RUN TIME. ***********************/
#endif // !VALUE_LINE
  /**
   * @brief Initializes the clock.
   * @note  Call this function as early as possible in your program.
   * @note  The configuration of the clock is specified in clock.hpp
   */
  static INLINE void initialize();
}  // namespace clock

#include "../bits/clock.tcc"
