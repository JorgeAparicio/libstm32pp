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

namespace clk {

  /****************************************************************************
   *                                                                          *
   *                               CLOCK SOURCES                              *
   *                                                                          *
   ****************************************************************************/

  /* Are you using an external high speed crystal, resonator or oscillator? ***/
#define USING_HSE_CRYSTAL
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
    __HSE_TIMEOUT = 0x800
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
    __LSE_TIMEOUT = 0x800
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
  rcc::bdcr::rtcsel::States const __RTCSEL = rcc::bdcr::rtcsel::
  LSE_CLOCK_AS_RTC_SOURCE;
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
    __RTCPRE = 8
  };
  rcc::bdcr::rtcsel::States const __RTCSEL = rcc::bdcr::rtcsel::
  LSE_CLOCK_AS_RTC_SOURCE;
  /****************************************** Define the prescaler parameters */
#endif // STM32F1XX
#endif // USING_RTC
  /****************************************************************************
   *                                                                          *
   *                             PLL CONFIGURATION                            *
   *                                                                          *
   ****************************************************************************/

  /* Do you want to use the PLL? **********************************************/
#define USING_PLL
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
  rcc::cfgr::pllsrc::States const __PLLSRC = rcc::cfgr::pllsrc::
  USE_PREDIV1_OUTPUT_AS_PLL_SOURCE;
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
    __PREDIV1 = 1
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
    __PLLXTPRE = 0
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
    __PREDIV2 = 2,
    __PLL2MUL = 7,
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
    __PREDIV1 = 1
  };
  rcc::cfgr2::prediv1src::States const __PREDIV1SRC =
  rcc::cfgr2::prediv1src::
  USE_PLL2_AS_PREDIV1_INPUT;
  /************************************************ Select the PREDIV1 source */
#endif // !CONNECTIVITY_LINE
  /*****************************************************************************
   *
   *         x PLLMUL
   * PLLSRC ----------> PLL
   *
   * Select the PLL parameters ************************************************/
  enum {
    __PLLMUL = 2
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
  rcc::pllcfgr::pllsrc::States const __PLLSRC =
  rcc::pllcfgr::pllsrc::
  USE_HSE_CLOCK_AS_PLL_CLOCK_SOURCE;
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
    __PLLM = 4,
    __PLLN = 168,
    __PLLP = 2,
    __PLLQ = 7,
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
  rcc::cfgr::sw::States const __SW =
      rcc::cfgr::sw::HSE_OSCILLATOR_SELECTED_AS_SYSTEM_CLOCK;
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
    __USBPRE = 0
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
    __PREDIV2 = 2,
#endif // USING_PLL
    __PLL3MUL = 7,
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
  rcc::cfgr2::i2s2src::States const __I2S2SRC = rcc::cfgr2::i2s2src::
  USE_SYSTEM_CLOCK_AS_I2S2_CLOCK;
  rcc::cfgr2::i2s3src::States const __I2S3SRC = rcc::cfgr2::i2s3src::
  USE_SYSTEM_CLOCK_AS_I2S3_CLOCK;
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
  rcc::pllcfgr::pllsrc::States const __PLLSRC =
  rcc::pllcfgr::pllsrc::
  USE_HSE_CLOCK_AS_PLL_CLOCK_SOURCE;
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
    __PLLM = 4,
#endif
    __PLLI2SN = 192,
    __PLLI2SR = 2,
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
    __HPRE = 0,
    __PPRE1 = 2,
    __PPRE2 = 1,
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
    __ADCPRE = 1,
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
  rcc::cfgr::mco::States const __MCO =
  rcc::cfgr::mco::OUTPUT_HSE_CLOCK;
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
  rcc::cfgr::mco1::States const __MCO1 = rcc::cfgr::mco1::
  OUTPUT_HSI_CLOCK;

  rcc::cfgr::mco2::States const __MCO2 = rcc::cfgr::mco2::
  OUTPUT_HSE_CLOCK;

  enum {
    __MCO1PRE = 1,
    __MCO2PRE = 1,
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
  flash::acr::latency::States const __LATENCY =
      flash::acr::latency::ZERO_WAIT_STATE;
  /*********************************** Select the flash memory access latency */
  /* IMPORTANT: USING A LOW LATENCY AT HIGH CORE'S FREQUENCY MIGHT RESULT IN
   *            FLASH MEMORY ACCESS ERRORS AT RUN TIME. ***********************/
#endif // !VALUE_LINE
  /**
   * @brief Initializes the clock.
   * @note  Call this function as early as possible in your program.
   * @note  The configuration of the clock is specified in clock.hpp
   */
  static inline void initialize();
}  // namespace clk

#include "../bits/clock.tcc"
