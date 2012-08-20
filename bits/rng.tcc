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
#include "../include/peripheral/rcc.hpp"

namespace rng {
  /**
   * @brief Enables the Random Number Generator clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  void Functions::enableClock()
  {
    RCC::enableClocks<
        rcc::ahb2enr::RNG
    >();
  }

  /**
   * @brief Disables the Random Number Generator clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  void Functions::disableClock()
  {
    RCC::disableClocks<
        rcc::ahb2enr::RNG
    >();
  }

  /**
   * @brief Enables the Random Number Generator.
   */
  void Functions::startGenerator()
  {
    RNG_REGS->CR |=
        cr::rngen::RANDOM_NUMBER_GENERATOR_ENABLED;
  }

  /**
   * @brief Disables the Random Number Generator.
   */
  void Functions::stopGenerator()
  {
    RNG_REGS->CR &=
        ~cr::rngen::RANDOM_NUMBER_GENERATOR_ENABLED;
  }

  /**
   * @brief Enables the interrupt.
   */
  void Functions::enableInterrupts()
  {
    RNG_REGS->CR |=
        cr::ie::INTERRUPT_ENABLED;
  }

  /**
   * @brief Disables the interrupt.
   */
  void Functions::disableInterrupts()
  {
    RNG_REGS->CR &=
        ~cr::ie::INTERRUPT_ENABLED;
  }

  /**
   * @brief Gets a random number.
   * @note  User should check if the random number is valid first.
   */
  template<typename T>
  T Functions::getValue()
  {
    return *(T*) (ADDRESS + dr::OFFSET);
  }

  /**
   * @brief Returns true if there is data ready to be read.
   */
  bool Functions::isDataReady()
  {
    return (RNG_REGS->SR &
        sr::drdy::VALID_RANDOM_DATA_READY);
  }

  /**
   * @brief Returns true if the seed is valid.
   */
  bool Functions::isSeedValid()
  {
    return (RNG_REGS->SR & sr::secs::SEED_OK);
  }

  /**
   * @brief Returns true if the clock is valid.
   */
  bool Functions::isClockValid()
  {
    return (RNG_REGS->SR & sr::cecs::CLOCK_OK);
  }

  /**
   * @brief Clears the seed error flag.
   */
  void Functions::clearSeedErrorFlag()
  {
    RNG_REGS->SR &=
        ~sr::seis::SEED_ERROR_DETECTED;
  }

  /**
   * @brief Clears the seed error flag.
   */
  void Functions::clearClockErrorFlag()
  {
    RNG_REGS->SR &=
        ~sr::ceis::CLOCK_ERROR_DETECTED;
  }

}  // namespace rng
