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
      rcc::registers::ahb2enr::bits::RNG
    >();
  }

  /**
   * @brief Disables the Random Number Generator clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  void Functions::disableClock()
  {
    RCC::disableClocks<
      rcc::registers::ahb2enr::bits::RNG
    >();
  }

  /**
   * @brief Enables the Random Number Generator.
   */
  void Functions::startGenerator()
  {
    _RNG->CR |=
        registers::cr::bits::rngen::states::RANDOM_NUMBER_GENERATOR_ENABLED;
  }

  /**
   * @brief Disables the Random Number Generator.
   */
  void Functions::stopGenerator()
  {
    _RNG->CR &=
        ~registers::cr::bits::rngen::states::RANDOM_NUMBER_GENERATOR_ENABLED;
  }

  /**
   * @brief Enables the interrupt.
   */
  void Functions::enableInterrupts()
  {
    _RNG->CR |=
        registers::cr::bits::ie::states::INTERRUPT_ENABLED;
  }

  /**
   * @brief Disables the interrupt.
   */
  void Functions::disableInterrupts()
  {
    _RNG->CR &=
        ~registers::cr::bits::ie::states::INTERRUPT_ENABLED;
  }

  /**
   * @brief Gets a random number.
   * @note  User should check if the random number is valid first.
   */
  template<typename T>
  T Functions::getValue()
  {
    return *(T*) (ADDRESS + registers::dr::OFFSET);
  }

  /**
   * @brief Returns true if there is data ready to be read.
   */
  bool Functions::isDataReady()
  {
    return (_RNG->SR &
        registers::sr::bits::drdy::states::VALID_RANDOM_DATA_READY);
  }

  /**
   * @brief Returns true if the seed is valid.
   */
  bool Functions::isSeedValid()
  {
    return (_RNG->SR & registers::sr::bits::secs::states::SEED_OK);
  }

  /**
   * @brief Returns true if the clock is valid.
   */
  bool Functions::isClockValid()
  {
    return (_RNG->SR & registers::sr::bits::cecs::states::CLOCK_OK);
  }

  /**
   * @brief Clears the seed error flag.
   */
  void Functions::clearSeedErrorFlag()
  {
    _RNG->SR &=
        ~registers::sr::bits::seis::states::SEED_ERROR_DETECTED;
  }

  /**
   * @brief Clears the seed error flag.
   */
  void Functions::clearClockErrorFlag()
  {
    _RNG->SR &=
        ~registers::sr::bits::ceis::states::CLOCK_ERROR_DETECTED;
  }

}  // namespace rng
