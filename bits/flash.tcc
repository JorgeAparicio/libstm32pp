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

namespace flash {

#ifndef VALUE_LINE
  void Functions::setLatency(acr::latency::States LATENCY)
  {
    FLASH_REGS->ACR &= ~acr::latency::MASK;
    FLASH_REGS->ACR |= LATENCY;
  }
#endif // !VALUE_LINE
#ifdef STM32F1XX
#ifndef VALUE_LINE
  /**
   * @brief Enables the prefetch buffer.
   */
  void Functions::enablePrefetch()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + flash::acr::OFFSET,
        flash::acr::prftbe::POSITION
        >()) = 1;
  }

  /**
   * @brief Disables the prefetch buffer.
   */
  void Functions::disablePrefetch()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + flash::acr::OFFSET,
        flash::acr::prftbe::POSITION
        >()) = 0;
  }
#endif // !VALUE_LINE
  /**
   * @brief Enables the half cycle flash access.
   */
  void Functions::enableHalfCycleFlashAccess()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + flash::acr::OFFSET,
        flash::acr::hlfcya::POSITION
        >()) = 1;
  }

  /**
   * @brief Disables the half cycle flash access.
   */
  void Functions::disableHalfCycleFlashAccess()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + flash::acr::OFFSET,
        flash::acr::hlfcya::POSITION
        >()) = 0;
  }
#ifdef VALUE_LINE

  /**
   * @brief Configures the flash memory access.
   * @note  Overrides the old configuration.
   */
  void Functions::configure(acr::hlfcya::States HLFCYA)
  {
    FLASH_REGS->ACR = HLFCYA;
  }

#else // VALUE_LINE
  /**
   * @brief Configures the flash memory access.
   * @note  Overrides the old configuration.
   */
  void Functions::configure(
      acr::latency::States LATENCY,
      acr::hlfcya::States HLFCYA,
      acr::prftbe::States PRFTBE)
  {
    FLASH_REGS->ACR = LATENCY + HLFCYA + PRFTBE;
  }
#endif // VALUE_LINE
#else // STM32F1XX
  /**
   * @brief Enables the prefetch buffer.
   */
  void Functions::enablePrefetch()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + flash::acr::OFFSET,
        flash::acr::prften::POSITION
    >()) = 1;
  }

  /**
   * @brief Disables the prefetch buffer.
   */
  void Functions::disablePrefetch()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + flash::acr::OFFSET,
        flash::acr::prften::POSITION
    >()) = 0;
  }

  /**
   * @brief Enables the data cache.
   */
  void Functions::enableDataCache()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + flash::acr::OFFSET,
        flash::acr::dcen::POSITION
    >()) = 1;
  }

  /**
   * @brief Disables the data cache.
   */
  void Functions::disableDataCache()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + flash::acr::OFFSET,
        flash::acr::dcen::POSITION
    >()) = 0;
  }

  /**
   * @brief Enables the instruction cache.
   */
  void Functions::enableInstructionCache()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + flash::acr::OFFSET,
        flash::acr::icen::POSITION
    >()) = 1;
  }

  /**
   * @brief Disables the instruction cache.
   */
  void Functions::disableInstructionCache()
  {
    *(u32 volatile*) (bitband::peripheral<
        ADDRESS + flash::acr::OFFSET,
        flash::acr::icen::POSITION
    >()) = 0;
  }

  /**
   * @brief Configures the flash memory access.
   * @note  Overrides the old configuration.
   */
  void Functions::configure(
      acr::latency::States LATENCY,
      acr::prften::States PRFTEN,
      acr::dcen::States DCEN,
      acr::icen::States ICEN)
  {
    FLASH_REGS->ACR = LATENCY + PRFTEN + DCEN + ICEN;
  }
#endif // STM32F1XX
}  // namespace flash
