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

namespace nvic {
  /**
   * @brief Enables an interrupt request.
   */
  template<irqn::E I>
  void Functions::enableIrq(void)
  {
    reinterpret_cast<Registers*>(ADDRESS)->ISER[I >> 5] = 1 << (I % 32);
  }

  /**
   * @brief Disables an interrupt request.
   */
  template<irqn::E I>
  void Functions::disableIrq(void)
  {
    reinterpret_cast<Registers*>(ADDRESS)->ICER[I >> 5] = 1 << (I % 32);
  }

  /**
   * @brief Sets the interrupt priority level.
   * @note  A lower priority number, means higher priority.
   */
  template<irqn::E I, u8 P>
  void Functions::setPriority()
  {
    static_assert(P % 32 == 31,
        "The priority number must have it 4 lower bits set to 1.");

    reinterpret_cast<Registers*>(ADDRESS)->IPR[I >> 2] &=
        irqn::MASK << (8 * (I % 4));

    reinterpret_cast<Registers*>(ADDRESS)->IPR[I >> 2] |=
        P << (8 * (I % 4));
  }
}  // namespace nvic
