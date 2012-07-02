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
   * @brief Enables an interrupt.
   */
  template<irqn::E I>
  void Functions::enableInterrupt(void)
  {
    reinterpret_cast<Registers*>(ADDRESS)->ISER[I >> 5] = 1 << (I & 0x1F);
  }

  /**
   * @brief Disables an interrupt.
   */
  template<irqn::E I>
  void Functions::disableInterrupt(void)
  {
    reinterpret_cast<Registers*>(ADDRESS)->ICER[I >> 5] = 1 << (I & 0x1F);
  }

  /**
   * @brief Sets the interrupt priority level.
   */
  template<irqn::E I, u8 P>
  void Functions::setInterruptPriority()
  {
    static_assert(P >= 7, "The minimum interrupt level is 7");
    reinterpret_cast<Registers*>(ADDRESS)->IPR[I >> 2] = P << (8 * (I % 4));
  }
}  // namespace nvic
