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

namespace fpu {

  void Functions::enableFullAccess()
  {
    _FPU->CPACR = cpacr::cp10::FULL_ACCESS +
        cpacr::cp11::FULL_ACCESS;
  }

  void Functions::enablePrivilegedAccess()
  {
    _FPU->CPACR = cpacr::cp10::PRIVILEGED_ACCESS +
        cpacr::cp11::PRIVILEGED_ACCESS;
  }

  void Functions::disable()
  {
    _FPU->CPACR = 0;
  }

}  // namespace fpu
