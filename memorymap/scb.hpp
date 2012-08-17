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

namespace scb {

  struct Registers {
      __RW
      u32 CPUID;  // 0x00: CPU ID
      __RW
      u32 ICSR;  // 0x04: Interrupt control and state
      __RW
      u32 VTOR;  // 0x08: Vector table offset
      __RW
      u32 AIRCR;  // 0x0C: Application interrupt and reset control
      __RW
      u32 SCR;  // 0x10: System control
      __RW
      u32 CCR;  // 0x14: Configuration and control
      __RW
      u32 SHPR1;  // 0x18: System handler priority 1
      __RW
      u32 SHPR2;  // 0x1C: System handler priority 2
      __RW
      u32 SHPR3;  // 0x20: System handler priority 3
      __RW
      u32 SHCRS;  // 0x24: System handler control and state
      __RW
      u32 CFSR;  // 0x28: Configurable fault status
      __RW
      u32 HFSR;  // 0x2C: Hard fault status
      __RW
      u32 MMAR;  // 0x30: Memory management fault address
      __RW
      u32 BFAR;  // 0x34: Bus fault address
      __RW
      u32 AFSR;  // 0x38: Auxiliary fault status
  };

  enum {
    ADDRESS = alias::PPB + 0xD00
  };

  namespace registers {
  // TODO SCB register bits
  }// namespace registers

}  // namespace scb

