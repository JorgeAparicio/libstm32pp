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

#include "common.hpp"

namespace sdio {
  struct Registers {
      __RW
      u32 POWER;         // 0x00: Power control
      __RW
      u32 CLKCR;         // 0x04: Clock control
      __RW
      u32 ARG;           // 0x08: Argument
      __RW
      u32 CMD;           // 0x0C: Command
      __R
      u32 RESPCMD;       // 0x10: Command response
      __R
      u32 RESP[4];       // 0x14-0x20: Response
      __RW
      u32 DTIMER;        // 0x24: Data timer
      __RW
      u32 DLEN;          // 0x28: Data length
      __RW
      u32 DCTRL;         // 0x2C: Data control
      __R
      u32 DCOUNT;        // 0x30: Data counter
      __R
      u32 STA;           // 0x34: Status
      __RW
      u32 ICR;           // 0x38: Interrupt clear
      __RW
      u32 MASK;          // 0x3C: Mask
      u32 _RESERVED0[2];
      __R
      u32 FIFOCNT;       // 0x48: FIFO counter
      u32 _RESERVED1[13];
      __RW
      u32 FIFO;          // 0x80: Data FIFO
  };

#ifdef STM32F1XX
  enum {ADDRESS = alias::address::PERIPH + 0x18000};
#else
  enum {
    ADDRESS = alias::address::APB2 + 0x2C00
  };
#endif

  namespace registers {
  // TODO SDIO register bits
  }// namespace registers
}  // namespace sdio
