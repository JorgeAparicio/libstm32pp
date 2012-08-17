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

namespace can {
  struct Registers {
      __RW
      u32 MCR;      // 0x000: Master control
      __RW
      u32 MSR;      // 0x004: Master status
      __RW
      u32 TSR;      // 0x008: Transmit status
      __RW
      u32 RFR[2];     // 0x00C: Receive FIFO
      __RW
      u32 IER;      // 0x014: Interrupt enable
      __RW
      u32 ESR;      // 0x018: Error status
      __RW
      u32 BTR;      // 0x01C: Bit timing
      u32 _RESERVED0[88];
      struct {
          __RW
          u32 IR;   // 0x180, 0x190, 0x1A0: identifier
          __RW
          u32 DTR;  // 0x184, 0x194, 0x1A4: data length control and time stamp
          __RW
          u32 DLR;  // 0x188, 0x198, 0x1A8: data low
          __RW
          u32 DHR;  // 0x18C, 0x19C, 0x1AC: data high
      } TX[3];  // TX FIFO mailbox
      struct {
          __RW
          u32 IR;   // 0x1B0, 0x1C0: Identifier
          __RW
          u32 DTR;  // 0x1B4, 0x1C4: Data length control and time stamp
          __RW
          u32 DLR;  // 0x1B8, 0x1C8: Data low
          __RW
          u32 DHR;  // 0x1BC, 0x1CC: Data high
      } RX[2];  // RX FIFO mailbox
      u32 _RESERVED1[12];
      __RW
      u32 FMR;      // 0x200: Filter master
      __RW
      u32 FM1R;     // 0x204: Filter mode
      u32 _RESERVED2;
      __RW
      u32 FS1R;     // 0x20C: Filter scale
      u32 _RESERVED3;
      __RW
      u32 FFA1R;    // 0x214: Filter FIFO assignment
      u32 _RESERVED4;
      __RW
      u32 FA1R;     // 0x21C: Filter activation
      u32 _RESERVED5[8];
      struct {
          __RW
          u32 L;    // 0x240 + 8 * I: Low
          __RW
          u32 H;    // 0x244 + 8 * I: High
#if defined CONNECTIVITY_LINE || \
    defined STM32F2XX || \
    defined STM32F4XX
      } FR[27];     // 0x240-0x31C: Filter bank
#else
      } FR[13];     // 0x240-0x2AC: Filter bank
#endif
  };

  namespace address {
    enum E {
      CAN1 = alias::APB1 + 0x6400,
      CAN2 = alias::APB1 + 0x6800
    };
  }  // namespace address

  namespace registers {
  // TODO CAN register bits
  }// namespace registers
}  // namespace can
