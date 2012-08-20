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

namespace rtc {
  struct Registers {
      enum {
        ADDRESS = alias::APB1 + 0x2800
      };

#ifdef STM32F1XX
      __RW
      u32 CR[2];   // 0x00, 0x04: Control
      __RW
      u32 PRL[2];// 0x08, 0x1C: Prescaler load
      __RW
      u32 DIV[2];// 0x10, 0x14: Prescaler divider
      __RW
      u32 CNT[2];// 0x18, 0x1C: Counter
      __RW
      u32 ALR[2];// 0x20, 0x24: Alarm
#elif defined STM32F2XX
      __RW
      u32 TR;  // 0x00: Time
      __RW
      u32 DR;// 0x04: Date
      __RW
      u32 CR;// 0x08: Control
      __RW
      u32 ISR;// 0x0C: Initialization and status
      __RW
      u32 PRER;// 0x10: Prescaler
      __RW
      u32 WUTR;// 0x14: Wakeup timer
      __RW
      u32 CALIBR;// 0x18: Calibration
      __RW
      u32 ALRMR[2];// 0x1C, 0x20: Alarm
      __RW
      u32 WPR;// 0x24: Write protection
      u32 _RESERVED0[2];
      __RW
      u32 TSTR;// 0x30: Time stamp time
      __RW
      u32 TSDR;// 0x34: Time stamp date
      u32 _RESERVED1[2];
      __RW
      u32 TAFCR;// 0x40: Tamper and alternate function configuration
      u32 _RESERVED2[3];
      __RW
      u32 BKPR[20];// 0x50-0x9C: Backup
#else
      __RW
      u32 TR;  // 0x00: Time
      __RW
      u32 DR;  // 0x04: Date
      __RW
      u32 CR;  // 0x08: Control
      __RW
      u32 ISR;  // 0x0C: Initialization and status
      __RW
      u32 PRER;  // 0x10: Prescaler
      __RW
      u32 WUTR;  // 0x14: Wakeup timer
      __RW
      u32 CALIBR;  // 0x18: Calibration
      __RW
      u32 ALRMR[2];  // 0x1C, 0x20: Alarm
      __RW
      u32 WPR;  // 0x24: Write protection
      __RW
      u32 SSR;  // 0x28: Sub second
      __RW
      u32 SHIFTR;  // 0x2C: Shift control
      __RW
      u32 TSTR;  // 0x30: Time stamp time
      __RW
      u32 TSDR;  // 0x34: Time stamp date
      __RW
      u32 TSSSR;  // 0x38: Time stamp sub second
      __RW
      u32 CALR;  // 0x3C: Calibration
      __RW
      u32 TAFCR;  // 0x40: Tamper and alternate function configuration
      __RW
      u32 ALRMSSR[2];  // 0x44, 0x48: Alarm sub second
      u32 RESERVED0;
      __RW
      u32 BKPR[20];  // 0x50-0x9C: Backup
#endif
  };

// TODO RTC register bits
}// namespace rtc
