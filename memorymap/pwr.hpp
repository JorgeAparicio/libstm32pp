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

namespace pwr {
  enum {
    ADDRESS = alias::APB1 + 0x7000
  };

  struct Registers {
      __RW
      u32 CR;  // 0x00: power control
      __RW
      u32 CSR;  // 0x04: power/status control
  };

  namespace cr {
    enum {
      OFFSET = 0x00
    };
    namespace ldps {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        VOLTAGE_REGULATOR_ON_DURING_STOP_MODE = 0 << POSITION,
        VOLTAGE_REGULATOR_IN_LOW_POWER_MODE_DURING_STOP_MODE =
        1 << POSITION
      };
    }  // namespace ldps

    namespace pdds {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        ENTER_STOP_MODE_DURING_CPU_DEEPSLEEP = 0 << POSITION,
        ENTER_STANDBY_MODE_DURING_CPU_DEEPSLEEP = 1 << POSITION
      };
    }  // namespace pdds

    namespace cwuf {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        NO_EFFECT = 0 << POSITION,
        CLEARS_THE_WAKEUP_FLAG = 1 << POSITION
      };
    }  // namespace cwuf

    namespace csbf {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        NO_EFFECT = 0 << POSITION,
        CLEARS_THE_STANDBY_FLAG = 1 << POSITION
      };
    }  // namespace csbf

    namespace pvde {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
      enum States {
        PVD_DISABLED = 0 << POSITION,
        PVD_ENABLED = 1 << POSITION
      };
    }  // namespace pvde

    namespace pls {
      enum {
        POSITION = 5,
        MASK = 0b111 << POSITION
      };
      enum States {
        VOLTAGE_THRESHOLD_2_DOT_2_V = 0 << POSITION,
        VOLTAGE_THRESHOLD_2_DOT_3_V = 1 << POSITION,
        VOLTAGE_THRESHOLD_2_DOT_4_V = 2 << POSITION,
        VOLTAGE_THRESHOLD_2_DOT_5_V = 3 << POSITION,
        VOLTAGE_THRESHOLD_2_DOT_6_V = 4 << POSITION,
        VOLTAGE_THRESHOLD_2_DOT_7_V = 5 << POSITION,
        VOLTAGE_THRESHOLD_2_DOT_8_V = 6 << POSITION,
        VOLTAGE_THRESHOLD_2_DOT_9_V = 7 << POSITION,
      };
    }  // namespace pls

    namespace dbp {
      enum {
        POSITION = 8,
        MASK = 1 << POSITION
      };
      enum States {
        ACCESS_TO_THE_RTC_AND_BACKUP_REGISTERS_DISABLED = 0 << POSITION,
        ACCESS_TO_THE_RTC_AND_BACKUP_REGISTERS_ENABLED = 1 << POSITION
      };
    }  // namespace dbp

#ifndef STM32F1XX
    namespace fpds {
      enum {
        POSITION = 9,
        MASK = 1 << POSITION
      };
      enum States {
        FLASH_MEMORY_NOT_POWERED_DOWN_DURING_SLEEP_MODE = 0 << POSITION,
        ACCESS_TO_THE_RTC_AND_BACKUP_REGISTERS_ENABLED = 1 << POSITION
      };
    }  // namespace fpds
#endif // !STM32F1XX
  }  // namespace cr
  namespace csr {
    enum {
      OFFSET = 0x00
    };
    namespace wuf {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        NO_WAKEUP_EVENT_OCCURRED = 0 << POSITION,
        A_WAKEUP_EVENT_WAS_RECEIVED = 1 << POSITION
      };
    }  // namespace wuf

    namespace sbf {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        DEVICE_HAS_NOT_BEEN_IN_STANDBY_MODE = 0 << POSITION,
        DEVICE_HAS_BEEN_IN_STANDBY_MODE = 1 << POSITION
      };
    }  // namespace sbf

    namespace pvdo {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        VDD_OR_VDDA_IS_HIGHER_THAN_THE_PVD_THRESHOLD_ = 0 << POSITION,
        VDD_OR_VDDA_IS_LOWER_THAN_THE_PVD_THRESHOLD_ = 1 << POSITION
      };
    }  // namespace pvdo
#ifndef STM32F1XX
    namespace brr {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        BACKUP_REGULATOR_NOT_READY = 0 << POSITION,
        BACKUP_REGULATOR_READY = 1 << POSITION
      };
    }  // namespace brr
#endif // !STM32F1XX
    namespace ewup {
      enum {
        POSITION = 8,
        MASK = 1 << POSITION
      };
      enum States {
        WKUP_PIN_USED_AS_GENERAL_PURPOSE = 0 << POSITION,
        WKUP_PIN_USED_AS_WAKEUP_PIN = 1 << POSITION
      };
    }  // namespace ewup
#ifndef STM32F1XX
    namespace bre {
      enum {
        POSITION = 9,
        MASK = 1 << POSITION
      };
      enum States {
        BACKUP_REGULATOR_DISABLED = 0 << POSITION,
        BACKUP_REGULATOR_ENABLED = 1 << POSITION
      };
    }  // namespace bre
#endif // !STM32F1XX
  }  // namespace cr
}  // namespace pwr
