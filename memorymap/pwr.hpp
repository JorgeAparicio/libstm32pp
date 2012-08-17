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
  struct Registers {
      __RW
      u32 CR;  // 0x00: power control
      __RW
      u32 CSR;  // 0x04: power/status control
  };

  enum {
    ADDRESS = alias::APB1 + 0x7000
  };

  namespace registers {
    namespace cr {
      enum {
        OFFSET = 0x00
      };
      namespace bits {
        namespace ldps {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              VOLTAGE_REGULATOR_ON_DURING_STOP_MODE = 0 << POSITION,
              VOLTAGE_REGULATOR_IN_LOW_POWER_MODE_DURING_STOP_MODE =
              1 << POSITION
            };
          }  // namespace states
        }  // namespace ldps

        namespace pdds {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              ENTER_STOP_MODE_DURING_CPU_DEEPSLEEP = 0 << POSITION,
              ENTER_STANDBY_MODE_DURING_CPU_DEEPSLEEP = 1 << POSITION
            };
          }  // namespace states
        }  // namespace pdds

        namespace cwuf {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_EFFECT = 0 << POSITION,
              CLEARS_THE_WAKEUP_FLAG = 1 << POSITION
            };
          }  // namespace states
        }  // namespace cwuf

        namespace csbf {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_EFFECT = 0 << POSITION,
              CLEARS_THE_STANDBY_FLAG = 1 << POSITION
            };
          }  // namespace states
        }  // namespace csbf

        namespace pvde {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              PVD_DISABLED = 0 << POSITION,
              PVD_ENABLED = 1 << POSITION
            };
          }  // namespace states
        }  // namespace pvde

        namespace pls {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 0b111 << POSITION
          };
          namespace states {
            enum E {
              VOLTAGE_THRESHOLD_2_DOT_2_V = 0 << POSITION,
              VOLTAGE_THRESHOLD_2_DOT_3_V = 1 << POSITION,
              VOLTAGE_THRESHOLD_2_DOT_4_V = 2 << POSITION,
              VOLTAGE_THRESHOLD_2_DOT_5_V = 3 << POSITION,
              VOLTAGE_THRESHOLD_2_DOT_6_V = 4 << POSITION,
              VOLTAGE_THRESHOLD_2_DOT_7_V = 5 << POSITION,
              VOLTAGE_THRESHOLD_2_DOT_8_V = 6 << POSITION,
              VOLTAGE_THRESHOLD_2_DOT_9_V = 7 << POSITION,
            };
          }  // namespace states
        }  // namespace pls

        namespace dbp {
          enum {
            POSITION = 8
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              ACCESS_TO_THE_RTC_AND_BACKUP_REGISTERS_DISABLED = 0 << POSITION,
              ACCESS_TO_THE_RTC_AND_BACKUP_REGISTERS_ENABLED = 1 << POSITION
            };
          }  // namespace states
        }  // namespace dbp

#ifndef STM32F1XX
      namespace fpds {
        enum {
          POSITION = 9
        };
        enum {
          MASK = 1 << POSITION
        };
        namespace states {
          enum E {
            FLASH_MEMORY_NOT_POWERED_DOWN_DURING_SLEEP_MODE = 0 << POSITION,
            ACCESS_TO_THE_RTC_AND_BACKUP_REGISTERS_ENABLED = 1 << POSITION
          };
        }  // namespace states
      }  // namespace fpds
#endif // !STM32F1XX
      }// namespace bits
    }  // namespace cr
    namespace csr {
      enum {
        OFFSET = 0x00
      };
      namespace bits {
        namespace wuf {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_WAKEUP_EVENT_OCCURRED = 0 << POSITION,
              A_WAKEUP_EVENT_WAS_RECEIVED = 1 << POSITION
            };
          }  // namespace states
        }  // namespace wuf

        namespace sbf {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DEVICE_HAS_NOT_BEEN_IN_STANDBY_MODE = 0 << POSITION,
              DEVICE_HAS_BEEN_IN_STANDBY_MODE = 1 << POSITION
            };
          }  // namespace states
        }  // namespace sbf

        namespace pvdo {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              VDD_OR_VDDA_IS_HIGHER_THAN_THE_PVD_THRESHOLD_ = 0 << POSITION,
              VDD_OR_VDDA_IS_LOWER_THAN_THE_PVD_THRESHOLD_ = 1 << POSITION
            };
          }  // namespace states
        }  // namespace pvdo
#ifndef STM32F1XX
        namespace brr {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              BACKUP_REGULATOR_NOT_READY = 0 << POSITION,
              BACKUP_REGULATOR_READY = 1 << POSITION
            };
          }  // namespace states
        }  // namespace brr
#endif // !STM32F1XX
        namespace ewup {
          enum {
            POSITION = 8
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              WKUP_PIN_USED_AS_GENERAL_PURPOSE = 0 << POSITION,
              WKUP_PIN_USED_AS_WAKEUP_PIN = 1 << POSITION
            };
          }  // namespace states
        }  // namespace ewup
#ifndef STM32F1XX
        namespace bre {
          enum {
            POSITION = 9
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              BACKUP_REGULATOR_DISABLED = 0 << POSITION,
              BACKUP_REGULATOR_ENABLED = 1 << POSITION
            };
          }  // namespace states
        }  // namespace bre
#endif // !STM32F1XX
      }  // namespace bits
    }  // namespace cr
  }  // namespace registers
}  // namespace pwr
