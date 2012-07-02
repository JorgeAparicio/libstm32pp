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

namespace i2c {
  struct Registers {
      __RW
      u32 CR1;    // 0x00: Control 1
      __RW
      u32 CR2;    // 0x04: Control 2
      __RW
      u32 OAR1;   // 0x08: Own address 1
      __RW
      u32 OAR2;   // 0x0C: Own address 2
      __RW
      u32 DR;     // 0x10: Data
      __RW
      u32 SR1;    // 0x14: Status 1
      __RW
      u32 SR2;    // 0x18: Status 2
      __RW
      u32 CCR;    // 0x1C: Clock control
      __RW
      u32 TRISE;  // 0x20: Rise Time
  };

  namespace address {
    enum E {
      I2C1 = alias::address::APB1 + 0x5400,
      I2C2 = alias::address::APB1 + 0x5800,

#ifndef STM32F1XX
    I2C3 = alias::address::APB1 + 0x5C00
#endif // !STM32F1XX
    };
  }  // namespace address

  namespace registers {
    namespace cr1 {
      enum {
        OFFSET = 0x00
      };
      namespace bits {
        namespace pe {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              PERIPHERAL_DISABLED = 0 << POSITION,
              PERIPHERAL_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace pe

        namespace smbus {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              I2C_MODE = 0 << POSITION,
              SMBUS_MODE = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace smbus

        namespace smbtype {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SMBUS_DEVICE = 0 << POSITION,
              SMBUS_HOST = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace smbtype

        namespace enarp {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              ADDRESS_RESOLUTION_PROTOCOL_DISABLED = 0 << POSITION,
              ADDRESS_RESOLUTION_PROTOCOL_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace enarp

        namespace enpec {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              PACKET_ERROR_CHECKING_DISABLED = 0 << POSITION,
              PACKET_ERROR_CHECKING_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace enpec

        namespace engc {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              GENERAL_CALL_DISABLED = 0 << POSITION,
              GENERAL_CALL_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace engc

        namespace nostretch {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              CLOCK_STRETCHING_DISABLED = 0 << POSITION,
              CLOCK_STRETCHING_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace nostretch

        namespace start {
          enum {
            POSITION = 8
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DONT_GENERATE_START = 0 << POSITION,
              GENERATE_START = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace start

        namespace stop {
          enum {
            POSITION = 9
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DONT_GENERATE_STOP = 0 << POSITION,
              GENERATE_STOP = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace stop

        namespace ack {
          enum {
            POSITION = 10
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              ACKNOWLEDGE_ENABLED = 0 << POSITION,
              ACKNOWLEDGE_DISABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ack

        namespace pos {
          enum {
            POSITION = 11
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              PEC_CURRENT_POSITION = 0 << POSITION,
              PEC_NEXT_POSITION = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace pos

        namespace pec {
          enum {
            POSITION = 12
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_PEC_TRANSFER = 0 << POSITION,
              PEC_TRANSFER = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace pec

        namespace alert {
          enum {
            POSITION = 13
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SMBA_HIGH = 0 << POSITION,
              SMBA_LOW = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace alert

        namespace swrst {
          enum {
            POSITION = 15
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              I2C_NOT_UNDER_RESET = 0 << POSITION,
              I2C_UNDER_RESET = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace swrst
      }  // namespace bits
    }  // namespace cr1

    namespace cr2 {
      enum {
        OFFSET = 0x04
      };
      namespace bits {
        namespace freq {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 0b111111 << POSITION
          };
        }  // namespace freq

        namespace iterren {
          enum {
            POSITION = 8
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              ERROR_INTERRUPT_DISABLED = 0 << POSITION,
              ERROR_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace iterren

        namespace itevten {
          enum {
            POSITION = 9
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              EVENT_INTERRUPT_DISABLED = 0 << POSITION,
              EVENT_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace itevten

        namespace itbufen {
          enum {
            POSITION = 10
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              BUFFER_INTERRUPT_DISABLED = 0 << POSITION,
              BUFFER_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace itbufen

        namespace dmaen {
          enum {
            POSITION = 11
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DMA_REQUEST_DISABLED = 0 << POSITION,
              DMA_REQUEST_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace dmaen

        namespace last {
          enum {
            POSITION = 12
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NEXT_DMA_IS_NOT_THE_LAST_TRANSFER = 0 << POSITION,
              NEXT_DMA_IS_THE_LAST_TRANSFER = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace last
      }  // namespace bits
    }  // namespace cr2

    namespace oar1 {
      enum {
        OFFSET = 0x08
      };
    }  // namespace oar1

    namespace oar2 {
      enum {
        OFFSET = 0x0C
      };
    }  // namespace oar2

    namespace dr {
      enum {
        OFFSET = 0x10
      };
    }  // namespace dr

    namespace sr1 {
      enum {
        OFFSET = 0x14
      };
      namespace bits {
        namespace sb {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_START_CONDITION = 0 << POSITION,
              START_CONDITION_GENERATED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace sb
        namespace addr {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_END_OF_ADDRESS_TRANSMISSION = 0 << POSITION,
              END_OF_ADDRESS_TRANSMISSION = 1 << POSITION,
              ADDRESS_MISMATCHED_OR_NOT_RECEIVED = 0 << POSITION,
              RECEIVED_ADDRESS_MATCHED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace addr

        namespace btf {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DATA_BYTE_TRANSFER_NOT_DONE = 0 << POSITION,
              DATA_BYTE_TRANSFER_SUCCEEDED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace btf

        namespace add10 {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_ADD10_EVENT_OCCURRED = 0 << POSITION,
              MASTER_HAS_SENT_FIRST_ADDRESS_BYTE = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace add10

        namespace stopf {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_STOP_CONDITION_DETECTED = 0 << POSITION,
              STOP_CONDITION_DETECTED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace stopf

        namespace rxne {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              RECEIVED_DATA_REGISTER_EMPTY = 0 << POSITION,
              RECEIVED_DATA_REGISTER_NOT_EMPTY = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace rxne

        namespace txe {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              TRANSMITTED_DATA_REGISTER_EMPTY = 0 << POSITION,
              TRANSMITTED_DATA_REGISTER_NOT_EMPTY = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace txe

        namespace berr {
          enum {
            POSITION = 8
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_MISPLACED_START_OR_STOP_CONDITION = 0 << POSITION,
              MISPLACED_START_OR_STOP_CONDITION = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace berr

        namespace arlo {
          enum {
            POSITION = 9
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_ARBITRATION_LOST_DETECTED = 0 << POSITION,
              ARBTRATION_LOST_DETECTED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace arlo

        namespace af {
          enum {
            POSITION = 10
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_ACKNOWLEDGE_FAILURE = 0 << POSITION,
              ACKNOWLEDGE_FAILURE = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace af

        namespace ovr {
          enum {
            POSITION = 11
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_OVERRUN_UNDERRUN = 0 << POSITION,
              OVERRUN_OR_UNDERRUN = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ovr

        namespace pecerr {
          enum {
            POSITION = 12
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_PEC_ERROR = 0 << POSITION,
              PEC_ERROR = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace pecerr

        namespace timeout {
          enum {
            POSITION = 14
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_TIMEOUT_ERROR = 0 << POSITION,
              TIMEOUT_ERROR = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace timeout

        namespace smbalert {
          enum {
            POSITION = 15
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_SMBALERT = 0 << POSITION,
              SMBALERT_EVENT_OCCURRED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace smbalert
      }  // namespace bits
    }  // namespace sr1

    namespace sr2 {
      enum {
        OFFSET = 0x18
      };
      namespace bits {
        namespace msl {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SLAVE_MODE = 0 << POSITION,
              MASTER_MODE = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace msl

        namespace busy {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_COMMUNICATION_ON_THE_BUS = 0 << POSITION,
              COMMUNICATION_ONGOING_ON_THE_BUS = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace busy

        namespace tra {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DATA_BYTES_RECEIVED = 0 << POSITION,
              DATA_BYTES_TRANSMITTED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace tra

        namespace gencall {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_GENERAL_CALL = 0 << POSITION,
              GENERAL_CALL_RECEIVED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace gencall

        namespace smbdefault {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_SMBUS_DEVICE_DEFAULT_ADDRESS = 0 << POSITION,
              SMBUS_DEVICE_DEFAULT_ADDRESS_RECEIVED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace smbdefault

        namespace smbhost {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_SMBUS_HOST_ADDRESS = 0 << POSITION,
              SMBUS_HOST_ADDRESS_RECEIVED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace smbhost

        namespace dualf {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              RECEIVED_ADDRESS_MATCHED_WITH_OAR1 = 0 << POSITION,
              RECEIVED_ADDRESS_MATCHED_WITH_OAR2 = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace dualf

        namespace pec {
          enum {
            POSITION = 8
          };
          enum {
            MASK = 0b11111111 << POSITION
          };
        }  // namespace pec
      }  // namespace bits
    }  // namespace sr2

    namespace ccr {
      enum {
        OFFSET = 0x1C
      };
      namespace bits {
        namespace ccr {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 0b11111111111 << POSITION
          };
        }  // namespace ccr

        namespace duty {
          enum {
            POSITION = 14
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              T_LOW_2_T_HIGH_1 = 0 << POSITION,
              T_LOW_16_T_HIGH_9 = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace duty

        namespace f_s {
          enum {
            POSITION = 15
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              STANDARD_MODE = 0 << POSITION,
              FAST_MODE = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace f_s
      }  // namespace bits
    }  // namespace ccr

    namespace trise {
      enum {
        OFFSET = 0x20
      };
    }  // namespace trise
  }  // namespace registers

  namespace operation {
    enum E {
      WRITE = 0,
      READ = 1,
    };
  }  // namespace operation
}  // namespace i2c
