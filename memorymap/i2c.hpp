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
  enum Address {
    I2C1 = alias::APB1 + 0x5400,
    I2C2 = alias::APB1 + 0x5800,
    #ifndef STM32F1XX
    I2C3 = alias::APB1 + 0x5C00
#endif // !STM32F1XX
  };

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

  namespace cr1 {
    enum {
      OFFSET = 0x00
    };
    namespace pe {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        PERIPHERAL_DISABLED = 0 << POSITION,
        PERIPHERAL_ENABLED = 1 << POSITION,
      };
    }  // namespace pe

    namespace smbus {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        I2C_MODE = 0 << POSITION,
        SMBUS_MODE = 1 << POSITION,
      };
    }  // namespace smbus

    namespace smbtype {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        SMBUS_DEVICE = 0 << POSITION,
        SMBUS_HOST = 1 << POSITION,
      };
    }  // namespace smbtype

    namespace enarp {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
      enum States {
        ADDRESS_RESOLUTION_PROTOCOL_DISABLED = 0 << POSITION,
        ADDRESS_RESOLUTION_PROTOCOL_ENABLED = 1 << POSITION,
      };
    }  // namespace enarp

    namespace enpec {
      enum {
        POSITION = 5,
        MASK = 1 << POSITION
      };
      enum States {
        PACKET_ERROR_CHECKING_DISABLED = 0 << POSITION,
        PACKET_ERROR_CHECKING_ENABLED = 1 << POSITION,
      };
    }  // namespace enpec

    namespace engc {
      enum {
        POSITION = 6,
        MASK = 1 << POSITION
      };
      enum States {
        GENERAL_CALL_DISABLED = 0 << POSITION,
        GENERAL_CALL_ENABLED = 1 << POSITION,
      };
    }  // namespace engc

    namespace nostretch {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        CLOCK_STRETCHING_DISABLED = 0 << POSITION,
        CLOCK_STRETCHING_ENABLED = 1 << POSITION,
      };
    }  // namespace nostretch

    namespace start {
      enum {
        POSITION = 8,
        MASK = 1 << POSITION
      };
      enum States {
        DONT_GENERATE_START = 0 << POSITION,
        GENERATE_START = 1 << POSITION,
      };
    }  // namespace start

    namespace stop {
      enum {
        POSITION = 9,
        MASK = 1 << POSITION
      };
      enum States {
        DONT_GENERATE_STOP = 0 << POSITION,
        GENERATE_STOP = 1 << POSITION,
      };
    }  // namespace stop

    namespace ack {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
      enum States {
        ACKNOWLEDGE_ENABLED = 0 << POSITION,
        ACKNOWLEDGE_DISABLED = 1 << POSITION,
      };
    }  // namespace ack

    namespace pos {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
      enum States {
        PEC_CURRENT_POSITION = 0 << POSITION,
        PEC_NEXT_POSITION = 1 << POSITION,
      };
    }  // namespace pos

    namespace pec {
      enum {
        POSITION = 12,
        MASK = 1 << POSITION
      };
      enum States {
        NO_PEC_TRANSFER = 0 << POSITION,
        PEC_TRANSFER = 1 << POSITION,
      };
    }  // namespace pec

    namespace alert {
      enum {
        POSITION = 13,
        MASK = 1 << POSITION
      };
      enum States {
        SMBA_HIGH = 0 << POSITION,
        SMBA_LOW = 1 << POSITION,
      };
    }  // namespace alert

    namespace swrst {
      enum {
        POSITION = 15,
        MASK = 1 << POSITION
      };
      enum States {
        I2C_NOT_UNDER_RESET = 0 << POSITION,
        I2C_UNDER_RESET = 1 << POSITION,
      };
    }  // namespace swrst
  }  // namespace cr1

  namespace cr2 {
    enum {
      OFFSET = 0x04
    };
    namespace freq {
      enum {
        POSITION = 0,
        MASK = 0b111111 << POSITION
      };
    }  // namespace freq

    namespace iterren {
      enum {
        POSITION = 8,
        MASK = 1 << POSITION
      };
      enum States {
        ERROR_INTERRUPT_DISABLED = 0 << POSITION,
        ERROR_INTERRUPT_ENABLED = 1 << POSITION,
      };
    }  // namespace iterren

    namespace itevten {
      enum {
        POSITION = 9,
        MASK = 1 << POSITION
      };
      enum States {
        EVENT_INTERRUPT_DISABLED = 0 << POSITION,
        EVENT_INTERRUPT_ENABLED = 1 << POSITION,
      };
    }  // namespace itevten

    namespace itbufen {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
      enum States {
        BUFFER_INTERRUPT_DISABLED = 0 << POSITION,
        BUFFER_INTERRUPT_ENABLED = 1 << POSITION,
      };
    }  // namespace itbufen

    namespace dmaen {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
      enum States {
        DMA_REQUEST_DISABLED = 0 << POSITION,
        DMA_REQUEST_ENABLED = 1 << POSITION,
      };
    }  // namespace dmaen

    namespace last {
      enum {
        POSITION = 12,
        MASK = 1 << POSITION
      };
      enum States {
        NEXT_DMA_IS_NOT_THE_LAST_TRANSFER = 0 << POSITION,
        NEXT_DMA_IS_THE_LAST_TRANSFER = 1 << POSITION,
      };
    }  // namespace last
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
    namespace sb {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        NO_START_CONDITION = 0 << POSITION,
        START_CONDITION_GENERATED = 1 << POSITION,
      };
    }  // namespace sb
    namespace addr {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        NO_END_OF_ADDRESS_TRANSMISSION = 0 << POSITION,
        END_OF_ADDRESS_TRANSMISSION = 1 << POSITION,
        ADDRESS_MISMATCHED_OR_NOT_RECEIVED = 0 << POSITION,
        RECEIVED_ADDRESS_MATCHED = 1 << POSITION,
      };
    }  // namespace addr

    namespace btf {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        DATA_BYTE_TRANSFER_NOT_DONE = 0 << POSITION,
        DATA_BYTE_TRANSFER_SUCCEEDED = 1 << POSITION,
      };
    }  // namespace btf

    namespace add10 {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        NO_ADD10_EVENT_OCCURRED = 0 << POSITION,
        MASTER_HAS_SENT_FIRST_ADDRESS_BYTE = 1 << POSITION,
      };
    }  // namespace add10

    namespace stopf {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
      enum States {
        NO_STOP_CONDITION_DETECTED = 0 << POSITION,
        STOP_CONDITION_DETECTED = 1 << POSITION,
      };
    }  // namespace stopf

    namespace rxne {
      enum {
        POSITION = 6,
        MASK = 1 << POSITION
      };
      enum States {
        RECEIVED_DATA_REGISTER_EMPTY = 0 << POSITION,
        RECEIVED_DATA_REGISTER_NOT_EMPTY = 1 << POSITION,
      };
    }  // namespace rxne

    namespace txe {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        TRANSMITTED_DATA_REGISTER_EMPTY = 0 << POSITION,
        TRANSMITTED_DATA_REGISTER_NOT_EMPTY = 1 << POSITION,
      };
    }  // namespace txe

    namespace berr {
      enum {
        POSITION = 8,
        MASK = 1 << POSITION
      };
      enum States {
        NO_MISPLACED_START_OR_STOP_CONDITION = 0 << POSITION,
        MISPLACED_START_OR_STOP_CONDITION = 1 << POSITION,
      };
    }  // namespace berr

    namespace arlo {
      enum {
        POSITION = 9,
        MASK = 1 << POSITION
      };
      enum States {
        NO_ARBITRATION_LOST_DETECTED = 0 << POSITION,
        ARBTRATION_LOST_DETECTED = 1 << POSITION,
      };
    }  // namespace arlo

    namespace af {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
      enum States {
        NO_ACKNOWLEDGE_FAILURE = 0 << POSITION,
        ACKNOWLEDGE_FAILURE = 1 << POSITION,
      };
    }  // namespace af

    namespace ovr {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
      enum States {
        NO_OVERRUN_UNDERRUN = 0 << POSITION,
        OVERRUN_OR_UNDERRUN = 1 << POSITION,
      };
    }  // namespace ovr

    namespace pecerr {
      enum {
        POSITION = 12,
        MASK = 1 << POSITION
      };
      enum States {
        NO_PEC_ERROR = 0 << POSITION,
        PEC_ERROR = 1 << POSITION,
      };
    }  // namespace pecerr

    namespace timeout {
      enum {
        POSITION = 14,
        MASK = 1 << POSITION
      };
      enum States {
        NO_TIMEOUT_ERROR = 0 << POSITION,
        TIMEOUT_ERROR = 1 << POSITION,
      };
    }  // namespace timeout

    namespace smbalert {
      enum {
        POSITION = 15,
        MASK = 1 << POSITION
      };
      enum States {
        NO_SMBALERT = 0 << POSITION,
        SMBALERT_EVENT_OCCURRED = 1 << POSITION,
      };
    }  // namespace smbalert
  }  // namespace sr1

  namespace sr2 {
    enum {
      OFFSET = 0x18
    };
    namespace msl {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        SLAVE_MODE = 0 << POSITION,
        MASTER_MODE = 1 << POSITION,
      };
    }  // namespace msl

    namespace busy {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        NO_COMMUNICATION_ON_THE_BUS = 0 << POSITION,
        COMMUNICATION_ONGOING_ON_THE_BUS = 1 << POSITION,
      };
    }  // namespace busy

    namespace tra {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        DATA_BYTES_RECEIVED = 0 << POSITION,
        DATA_BYTES_TRANSMITTED = 1 << POSITION,
      };
    }  // namespace tra

    namespace gencall {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
      enum States {
        NO_GENERAL_CALL = 0 << POSITION,
        GENERAL_CALL_RECEIVED = 1 << POSITION,
      };
    }  // namespace gencall

    namespace smbdefault {
      enum {
        POSITION = 5,
        MASK = 1 << POSITION
      };
      enum States {
        NO_SMBUS_DEVICE_DEFAULT_ADDRESS = 0 << POSITION,
        SMBUS_DEVICE_DEFAULT_ADDRESS_RECEIVED = 1 << POSITION,
      };
    }  // namespace smbdefault

    namespace smbhost {
      enum {
        POSITION = 6,
        MASK = 1 << POSITION
      };
      enum States {
        NO_SMBUS_HOST_ADDRESS = 0 << POSITION,
        SMBUS_HOST_ADDRESS_RECEIVED = 1 << POSITION,
      };
    }  // namespace smbhost

    namespace dualf {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        RECEIVED_ADDRESS_MATCHED_WITH_OAR1 = 0 << POSITION,
        RECEIVED_ADDRESS_MATCHED_WITH_OAR2 = 1 << POSITION,
      };
    }  // namespace dualf

    namespace pec {
      enum {
        POSITION = 8,
        MASK = 0b11111111 << POSITION
      };
    }  // namespace pec
  }  // namespace sr2

  namespace ccr {
    enum {
      OFFSET = 0x1C
    };
    namespace ccr {
      enum {
        POSITION = 0,
        MASK = 0b11111111111 << POSITION
      };
    }  // namespace ccr

    namespace duty {
      enum {
        POSITION = 14,
        MASK = 1 << POSITION
      };
      enum States {
        T_LOW_2_T_HIGH_1 = 0 << POSITION,
        T_LOW_16_T_HIGH_9 = 1 << POSITION,
      };
    }  // namespace duty

    namespace f_s {
      enum {
        POSITION = 15,
        MASK = 1 << POSITION
      };
      enum States {
        STANDARD_MODE = 0 << POSITION,
        FAST_MODE = 1 << POSITION,
      };
    }  // namespace f_s
  }  // namespace ccr

  namespace trise {
    enum {
      OFFSET = 0x20
    };
  }  // namespace trise

  namespace operation {
    enum E {
      WRITE = 0,
      READ = 1,
    };
  }  // namespace operation
}  // namespace i2c
