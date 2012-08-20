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

namespace spi {
  enum Address {
#ifdef STM32F1XX
    SPI1 = alias::APB2 + 0x3000,
    SPI2 = alias::APB1 + 0x3800,
    SPI3 = alias::APB1 + 0x3C00
#else
    SPI1 = alias::APB2 + 0x3000,
    SPI2 = alias::APB1 + 0x3800,
    SPI3 = alias::APB1 + 0x3C00
#endif
  };

  struct Registers {
      __RW
      u32 CR1;  // 0x00: Control 1
      __RW
      u32 CR2;  // 0x04: Control 2
      __RW
      u32 SR;  // 0x08: Status
      __RW
      u32 DR;  // 0x0C: Data
      __RW
      u32 CRCPR;  // 0x10: CRC polynomial
      __RW
      u32 RXCRCR;  // 0x14: RX CRC
      __RW
      u32 TXCRCR;  // 0x18: TX CRC
      __RW
      u32 I2SCFGR;  // 0x1C: I2S configuration
      __RW
      u32 I2SPR;  // 0x20: I2S prescaler
  };

  namespace cr1 {
    enum {
      OFFSET = 0x00
    };

    namespace cpha {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        FIRST_CLOCK_TRANSITION_IS_FIRST_DATA_CAPTURED_EDGE = 0 << POSITION,
        SECOND_CLOCK_TRANSITION_IS_FIRST_DATA_CAPTURED_EDGE = 1 << POSITION
      };
    }  // namespace cpha

    namespace cpol {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        CK_TO_0_WHEN_IDLE = 0 << POSITION,
        CK_TO_1_WHEN_IDLE = 1 << POSITION
      };
    }  // namespace cpol

    namespace msrt {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        SLAVE_CONFIGURATION = 0 << POSITION,
        MASTER_CONFIGURATION = 1 << POSITION
      };
    }  // namespace msrt

    namespace br {
      enum {
        POSITION = 3,
        MASK = 0b111 << POSITION
      };
      enum States {
        BAUD_RATE_CONTROL_DIV_2 = 0 << POSITION,
        BAUD_RATE_CONTROL_DIV_4 = 1 << POSITION,
        BAUD_RATE_CONTROL_DIV_8 = 2 << POSITION,
        BAUD_RATE_CONTROL_DIV_16 = 3 << POSITION,
        BAUD_RATE_CONTROL_DIV_32 = 4 << POSITION,
        BAUD_RATE_CONTROL_DIV_64 = 5 << POSITION,
        BAUD_RATE_CONTROL_DIV_128 = 6 << POSITION,
        BAUD_RATE_CONTROL_DIV_256 = 7 << POSITION
      };
    }  // namespace br

    namespace spe {
      enum {
        POSITION = 6,
        MASK = 1 << POSITION
      };
      enum States {
        PERIPHERAL_DISABLED = 0 << POSITION,
        PERIPHERAL_ENABLED = 1 << POSITION
      };
    }  // namespace spe

    namespace lsbfirst {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        MSB_TRANSMITTED_FIRST = 0 << POSITION,
        LSB_TRANSMITTED_FIRST = 1 << POSITION
      };
    }  // namespace lsbfirst

    namespace ssi {
      enum {
        POSITION = 8,
        MASK = 1 << POSITION
      };
    }  // namespace ssi

    namespace ssm {
      enum {
        POSITION = 9,
        MASK = 1 << POSITION
      };
      enum States {
        SOFTWARE_SLAVE_MANAGEMENT_DISABLED = 0 << POSITION,
        SOFTWARE_SLAVE_MANAGEMENT_ENABLED = 1 << POSITION
      };
    }  // namespace ssm

    namespace rxonly {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
      enum States {
        FULL_DUPLEX = 0 << POSITION,
        OUTPUT_DISABLE = 1 << POSITION
      };
    }  // namespace rxonly

    namespace dff {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
      enum States {
        DATA_FRAME_FORMAT_8_BIT = 0 << POSITION,
        DATA_FRAME_FORMAT_16_BIT = 1 << POSITION
      };
    }  // namespace dff

    namespace crcnext {
      enum {
        POSITION = 12,
        MASK = 1 << POSITION
      };
      enum States {
        NO_CRC_PHASE = 0 << POSITION,
        CRC_PHASE = 1 << POSITION
      };
    }  // namespace crcnext

    namespace crcen {
      enum {
        POSITION = 13,
        MASK = 1 << POSITION
      };
      enum States {
        CRC_CALCULATION_DISABLED = 0 << POSITION,
        CRC_CALCULATION_ENABLED = 1 << POSITION
      };
    }  // namespace crcen

    namespace bidioe {
      enum {
        POSITION = 14,
        MASK = 1 << POSITION
      };
      enum States {
        OUTPUT_DISABLED = 0 << POSITION,
        OUTPUT_ENABLED = 1 << POSITION
      };
    }  // namespace bidioe

    namespace bidimode {
      enum {
        POSITION = 15,
        MASK = 1 << POSITION
      };
      enum States {
        DATA_MODE_2LINE_UNIDIRECTIONAL = 0 << POSITION,
        DATA_MODE_1LINE_BIDIRECTIONAL = 1 << POSITION
      };
    }  // namespace bidimode
  }  // namespace cr1

  namespace cr2 {
    enum {
      OFFSET = 0x04
    };
    namespace rxdmaen {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        RX_BUFFER_DMA_DISABLED = 0 << POSITION,
        RX_BUFFER_DMA_ENABLED = 1 << POSITION
      };
    }  // namespace rxdmaen

    namespace txdmaen {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        TX_BUFFER_DMA_DISABLED = 0 << POSITION,
        TX_BUFFER_DMA_ENABLED = 1 << POSITION
      };
    }  // namespace txdmaen

    namespace ssoe {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        SS_OUTPUT_DISABLED_MASTER_MODE = 0 << POSITION,
        SS_OUTPUT_ENABLED_MASTER_MODE = 1 << POSITION
      };
    }  // namespace ssoe

    namespace frf {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
      enum States {
        SPI_MOTOROLA_MODE = 0 << POSITION,
        SPI_TI_MODE = 1 << POSITION
      };
    }  // namespace frf

    namespace errie {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
      enum States {
        ERROR_INTERRUPT_DISABLED = 0 << POSITION,
        ERROR_INTERRUPT_ENABLED = 1 << POSITION
      };
    }  // namespace errie

    namespace rxneie {
      enum {
        POSITION = 6,
        MASK = 1 << POSITION
      };
      enum States {
        RXNE_INTERRUPT_DISABLED = 0 << POSITION,
        RXNE_INTERRUPT_ENABLED = 1 << POSITION
      };
    }  // namespace rxneie

    namespace txeie {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        TXE_INTERRUPT_DISABLED = 0 << POSITION,
        TXE_INTERRUPT_ENABLED = 1 << POSITION
      };
    }  // namespace txeie
  }  // namespace cr2

  namespace sr {
    enum {
      OFFSET = 0x08
    };

    namespace rxne {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        RX_BUFFER_EMPTY = 0 << POSITION,
        RX_BUFFER_NOT_EMPTY = 1 << POSITION
      };
    }  // namespace rxne

    namespace txe {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        TX_BUFFER_NOT_EMPTY = 0 << POSITION,
        TX_BUFFER_EMPTY = 1 << POSITION
      };
    }  // namespace txe

    namespace chside {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        CHANNEL_LEFT_TO_BE_TRANSMITED_OR_HAS_BEEN_RECEIVED = 0 << POSITION,
        CHANNEL_RIGHT_TO_BE_TRANSMITED_OR_HAS_BEEN_RECEIVED = 1 << POSITION
      };
    }  // namespace chside

    namespace udr {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        NO_UNDERRUN_OCURRED = 0 << POSITION,
        UNDERRUN_OCURRED = 1 << POSITION
      };
    }  // namespace udr

    namespace crcerr {
      enum {
        POSITION = 4,
        MASK = 1 << POSITION
      };
      enum States {
        CRC_RECEIVED_MATCHES_SPI_RXCRCR_VALUE = 0 << POSITION,
        CRC_RECEIVED_DOESNT_MATCH_SPI_RXCRCR_VALUE = 1 << POSITION
      };
    }  // namespace crcerr

    namespace modf {
      enum {
        POSITION = 5,
        MASK = 1 << POSITION
      };
      enum States {
        NO_MODE_FAULT_OCURRED = 0 << POSITION,
        MODE_FAULT_OCURRED = 1 << POSITION
      };
    }  // namespace modf

    namespace ovr {
      enum {
        POSITION = 6,
        MASK = 1 << POSITION
      };
      enum States {
        NO_OVERRUN_OCURRED = 0 << POSITION,
        OVERRUN_OCURRED = 1 << POSITION
      };
    }  // namespace ovr

    namespace bsy {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        SPI_OR_I2S_NOT_BUSY = 0 << POSITION,
        SPI_OR_I2S_BUSY = 1 << POSITION
      };
    }  // namespace bsy

    namespace tifrfe {
      enum {
        POSITION = 8,
        MASK = 1 << POSITION
      };
      enum States {
        NO_FRAME_FORMAT_ERROR = 0 << POSITION,
        FRAME_FORMAT_ERROR_OCURRED = 1 << POSITION
      };
    }  // namespace tifrfe
  }  // namespace sr

  namespace dr {
    enum {
      OFFSET = 0x0C
    };
  }  // namespace dr

  namespace crcpr {
    enum {
      OFFSET = 0x10
    };
  }  // namespace crcpr

  namespace rxcrcr {
    enum {
      OFFSET = 0x14
    };
  }  // namespace rxcrcr

  namespace txcrcr {
    enum {
      OFFSET = 0x18
    };
  }  // namespace txcrcr

  namespace i2scfgr {
    enum {
      OFFSET = 0x1C
    };

    namespace chlen {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        CHANNEL_LENGTH_16BIT = 0 << POSITION,
        CHANNEL_LENGTH_32BIT = 1 << POSITION
      };
    }  // namespace chlen

    namespace datlen {
      enum {
        POSITION = 1,
        MASK = 0b11 << POSITION
      };
      enum States {
        DATA_LENGTH_16BIT = 0 << POSITION,
        DATA_LENGTH_24BIT = 1 << POSITION,
        DATA_LENGTH_32BIT = 2 << POSITION
      };
    }  // namespace datlen

    namespace ckpol {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        I2S_CLOCK_STEADY_STATE_LOW_LEVEL = 0 << POSITION,
        I2S_CLOCK_STEADY_STATE_HIGH_LEVEL = 1 << POSITION
      };
    }  // namespace ckpol

    namespace i2sstd {
      enum {
        POSITION = 4,
        MASK = 0b11 << POSITION
      };
      enum States {
        I2S_PHILIP_STANDARD = 0 << POSITION,
        MSB_JUSTIFIED_STANDARD = 1 << POSITION,
        LSB_JUSTIFIED_STANDARD = 2 << POSITION,
        PCM_STANDARD = 3 << POSITION
      };
    }  // namespace i2sstd

    namespace pcmsync {
      enum {
        POSITION = 7,
        MASK = 1 << POSITION
      };
      enum States {
        SHORT_FRAME_SYNCHRONIZATION = 0 << POSITION,
        LONG_FRAME_SYNCHRONIZATION = 1 << POSITION
      };
    }  // namespace pcmsync

    namespace i2scfg {
      enum {
        POSITION = 8,
        MASK = 0b11 << POSITION
      };
      enum States {
        SLAVE_TRANSMIT = 0 << POSITION,
        SLAVE_RECEIVE = 1 << POSITION,
        MASTER_TRANSMIT = 2 << POSITION,
        MASTER_RECEIVE = 3 << POSITION
      };
    }  // namespace i2scfg

    namespace i2se {
      enum {
        POSITION = 10,
        MASK = 1 << POSITION
      };
      enum States {
        I2S_DISABLED = 0 << POSITION,
        I2S_ENABLED = 1 << POSITION
      };
    }  // namespace i2se

    namespace i2smod {
      enum {
        POSITION = 11,
        MASK = 1 << POSITION
      };
      enum States {
        SPI_MODE_SELECTED = 0 << POSITION,
        I2S_MODE_SELECTED = 1 << POSITION
      };
    }  // namespace i2smod
  }  // namespace i2scfgr

  namespace i2spr {
    enum {
      OFFSET = 0x20
    };

    namespace odd {
      enum {
        POSITION = 8,
        MASK = 1 << POSITION
      };
      enum States {
        REAL_DIVIDER_VALUE_I2SDIVx2 = 0 << POSITION,
        REAL_DIVIDER_VALUE_I2SDIVx2_PLUS_1 = 1 << POSITION
      };
    }  // namespace odd

    namespace mckoe {
      enum {
        POSITION = 9,
        MASK = 1 << POSITION
      };
      enum States {
        MASTER_CLOCK_OUTPUT_DISABLED = 0 << POSITION,
        MASTER_CLOCK_OUTPUT_ENABLED = 1 << POSITION
      };
    }  // namespace mckoe
  }  // namespace i2spr
}  // namespace spi
