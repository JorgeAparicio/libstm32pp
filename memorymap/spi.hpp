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

#ifdef STM32F1XX
  namespace address {
    enum E {
      SPI1 = alias::address::APB2 + 0x3000,
      SPI2 = alias::address::APB1 + 0x3800,
      SPI3 = alias::address::APB1 + 0x3C00
    };
  }  // namespace address
#else
  namespace address {
    enum E {
      SPI1 = alias::address::APB2 + 0x3000,
      SPI2 = alias::address::APB1 + 0x3800,
      SPI3 = alias::address::APB1 + 0x3C00
    };
  }  // namespace address
#endif

  namespace registers {
    namespace cr1 {
      enum {
        OFFSET = 0x00
      };

      namespace bits {
        namespace cpha {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              FIRST_CLOCK_TRANSITION_IS_FIRST_DATA_CAPTURED_EDGE = 0,
              SECOND_CLOCK_TRANSITION_IS_FIRST_DATA_CAPTURED_EDGE = 1
            };
          }  // namespace states
        }  // namespace cpha

        namespace cpol {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              CK_TO_0_WHEN_IDLE = 0,
              CK_TO_1_WHEN_IDLE = 1
            };
          }  // namespace states
        }  // namespace cpol

        namespace msrt {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SLAVE_CONFIGURATION = 0,
              MASTER_CONFIGURATION = 1
            };
          }  // namespace states
        }  // namespace msrt

        namespace br {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 0b111 << POSITION
          };
          namespace states {
            enum E {
              BAUD_RATE_CONTROL_DIV_2 = 0,
              BAUD_RATE_CONTROL_DIV_4 = 1,
              BAUD_RATE_CONTROL_DIV_8 = 2,
              BAUD_RATE_CONTROL_DIV_16 = 3,
              BAUD_RATE_CONTROL_DIV_32 = 4,
              BAUD_RATE_CONTROL_DIV_64 = 5,
              BAUD_RATE_CONTROL_DIV_128 = 6,
              BAUD_RATE_CONTROL_DIV_256 = 7,
            };
          }  // namespace states
        }  // namespace br

        namespace spe {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              PERIPHERAL_DISABLED = 0,
              PERIPHERAL_ENABLED = 1
            };
          }  // namespace states
        }  // namespace spe

        namespace lsbfirst {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              MSB_TRANSMITTED_FIRST = 0,
              LSB_TRANSMITTED_FIRST = 1
            };
          }  // namespace states
        }  // namespace lsbfirst

        namespace ssi {
          enum {
            POSITION = 8
          };
          enum {
            MASK = 1 << POSITION
          };
        }  // namespace ssi

        namespace ssm {
          enum {
            POSITION = 9
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SOFTWARE_SLAVE_MANAGEMENT_DISABLED = 0,
              SOFTWARE_SLAVE_MANAGEMENT_ENABLED = 1
            };
          }  // namespace states
        }  // namespace ssm

        namespace rxonly {
          enum {
            POSITION = 10
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              FULL_DUPLEX = 0,
              OUTPUT_DISABLE = 1
            };
          }  // namespace states
        }  // namespace rxonly

        namespace dff {
          enum {
            POSITION = 11
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DATA_FRAME_FORMAT_8_BIT = 0,
              DATA_FRAME_FORMAT_16_BIT = 1
            };
          }  // namespace states
        }  // namespace dff

        namespace crcnext {
          enum {
            POSITION = 12
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_CRC_PHASE = 0,
              CRC_PHASE = 1
            };
          }  // namespace states
        }  // namespace crcnext

        namespace crcen {
          enum {
            POSITION = 13
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              CRC_CALCULATION_DISABLED = 0,
              CRC_CALCULATION_ENABLED = 1
            };
          }  // namespace states
        }  // namespace crcen

        namespace bidioe {
          enum {
            POSITION = 14
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              OUTPUT_DISABLED = 0,
              OUTPUT_ENABLED = 1
            };
          }  // namespace states
        }  // namespace bidioe

        namespace bidimode {
          enum {
            POSITION = 15
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DATA_MODE_2LINE_UNIDIRECTIONAL = 0,
              DATA_MODE_1LINE_BIDIRECTIONAL = 1
            };
          }  // namespace states
        }  // namespace bidimode
      }  // namespace bits
    }  // namespace cr1

    namespace cr2 {
      enum {
        OFFSET = 0x04
      };
      namespace bits {
        namespace rxdmaen {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              RX_BUFFER_DMA_DISABLED = 0,
              RX_BUFFER_DMA_ENABLED = 1
            };
          }  // namespace states
        }  // namespace rxdmaen

        namespace txdmaen {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              TX_BUFFER_DMA_DISABLED = 0,
              TX_BUFFER_DMA_ENABLED = 1
            };
          }  // namespace states
        }  // namespace txdmaen

        namespace ssoe {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SS_OUTPUT_DISABLED_MASTER_MODE = 0,
              SS_OUTPUT_ENABLED_MASTER_MODE = 1
            };
          }  // namespace states
        }  // namespace ssoe

        namespace frf {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SPI_MOTOROLA_MODE = 0,
              SPI_TI_MODE = 1
            };
          }  // namespace states
        }  // namespace frf

        namespace errie {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              ERROR_INTERRUPT_IS_MASKED = 0,
              ERROR_INTERRUPT_ENABLED = 1
            };
          }  // namespace states
        }  // namespace errie

        namespace rxneie {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              RXNE_INTERRUPT_MASKED = 0,
              RXNE_INTERRUPT_ENABLED = 1
            };
          }  // namespace states
        }  // namespace rxneie

        namespace txeie {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              TXE_INTERRUPT_MASKED = 0,
              TXE_INTERRUPT_ENABLED = 1
            };
          }  // namespace states
        }  // namespace txeie
      }  // namespace bits
    }  // namespace cr2

    namespace sr {
      enum {
        OFFSET = 0x08
      };

      namespace bits {
        namespace rxne {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              RX_BUFFER_EMPTY = 0,
              RX_BUFFER_NOT_EMPTY = 1
            };
          }  // namespace states
        }  // namespace rxne

        namespace txe {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              TX_BUFFER_NOT_EMPTY = 0,
              TX_BUFFER_EMPTY = 1
            };
          }  // namespace states
        }  // namespace txe

        namespace chside {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              CHANNEL_LEFT_TO_BE_TRANSMITED_OR_HAS_BEEN_RECEIVED = 0,
              CHANNEL_RIGHT_TO_BE_TRANSMITED_OR_HAS_BEEN_RECEIVED = 1
            };
          }  // namespace states
        }  // namespace chside

        namespace udr {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_UNDERRUN_OCURRED = 0,
              UNDERRUN_OCURRED = 1
            };
          }  // namespace states
        }  // namespace udr

        namespace crcerr {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              CRC_RECEIVED_MATCHES_SPI_RXCRCR_VALUE = 0,
              CRC_RECEIVED_DOESNT_MATCH_SPI_RXCRCR_VALUE = 1
            };
          }  // namespace states
        }  // namespace crcerr

        namespace modf {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_MODE_FAULT_OCURRED = 0,
              MODE_FAULT_OCURRED = 1
            };
          }  // namespace states
        }  // namespace modf

        namespace ovr {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_OVERRUN_OCURRED = 0,
              OVERRUN_OCURRED = 1
            };
          }  // namespace states
        }  // namespace ovr

        namespace bsy {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SPI_OR_I2S_NOT_BUSY = 0,
              SPI_OR_I2S_BUSY = 1
            };
          }  // namespace states
        }  // namespace bsy

        namespace tifrfe {
          enum {
            POSITION = 8
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_FRAME_FORMAT_ERROR = 0,
              FRAME_FORMAT_ERROR_OCURRED = 1
            };
          }  // namespace states
        }  // namespace tifrfe
      }  // namespace bits
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

      namespace bits {
        namespace chlen {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              CHANNEL_LENGTH_16BIT = 0,
              CHANNEL_LENGTH_32BIT = 1
            };
          }  // namespace states
        }  // namespace chlen

        namespace datlen {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 0b11 << POSITION
          };
          namespace states {
            enum E {
              DATA_LENGTH_16BIT = 0,
              DATA_LENGTH_24BIT = 1,
              DATA_LENGTH_32BIT = 2,
            };
          }  // namespace states
        }  // namespace datlen

        namespace ckpol {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              I2S_CLOCK_STEADY_STATE_LOW_LEVEL = 0,
              I2S_CLOCK_STEADY_STATE_HIGH_LEVEL = 1,
            };
          }  // namespace states
        }  // namespace ckpol

        namespace i2sstd {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 0b11 << POSITION
          };
          namespace states {
            enum E {
              I2S_PHILIP_STANDARD = 0,
              MSB_JUSTIFIED_STANDARD = 1,
              LSB_JUSTIFIED_STANDARD = 2,
              PCM_STANDARD = 3
            };
          }  // namespace states
        }  // namespace i2sstd

        namespace pcmsync {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SHORT_FRAME_SYNCHRONIZATION = 0,
              LONG_FRAME_SYNCHRONIZATION = 1,
            };
          }  // namespace states
        }  // namespace pcmsync

        namespace i2scfg {
          enum {
            POSITION = 8
          };
          enum {
            MASK = 0b11 << POSITION
          };
          namespace states {
            enum E {
              SLAVE_TRANSMIT = 0,
              SLAVE_RECEIVE = 1,
              MASTER_TRANSMIT = 2,
              MASTER_RECEIVE = 3
            };
          }  // namespace states
        }  // namespace i2scfg

        namespace i2se {
          enum {
            POSITION = 10
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              I2S_DISABLED = 0,
              I2S_ENABLED = 1,
            };
          }  // namespace states
        }  // namespace i2se

        namespace i2smod {
          enum {
            POSITION = 11
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SPI_MODE_SELECTED = 0,
              I2S_MODE_SELECTED = 1,
            };
          }  // namespace states
        }  // namespace i2smod
      }  // namespace bits
    }  // namespace i2scfgr

    namespace i2spr {
      enum {
        OFFSET = 0x20
      };

      namespace bits {
        namespace odd {
          enum {
            POSITION = 8
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              REAL_DIVIDER_VALUE_I2SDIVx2 = 0,
              REAL_DIVIDER_VALUE_I2SDIVx2_PLUS_1 = 1
            };
          }  // namespace states
        }  // namespace odd

        namespace mckoe {
          enum {
            POSITION = 9
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              MASTER_CLOCK_OUTPUT_DISABLED = 0,
              MASTER_CLOCK_OUTPUT_ENABLED = 1
            };
          }  // namespace states
        }  // namespace mckoe
      }  // namespace bits
    }  // namespace i2spr
  }  // namespace registers
}  // namespace spi
