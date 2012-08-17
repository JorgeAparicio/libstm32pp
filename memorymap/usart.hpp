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

namespace usart {
#ifdef STM32F1XX
  namespace address {
    enum E {
      USART1 = alias::APB2 + 0x3800,
      USART2 = alias::APB1 + 0x4400,
      USART3 = alias::APB1 + 0x4800,
      UART4 = alias::APB1 + 0x4C00,
      UART5 = alias::APB1 + 0x5000,
    };
  }  // namespace address
#else
  namespace address {
    enum E {
      USART1 = alias::APB2 + 0x1000,
      USART2 = alias::APB1 + 0x4400,
      USART3 = alias::APB1 + 0x4800,
      UART4 = alias::APB1 + 0x4C00,
      UART5 = alias::APB1 + 0x5000,
      USART6 = alias::APB2 + 0x1400,
    };
  }  // namespace address
#endif

  struct Registers {
      __RW
      u32 SR;         // 0x00: Status
      __RW
      u32 DR;         // 0x04: Data
      __RW
      u32 BRR;        // 0x08: Baud rate
      __RW
      u32 CR1;        // 0x0C: Control 1
      __RW
      u32 CR2;        // 0x10: Control 2
      __RW
      u32 CR3;        // 0x14: Control 3
      __RW
      u32 GTPR;       // 0x18: Guard time and prescaler
  };

  namespace registers {
    namespace sr {
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
              NO_PARITY_ERROR_DETECTED = 0 << POSITION,
              PARITY_ERROR_DETECTED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace pe

        namespace fe {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_FRAMING_ERROR_DETECTED = 0 << POSITION,
              FRAMING_ERROR_DETECTED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace fe

        namespace nf {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_NOISE_DETECTED = 0 << POSITION,
              NOISE_DETECTED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace nf

        namespace ore {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_OVERRUN_ERROR_DETECTED = 0 << POSITION,
              OVERRUN_ERROR_DETECTED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ore

        namespace idle {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_IDLE_LINE_DETECTED = 0 << POSITION,
              IDLE_LINE_DETECTED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace idle

        namespace rxne {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_DATA_RECEIVED = 0 << POSITION,
              DATA_RECEIVED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace rxne

        namespace tc {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              TRANSMISSION_NOT_COMPLETED = 0 << POSITION,
              TRANSMISSION_COMPLETED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace tc

        namespace txe {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DATA_NOT_TRANSFERED_TO_THE_SHIFT_REGISTER = 0 << POSITION,
              DATA_TRANSFERED_TO_THE_SHIFT_REGISTER = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace txe

        namespace lbd {
          enum {
            POSITION = 8
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              LIN_BREAK_NOT_DETECTED = 0 << POSITION,
              LIN_BREAK_DETECTED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace lbd

        namespace cts {
          enum {
            POSITION = 9
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_CHANGE_IN_NCTS_LINE = 0 << POSITION,
              NCTS_LINE_TOGGLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace cts
      }  // namespace bits
    }  // namespace sr

    namespace dr {
      enum {
        OFFSET = 0x04
      };
    }  // namespace dr

    namespace brr {
      enum {
        OFFSET = 0x08
      };
    }  // namespace brr

    namespace cr1 {
      enum {
        OFFSET = 0x0C
      };

      namespace bits {
        namespace sbk {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DONT_SEND_BREAK_CHARACTER = 0 << POSITION,
              SEND_BREAK_CHARACTER = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace

        namespace rwu {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              RECEIVER_IN_ACTIVE_MODE = 0 << POSITION,
              RECEIVER_IN_MUTE_MODE = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace rwu

        namespace re {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              RECEIVER_DISABLED = 0 << POSITION,
              RECEIVER_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace re

        namespace te {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              TRANSMITTER_DISABLED = 0 << POSITION,
              TRANSMITTER_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace te

        namespace idleie {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              IDLE_INTERRUPT_DISABLED = 0 << POSITION,
              IDLE_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace idleie

        namespace rxneie {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              RXNE_ORE_INTERRUPT_DISABLED = 0 << POSITION,
              RXNE_ORE_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace rxneie

        namespace tcie {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              TC_INTERRUPT_DISABLED = 0 << POSITION,
              TC_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace tcie

        namespace txeie {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              TXEIE_INTERRUPT_DISABLED = 0 << POSITION,
              TXEIE_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace txeie

        namespace peie {
          enum {
            POSITION = 8
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              PEIE_INTERRUPT_DISABLED = 0 << POSITION,
              PEIE_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace peie

        namespace ps {
          enum {
            POSITION = 9
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              EVEN_PARITY = 0 << POSITION,
              ODD_PARITY = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ps

        namespace pce {
          enum {
            POSITION = 10
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              PARITY_CONTROL_DISABLED = 0 << POSITION,
              PARITY_CONTROL_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace pce

        namespace wake {
          enum {
            POSITION = 11
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              WAKE_ON_IDLE_LINE = 0 << POSITION,
              WAKE_ON_ADDRESS_MARK = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace wake

        namespace m {
          enum {
            POSITION = 12
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              START_8_DATA_N_STOP = 0 << POSITION,
              START_9_DATA_N_STOP = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace m

        namespace ue {
          enum {
            POSITION = 13
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              USART_DISABLED = 0 << POSITION,
              USART_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ue

        namespace over8 {
          enum {
            POSITION = 15
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              OVERSAMPLING_BY_16 = 0 << POSITION,
              OVERSAMPLING_BY_8 = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace over8
      }  // namespace bits
    }  // namespace cr1

    namespace cr2 {
      enum {
        OFFSET = 0x10
      };

      namespace bits {
        namespace add {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 0b111 << POSITION
          };
        }  // namespace bits

        namespace lbdl {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              _10_BIT_BREAK_DETECTION = 0 << POSITION,
              _11_BIT_BREAK_DETECTION = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace lbdl

        namespace lbdie {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              LBD_INTERRUPT_DISABLED = 0 << POSITION,
              LBD_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace lbdie

        namespace lbcl {
          enum {
            POSITION = 8
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              LAST_DATA_BIT_CLOCK_PULSE_NOT_OUTPUT = 0 << POSITION,
              LAST_DATA_BIT_CLOCK_PULSE_OUTPUT = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace lbcl

        namespace cpha {
          enum {
            POSITION = 9
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              FIRST_CLOCK_TRANSITION_IS_THE_FIRST_CAPTURE_EDGE = 0 << POSITION,
              SECOND_CLOCK_TRANSITION_IS_THE_FIRST_CAPTURE_EDGE = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace cpha

        namespace cpol {
          enum {
            POSITION = 10
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              IDLE_CLOCK_LOW = 0 << POSITION,
              IDLE_CLOCK_HIGH = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace cpol

        namespace clken {
          enum {
            POSITION = 11
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SCLK_DISABLED = 0 << POSITION,
              SCLK_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace clken

        namespace stop {
          enum {
            POSITION = 12
          };
          enum {
            MASK = 0b11 << POSITION
          };
          namespace states {
            enum E {
              _1_STOP_BIT = 0 << POSITION,
              _0_5_STOP_BIT = 1 << POSITION,
              _2_STOP_BIT = 2 << POSITION,
              _1_5_STOP_BIT = 3 << POSITION,
            };
          }  // namespace states
        }  // namespace stop

        namespace linen {
          enum {
            POSITION = 14
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              LIN_MODE_DISABLED = 0 << POSITION,
              LIN_MODE_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace linen
      }  // namespace bits
    }  // namespace cr2

    namespace cr3 {
      enum {
        OFFSET = 0x14
      };

      namespace bits {
        namespace eie {
          enum {
            POSITION = 0
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
        }  // namespace eie

        namespace iren {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              IRDA_DISABLED = 0 << POSITION,
              IRDA_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace iren

        namespace irlp {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              IRDA_NORMAL_POWER = 0 << POSITION,
              IRDA_LOW_POWER = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace irlp

        namespace hdsel {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              FULL_DUPLEX = 0 << POSITION,
              HALF_DUPLEX = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace hdsel

        namespace nack {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NACK_FOR_PARITY_ERROR_DISABLED = 0 << POSITION,
              NACK_FOR_PARITY_ERROR_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace nack

        namespace scen {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SMART_CARD_MODE_DISABLED = 0 << POSITION,
              SMART_CARD_MODE_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace scen

        namespace dmar {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              RECEIVER_DMA_DISABLED = 0 << POSITION,
              RECEIVER_DMA_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace dmar

        namespace dmat {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              TRANSMITTER_DMA_DISABLED = 0 << POSITION,
              TRANSMITTER_DMA_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace dmat

        namespace rtse {
          enum {
            POSITION = 8
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              RTS_HARDWARE_FLOW_DISABLED = 0 << POSITION,
              RTS_HARDWARE_FLOW_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace rtse

        namespace ctse {
          enum {
            POSITION = 9
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              CTS_HARDWARE_FLOW_DISABLED = 0 << POSITION,
              CTS_HARDWARE_FLOW_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ctse

        namespace ctsie {
          enum {
            POSITION = 10
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              CTS_INTERRUPT_DISABLED = 0 << POSITION,
              CTS_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ctsie

        namespace onebit {
          enum {
            POSITION = 11
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              THREE_SAMPLE_BIT_METHOD = 0 << POSITION,
              ONE_SAMPLE_BIT_METHOD = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace onebit
      }  // namespace bits
    }  // namespace cr3

    namespace gtpr {
      enum {
        OFFSET = 0x18
      };
    }  // namespace gtpr
  }  // namespace registers
}  // namespace usart
