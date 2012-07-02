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

namespace dma {
  namespace common {
#ifdef STM32F1XX
    struct Registers
    {
      __RW
      u32 ISR;   // 0x00: Interrupt status
      __RW
      u32 IFCR;// 0x04: Interrupt flag clear
    };
#else // STM32F1XX
    struct Registers
    {
        __RW
        u32 ISR[2];  // 0x00, 0x04: Interrupt status
        __RW
        u32 IFCR[2];  // 0x08, 0x0C: Interrupt flag clear
    };
#endif // STM32F1XX
#ifdef STM32F1XX
    namespace address {
      enum E {
        DMA1 = alias::address::AHB + 0x0000,
        DMA2 = alias::address::AHB + 0x0400,
      };
    }  // namespace address
#else // STM32F1XX
    namespace address {
      enum E {
        DMA1 = alias::address::AHB1 + 0x6000,
        DMA2 = alias::address::AHB1 + 0x6400,
      };
    }  // namespace address
#endif // STM32F1XX
    namespace registers {
#ifdef STM32F1XX
      namespace isr {
        enum {
          OFFSET = 0x00
        };
      }  // namespace isr

      namespace ifcr {
        enum {
          OFFSET = 0x00
        };
      }  // namespace ifcr
#else // STM32F1XX
      namespace lisr {
        enum {
          OFFSET = 0x00
        };
      }  // namespace lisr

      namespace hisr {
        enum {
          OFFSET = 0x04
        };
      }  // namespace hisr

      namespace lifcr {
        enum {
          OFFSET = 0x08
        };
      }  // namespace lifcr

      namespace hifcr {
        enum {
          OFFSET = 0x0C
        };
      }  // namespace hifcr
#endif // STM32F1XX
    }  // namespace registers
  }  // namespace common

#ifdef STM32F1XX
  namespace channel {
    struct Registers {
      __RW
      u32 CCR;  // 0x00: Configuration
      __RW
      u32 CNDTR;// 0x04: Number of data
      __RW
      u32 CPAR;// 0x08: Peripheral address
      __RW
      u32 CMAR;// 0x0C: Memory address
    };

    namespace address {
      enum E {
        CHANNEL_1 = 0x08,
        CHANNEL_2 = 0x1C,
        CHANNEL_3 = 0x30,
        CHANNEL_4 = 0x44,
        CHANNEL_5 = 0x58,
        CHANNEL_6 = 0x6C,
        CHANNEL_7 = 0x80,
      };
    }  // namespace address

    namespace registers {
      namespace cr {
        enum {
          OFFSET = 0x00
        };
        namespace bits {
          namespace en {
            enum {
              POSITION = 0
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                CHANNEL_DISABLED = 0 << POSITION,
                CHANNEL_ENABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace en

          namespace tcie {
            enum {
              POSITION = 1
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                TRANSFER_COMPLETE_INTERRUPT_DISABLE = 0 << POSITION,
                TRANSFER_COMPLETE_INTERRUPT_ENABLE = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace tcie

          namespace htie {
            enum {
              POSITION = 2
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                HALF_TRANSFER_INTERRUPT_DISABLED = 0 << POSITION,
                HALF_TRANSFER_INTERRUPT_ENABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace htie

          namespace teie {
            enum {
              POSITION = 3
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                TRANSFER_ERROR_INTERRUPT_DISABLED = 0 << POSITION,
                TRANSFER_ERROR_INTERRUPT_ENABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace teie

          namespace dir {
            enum {
              POSITION = 4
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                READ_FROM_PERIPHERAL = 0 << POSITION,
                READ_FROM_MEMORY = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace dir

          namespace circ {
            enum {
              POSITION = 5
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                CIRCULAR_MODE_DISABLED = 0 << POSITION,
                CIRCULAR_MODE_ENABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace circ

          namespace pinc {
            enum {
              POSITION = 6
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                PERIPHERAL_INCREMENT_MODE_DISABLED = 0 << POSITION,
                PERIPHERAL_INCREMENT_MODE_ENABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace pinc

          namespace minc {
            enum {
              POSITION = 7
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                MEMORY_INCREMENT_MODE_DISABLED = 0 << POSITION,
                MEMORY_INCREMENT_MODE_ENABLE = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace minc

          namespace psize {
            enum {
              POSITION = 8
            };
            enum {
              MASK = 0b11 << POSITION
            };
            namespace states {
              enum E {
                PERIPHERAL_SIZE_8BITS = 0 << POSITION,
                PERIPHERAL_SIZE_16BITS = 1 << POSITION,
                PERIPHERAL_SIZE_32BITS = 2 << POSITION,
              };
            }  // namespace states
          }  // namespace psize

          namespace msize {
            enum {
              POSITION = 10
            };
            enum {
              MASK = 0b11 << POSITION
            };
            namespace states {
              enum E {
                MEMORY_SIZE_8BITS = 0 << POSITION,
                MEMORY_SIZE_16BITS = 1 << POSITION,
                MEMORY_SIZE_32BITS = 2 << POSITION,
              };
            }  // namespace states
          }  // namespace msize

          namespace pl {
            enum {
              POSITION = 12
            };
            enum {
              MASK = 0b11 << POSITION
            };
            namespace states {
              enum E {
                CHANNEL_PRIORITY_LEVEL_LOW = 0 << POSITION,
                CHANNEL_PRIORITY_LEVEL_MEDIUM = 1 << POSITION,
                CHANNEL_PRIORITY_LEVEL_HIGH = 2 << POSITION,
                CHANNEL_PRIORITY_LEVEL_VERY_HIGH = 3 << POSITION,
              };
            }  // namespace states
          }  // namespace pl

          namespace mem2mem {
            enum {
              POSITION = 14
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                MEMORY_TO_MEMORT_MODE_DISABLED = 0 << POSITION,
                MEMORY_TO_MEMORY_MODE_ENABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace mem2mem
        }  // namespace bits
      }  // namespace cr

      namespace ndtr {
        enum {
          OFFSET = 0x04
        };
      }  // namespace ndtr

      namespace par {
        enum {
          OFFSET = 0x08
        };
      }  // namespace par

      namespace mar {
        enum {
          OFFSET = 0x0C
        };
      }  // namespace mar
    }  // namespace registers
  }  // namespace channel

#else // STM32F1XX
  namespace stream {
    struct Registers {
        __RW
        u32 CR;  // 0x00: Configuration
        __RW
        u32 NDTR;  // 0x04: Number of data
        __RW
        u32 PAR;  // 0x08: Peripheral address
        __RW
        u32 M0AR;  // 0x0C: Memory 0 address
        __RW
        u32 M1AR;  // 0x10: Memory 1 address
        __RW
        u32 FCR;  // 0x14: FIFO control
    };

    namespace address {
      enum E {
        STREAM_0 = 0x10,
        STREAM_1 = 0x28,
        STREAM_2 = 0x40,
        STREAM_3 = 0x58,
        STREAM_4 = 0x70,
        STREAM_5 = 0x88,
        STREAM_6 = 0xA0,
        STREAM_7 = 0xB8,
      };
    }

    namespace registers {
      namespace cr {
        enum {
          OFFSET = 0x00
        };
        namespace bits {
          namespace en {
            enum {
              POSITION = 0
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                STREAM_DISABLED = 0 << POSITION,
                STREAM_ENABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace en

          namespace dmeie {
            enum {
              POSITION = 1
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                DIRECT_MODE_ERROR_INTERRUPT_DISABLED = 0 << POSITION,
                DIRECT_MODE_ERROR_INTERRUPT_ENABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace dmeie

          namespace teie {
            enum {
              POSITION = 2
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                TRANSFER_ERROR_INTERRUPT_DISABLED = 0 << POSITION,
                TRANSFER_ERROR_INTERRUPT_ENABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace teie

          namespace htie {
            enum {
              POSITION = 3
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                HALF_TRANSFER_INTERRUPT_DISABLED = 0 << POSITION,
                HALF_TRANSFER_INTERRUPT_ENABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace htie

          namespace tcie {
            enum {
              POSITION = 4
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                TRANSFER_COMPLETE_INTERRUPT_DISABLED = 0 << POSITION,
                TRANSFER_COMPLETE_INTERRUPT_ENABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace tcie

          namespace pfctrl {
            enum {
              POSITION = 5
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                DMA_FLOW_CONTROLLER = 0 << POSITION,
                PERIPHERAL_FLOW_CONTROLLER = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace pfctrl

          namespace dir {
            enum {
              POSITION = 6
            };
            enum {
              MASK = 0b11 << POSITION
            };
            namespace states {
              enum E {
                PERIPHERAL_TO_MEMORY = 0 << POSITION,
                MEMORY_TO_PERIPHERAL = 1 << POSITION,
                MEMORY_TO_MEMORY = 2 << POSITION,
              };
            }  // namespace states
          }  // namespace dir

          namespace circ {
            enum {
              POSITION = 8
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                CIRCULAR_MODE_DISABLED = 0 << POSITION,
                CIRCULAR_MODE_ENABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace circ

          namespace pinc {
            enum {
              POSITION = 9
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                PERIPHERAL_INCREMENT_MODE_DISABLED = 0 << POSITION,
                PERIPHERAL_INCREMENT_MODE_ENABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace pinc

          namespace minc {
            enum {
              POSITION = 10
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                MEMORY_INCREMENT_MODE_DISABLED = 0 << POSITION,
                MEMORY_INCREMENT_MODE_ENABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace minc

          namespace psize {
            enum {
              POSITION = 11
            };
            enum {
              MASK = 0b11 << POSITION
            };
            namespace states {
              enum E {
                PERIPHERAL_SIZE_8BITS = 0 << POSITION,
                PERIPHERAL_SIZE_16BITS = 1 << POSITION,
                PERIPHERAL_SIZE_32BITS = 2 << POSITION,
              };
            }  // namespace states
          }  // namespace psize

          namespace msize {
            enum {
              POSITION = 13
            };
            enum {
              MASK = 0b11 << POSITION
            };
            namespace states {
              enum E {
                MEMORY_SIZE_8BITS = 0 << POSITION,
                MEMORY_SIZE_16BITS = 1 << POSITION,
                MEMORY_SIZE_32BITS = 2 << POSITION,
              };
            }  // namespace states
          }  // namespace msize

          namespace pincos {
            enum {
              POSITION = 15
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                PERIPHERAL_INCREMENT_OFFSET_SIZE_PSIZE = 0 << POSITION,
                PERIPHERAL_INCREMENT_OFFSET_SIZE_32BITS = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace pincos

          namespace pl {
            enum {
              POSITION = 16
            };
            enum {
              MASK = 0b11 << POSITION
            };
            namespace states {
              enum E {
                PRIORITY_LEVEL_LOW = 0 << POSITION,
                PRIORITY_LEVEL_MEDIUM = 1 << POSITION,
                PRIORITY_LEVEL_HIGH = 2 << POSITION,
                PRIORITY_LEVEL_VERY_HIGH = 3 << POSITION,
              };
            }  // namespace states
          }  // namespace pl

          namespace dbm {
            enum {
              POSITION = 18
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                DOUBLE_BUFFER_MODE_DISABLED = 0 << POSITION,
                DOUBLE_BUFFER_MODE_ENABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace dbm

          namespace ct {
            enum {
              POSITION = 19
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                CURRENT_TARGET_MEMORY_0 = 0 << POSITION,
                CURRENT_TARGET_MEMORY_1 = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace ct

          namespace pburst {
            enum {
              POSITION = 21
            };
            enum {
              MASK = 0b11 << POSITION
            };
            namespace states {
              enum E {
                PERIPHERAL_BURST_TRANSFER_SINGLE = 0 << POSITION,
                PERIPHERAL_BURST_TRANSFER_4BEATS = 1 << POSITION,
                PERIPHERAL_BURST_TRANSFER_8BEATS = 2 << POSITION,
                PERIPHERAL_BURST_TRANSFER_16BEATS = 3 << POSITION,
              };
            }  // namespace states
          }  // namespace pburst

          namespace mburst {
            enum {
              POSITION = 23
            };
            enum {
              MASK = 0b11 << POSITION
            };
            namespace states {
              enum E {
                MEMORY_BURST_TRANSFER_SINGLE = 0 << POSITION,
                MEMORY_BURST_TRANSFER_4BEATS = 1 << POSITION,
                MEMORY_BURST_TRANSFER_8BEATS = 2 << POSITION,
                MEMORY_BURST_TRANSFER_16BEATS = 3 << POSITION,
              };
            }  // namespace states
          }  // namespace mburst

          namespace chsel {
            enum {
              POSITION = 25
            };
            enum {
              MASK = 0b111 << POSITION
            };
            namespace states {
              enum E {
                CHANNEL_0 = 0 << POSITION,
                CHANNEL_1 = 1 << POSITION,
                CHANNEL_2 = 2 << POSITION,
                CHANNEL_3 = 3 << POSITION,
                CHANNEL_4 = 4 << POSITION,
                CHANNEL_5 = 5 << POSITION,
                CHANNEL_6 = 6 << POSITION,
                CHANNEL_7 = 7 << POSITION,
              };
            }  // namespace states
          }  // namespace chsel
        }  // namespace bits
      }  // namespace cr

      namespace ndtr {
        enum {
          OFFSET = 0x04
        };
      }  // namespace ndtr

      namespace par {
        enum {
          OFFSET = 0x08
        };
      }  // namespace par

      namespace m0ar {
        enum {
          OFFSET = 0x0C
        };
      }  // namespace m0ar

      namespace m1ar {
        enum {
          OFFSET = 0x10
        };
      }  // namespace m1ar

      namespace fcr {
        enum {
          OFFSET = 0x14
        };
        namespace bits {
          namespace fth {
            enum {
              POSITION = 0
            };
            enum {
              MASK = 0b11 << POSITION
            };
            namespace states {
              enum E {
                FIFO_THRESHOLD_SELECTION_1_OVER_4 = 0 << POSITION,
                FIFO_THRESHOLD_SELECTION_2_OVER_4 = 1 << POSITION,
                FIFO_THRESHOLD_SELECTION_3_OVER_4 = 2 << POSITION,
                FIFO_THRESHOLD_SELECTION_FULL = 3 << POSITION,
              };
            }  // namespace states
          }  // namespace fth

          namespace dmdis {
            enum {
              POSITION = 2
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                DIRECT_MODE_ENABLED = 0 << POSITION,
                DIRECT_MODE_DISABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace dmdis

          namespace fs {
            enum {
              POSITION = 3
            };
            enum {
              MASK = 0b111 << POSITION
            };
          }  // namespace fs

          namespace feie {
            enum {
              POSITION = 7
            };
            enum {
              MASK = 1 << POSITION
            };
            namespace states {
              enum E {
                FIFO_ERROR_INTERRUPT_DISABLED = 0 << POSITION,
                FIFO_ERROR_INTERRUPT_ENABLED = 1 << POSITION,
              };
            }  // namespace states
          }  // namespace feie
        }  // namespace bits
      }  // namespace fcr
    }  // namespace registers
  }  // namespace stream
#endif // STM32F1XX
}  // namespace dma
