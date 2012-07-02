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

namespace adc {
#ifdef SMT32F1XX
  namespace address {
    enum E {
      ADC1 = alias::address::APB2 + 0x2400,
      ADC2 = alias::address::APB2 + 0x2800,
      ADC3 = alias::address::APB2 + 0x3C00,
    };
  }  // namespace address
#else
  namespace address {
    enum E {
      ADC1 = alias::address::APB2 + 0x2000,
      ADC2 = alias::address::APB2 + 0x2100,
      ADC3 = alias::address::APB2 + 0x2200,
      ADC = alias::address::APB2 + 0x2300,
    };
  }  // namespace address
#endif
  struct Registers {
      __RW
      u32 SR;       // 0x00: Status
      __RW
      u32 CR1;      // 0x04: Control 1
      __RW
      u32 CR2;      // 0x08: Control 2
      __RW
      u32 SMPR[2];  // 0x0C: Sample time
      __RW
      u32 JOFR[4];  // 0x14: Injected channel data offset
      __RW
      u32 HTR;      // 0x24: Watchdog higher threshold
      __RW
      u32 LTR;      // 0x28: Watchdog lower threshold
      __RW
      u32 SQR[3];   // 0x2C: Regular sequence
      __RW
      u32 JSQR;     // 0x38: Injected sequence
      __RW
      u32 JDR[4];   // 0x3C: Injected data 1
      __RW
      u32 DR;       // 0x4C: Data
  };

#if defined STM32F2XX || \
    defined STM32F4XX
  struct CommonRegisters {
    __RW
    u32 CSR;  //0x00: Common status
    __RW
    u32 CCR;  //0x04: Common control
    __RW
    u32 CDR;  //0x08: Common regular data
  };
#endif

  namespace registers {
    namespace sr {
      enum {
        OFFSET = 0x00
      };
      namespace bits {
        namespace awd {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_ANALOG_WATCHDOG_EVENT_OCURRED = 0 << POSITION,
              ANALOG_WATCHDOG_EVENT_OCURRED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace awd
        namespace eoc {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              CONVERSION_IS_NOT_COMPLETE = 0 << POSITION,
              COMVERSION_COMPLETED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace eoc
        namespace jeoc {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              ALL_INJECTED_CONVERSIONS_ARE_NOT_COMPLETE = 0 << POSITION,
              ALL_INJECTED_CONVERSIONS_COMPLETED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace jeoc
        namespace jstrt {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_INJECTED_GROUP_CONVERSION_STARTED = 0 << POSITION,
              INJECTED_GROUP_CONVERSION_HAS_STARTED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace jsrt
        namespace strt {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_REGULAR_CHANNEL_CONVERSION_STARTED = 0 << POSITION,
              REGULAR_CHANNEL_CONVERSION_HAS_STARTED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace strt
#ifndef STM32F1XX
        namespace ovr {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_OVERRUN_OCURRED = 0 << POSITION,
              OVERRUN_OCURRED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ovr
#endif
      }  // namespace bits
    }  // namespace sr

    namespace cr1 {
      enum {
        OFFSET = 0x04
      };
      namespace bits {
        namespace awdch {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 0b11111 << POSITION
          };
          namespace states {
            enum E {
              SET_ANALOG_WATCHDOG_ON_CHANNEL0 = 0 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL1 = 1 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL2 = 2 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL3 = 3 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL4 = 4 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL5 = 5 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL6 = 6 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL7 = 7 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL8 = 8 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL9 = 9 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL10 = 10 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL11 = 11 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL12 = 12 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL13 = 13 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL14 = 14 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL15 = 15 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL16 = 16 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL17 = 17 << POSITION,
              SET_ANALOG_WATCHDOG_ON_CHANNEL18 = 18 << POSITION,
            };
          }  // namespace states
        }  // namespace awdch

        namespace eocie {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              END_OF_CONVERSION_INTERRUPT_DISABLED = 0 << POSITION,
              END_OF_CONVERSION_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace eocie

        namespace awdie {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              ANALOG_WATCHDOG_INTERRUPT_DISABLED = 0 << POSITION,
              ANALOG_WATCHDOG_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace awdie

        namespace jeocie {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              END_OF_ALL_INJECTED_CONVERSIONS_INTERRUPT_DISABLED =
              0 << POSITION,
              END_OF_ALL_INJECTED_CONVERSIONS_INTERRUPT_ENABLED =
              1 << POSITION,
            };
          }  // namespace states
        }  // namespace jeocie
        namespace scan {
          enum {
            POSITION = 8
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SCAN_MODE_DISABLED = 0 << POSITION,
              SCAN_MODE_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace scan

        namespace awdsgl {
          enum {
            POSITION = 9
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              ANALOG_WATCHDOG_ENABLED_ON_ALL_CHANNELS = 0 << POSITION,
              ANALOG_WATCHDOG_ENABLED_ON_A_SINGLE_CHANNEL = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace awdsgl

        namespace jauto {
          enum {
            POSITION = 10
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              AUTOMATIC_INJECTED_CONVERSION_DISABLED = 0 << POSITION,
              AUTOMATIC_INJECTED_CONVERSION_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace jauto

        namespace discen {
          enum {
            POSITION = 11
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DISCONTINUOUS_MODE_ON_REGULAR_CHANNELS_DISABLED = 0 << POSITION,
              DISCONTINUOUS_MODE_ON_REGULAR_CHANNELS_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace discen
        namespace jdiscen {
          enum {
            POSITION = 12
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DISCONTINUOUS_MODE_ON_INJECTED_CHANNELS_DISABLED = 0 << POSITION,
              DISCONTINUOUS_MODE_ON_INJECTED_CHANNELS_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace discen

        namespace discnum {
          enum {
            POSITION = 13
          };
          enum {
            MASK = 0b111 << POSITION
          };
          namespace states {
            enum E {
              _1_CHANNEL_FOR_DISCONTINUOUS_MODE = 0 << POSITION,
              _2_CHANNEL_FOR_DISCONTINUOUS_MODE = 1 << POSITION,
              _3_CHANNEL_FOR_DISCONTINUOUS_MODE = 2 << POSITION,
              _4_CHANNEL_FOR_DISCONTINUOUS_MODE = 3 << POSITION,
              _5_CHANNEL_FOR_DISCONTINUOUS_MODE = 4 << POSITION,
              _6_CHANNEL_FOR_DISCONTINUOUS_MODE = 5 << POSITION,
              _7_CHANNEL_FOR_DISCONTINUOUS_MODE = 6 << POSITION,
              _8_CHANNEL_FOR_DISCONTINUOUS_MODE = 7 << POSITION,
            };
          }  // namespace states
        }  // namespace discnum

        namespace jawden {
          enum {
            POSITION = 22
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              ANALOG_WATCHDOG_DISABLED_ON_INJECTED_CHANNELS = 0 << POSITION,
              ANALOG_WATCHDOG_ENABLED_ON_INJECTED_CHANNELS = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace jawden
        namespace awden {
          enum {
            POSITION = 23
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              ANALOG_WATCHDOG_DISABLED_ON_REGULAR_CHANNELS = 0 << POSITION,
              ANALOG_WATCHDOG_ENABLED_ON_REGULAR_CHANNELS = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace awden
        namespace res {
          enum {
            POSITION = 24
          };
          enum {
            MASK = 0b11 << POSITION
          };
          namespace states {
            enum E {
              _12_BITS_RESOLUTION = 0 << POSITION,
              _10_BITS_RESOLUTION = 1 << POSITION,
              _8_BITS_RESOLUTION = 2 << POSITION,
              _6_BITS_RESOLUTION = 3 << POSITION,
            };
          }  // namespace states
        }  // namespace res
        namespace ovrie {
          enum {
            POSITION = 26
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              OVERRUN_INTERRUPT_DISABLED = 0 << POSITION,
              ENABLE_OVERRUN_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ovrie

      }  // namespace bits
    }  // namespace cr1

    namespace cr2 {
      enum {
        OFFSET = 0x08
      };
      namespace bits {
        namespace adon {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              ADC_POWERED_DOWN = 0 << POSITION,
              ADC_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace adon
        namespace cont {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SINGLE_CONVERSION_MODE = 0 << POSITION,
              CONTINUOUS_CONVERSION_MODE = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace cont
        namespace dma {
          enum {
            POSITION = 8
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DMA_MODE_DISABLED = 0 << POSITION,
              DMA_MODE_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace dma
        namespace dds {
          enum {
            POSITION = 9
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_NEW_DMA_REQUEST_IS_ISSUED_AFTER_THE_LAST_TRANSFER =
              0 << POSITION,
              DMA_REQUEST_ARE_ISSUED_AS_LONG_AS_DATA_IS_CONVERTED =
              1 << POSITION,
            };
          }  // namespace states
        }  // namespace dds

        namespace eocs {
          enum {
            POSITION = 10
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              EOC_BIT_IS_SET_AFTER_A_SEQUENCE_OF_REGULAR_CONVERSIONS
              = 0 << POSITION,
              EOC_BIT_IS_SET_AFTER_EACH_REGULAR_CONVERSION = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace eocs

        namespace align {
          enum {
            POSITION = 11
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              RIGTH_ALIGNED_DATA = 0 << POSITION,
              LEFT_ALIGNED_DATA = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace align

        namespace jextsel {
          enum {
            POSITION = 16
          };
          enum {
            MASK = 0b1111 << POSITION
          };
          namespace states {
            enum E {
              INJECTED_GROUP_TRIGGERED_BY_TIMER1_CC4 = 0 << POSITION,
              INJECTED_GROUP_TRIGGERED_BY_TIMER1_TRGO = 1 << POSITION,
              INJECTED_GROUP_TRIGGERED_BY_TIMER2_CC1 = 2 << POSITION,
              INJECTED_GROUP_TRIGGERED_BY_TIMER2_TRGO = 3 << POSITION,
              INJECTED_GROUP_TRIGGERED_BY_TIMER3_CC2 = 4 << POSITION,
              INJECTED_GROUP_TRIGGERED_BY_TIMER3_CC4 = 5 << POSITION,
              INJECTED_GROUP_TRIGGERED_BY_TIMER4_CC1 = 6 << POSITION,
              INJECTED_GROUP_TRIGGERED_BY_TIMER4_CC2 = 7 << POSITION,
              INJECTED_GROUP_TRIGGERED_BY_TIMER4_CC3 = 8 << POSITION,
              INJECTED_GROUP_TRIGGERED_BY_TIMER4_TRGO = 9 << POSITION,
              INJECTED_GROUP_TRIGGERED_BY_TIMER5_CC4 = 10 << POSITION,
              INJECTED_GROUP_TRIGGERED_BY_TIMER5_TRGO = 11 << POSITION,
              INJECTED_GROUP_TRIGGERED_BY_TIMER8_CC2 = 12 << POSITION,
              INJECTED_GROUP_TRIGGERED_BY_TIMER8_CC3 = 13 << POSITION,
              INJECTED_GROUP_TRIGGERED_BY_TIMER8_CC4 = 14 << POSITION,
              INJECTED_GROUP_TRIGGERED_BY_EXTI15 = 15 << POSITION,

            };
          }  // namespace states
        }  // namespace ajextel

        namespace jexten {
          enum {
            POSITION = 20
          };
          enum {
            MASK = 0b11 << POSITION
          };
          namespace states {
            enum E {
              INJECTED_TRIGGER_DISABLED = 0 << POSITION,
              INJECTED_TRIGGERED_ON_RISING_EDGE = 1 << POSITION,
              INJECTED_TRIGGERED_DETECTION_ON_FALLING_EDGE = 2 << POSITION,
              INJECTED_TRIGGERED_ON_RISING_AND_FALLING_EDGES = 3 << POSITION,
            };
          }  // namespace states
        }  // namespace jexten
        namespace jswstart {
          enum {
            POSITION = 22
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              INJECTED_CHANNELS_ON_RESET_STATE = 0 << POSITION,
              START_CONVERSION_ON_INJECTED_CHANNELS = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace jswstart

        namespace extsel {
          enum {
            POSITION = 24
          };
          enum {
            MASK = 0b1111 << POSITION
          };
          namespace states {
            enum E {
              REGULAR_GROUP_TRIGGERED_BY_TIMER1_CC1 = 0 << POSITION,
              REGULAR_GROUP_TRIGGERED_BY_TIMER1_CC2 = 1 << POSITION,
              REGULAR_GROUP_TRIGGERED_BY_TIMER1_CC3 = 2 << POSITION,
              REGULAR_GROUP_TRIGGERED_BY_TIMER2_CC2 = 3 << POSITION,
              REGULAR_GROUP_TRIGGERED_BY_TIMER2_CC3 = 4 << POSITION,
              REGULAR_GROUP_TRIGGERED_BY_TIMER2_CC4 = 5 << POSITION,
              REGULAR_GROUP_TRIGGERED_BY_TIMER2_TRGO = 6 << POSITION,
              REGULAR_GROUP_TRIGGERED_BY_TIMER3_CC1 = 7 << POSITION,
              REGULAR_GROUP_TRIGGERED_BY_TIMER3_TRGO = 8 << POSITION,
              REGULAR_GROUP_TRIGGERED_BY_TIMER4_CC4 = 9 << POSITION,
              REGULAR_GROUP_TRIGGERED_BY_TIMER5_CC1 = 10 << POSITION,
              REGULAR_GROUP_TRIGGERED_BY_TIMER5_CC2 = 11 << POSITION,
              REGULAR_GROUP_TRIGGERED_BY_TIMER5_CC3 = 12 << POSITION,
              REGULAR_GROUP_TRIGGERED_BY_TIMER8_CC1 = 13 << POSITION,
              REGULAR_GROUP_TRIGGERED_BY_TIMER8_TRGO = 14 << POSITION,
              REGULAR_GROUP_TRIGGERED_BY_EXTI11 = 15 << POSITION,
            };
          }  // namespace states
        }  // namespace extsel

        namespace exten {
          enum {
            POSITION = 28
          };
          enum {
            MASK = 0b11 << POSITION
          };
          namespace states {
            enum E {
              REGULAR_TRIGGER_DISABLED = 0 << POSITION,
              REGULAR_TRIGGER_ON_THE_RISING_EDGE = 1 << POSITION,
              REGULAR_TRIGGER_ON_THE_FALLING_EDGE = 2 << POSITION,
              REGULAR_TRIGGER_ON_RISING_AND_FALLING_EDGES = 3 << POSITION,
            };
          }  // namespace states
        }  // namespace exten

        namespace swstart {
          enum {
            POSITION = 30
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              REGULAR_CHANNELS_ON_RESET_STATE = 0 << POSITION,
              START_CONVERSION_ON_REGULAR_CHANNELS = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace swstart
      }  // namespace bits
    }  // namespace cr2

    namespace smpr1 {
      enum {
        OFFSET = 0x0C
      };
    }  // namespace smpr1

    namespace smpr2 {
      enum {
        OFFSET = 0x10
      };
    }  // namespace smpr2

    namespace smp {
      enum {
        MASK = 0b111,
      };
      enum {
        POSITION = 3
      };
      namespace states {
        enum E {
          SAMPLING_TIME_3_CYCLES = 0,
          SAMPLING_TIME_15_CYCLES = 1,
          SAMPLING_TIME_28_CYCLES = 2,
          SAMPLING_TIME_56_CYCLES = 3,
          SAMPLING_TIME_84_CYCLES = 4,
          SAMPLING_TIME_112_CYCLES = 5,
          SAMPLING_TIME_144_CYCLES = 6,
          SAMPLING_TIME_480_CYCLES = 7,
        };
      }  // namespace states
    }  // namespace smp

    namespace jofr1 {
      enum {
        OFFSET = 0x14
      };
    }  // namespace jofr1

    namespace jofr2 {
      enum {
        OFFSET = 0x18
      };
    }  // namespace jofr2

    namespace jofr3 {
      enum {
        OFFSET = 0x1C
      };
    }  // namespace jofr3

    namespace jofr4 {
      enum {
        OFFSET = 0x20
      };
    }  // namespace jofr4

    namespace htr {
      enum {
        OFFSET = 0x24
      };
    }  // namespace htr

    namespace ltr {
      enum {
        OFFSET = 0x28
      };
    }  // namespace ltr

    namespace sqr1 {
      enum {
        OFFSET = 0x2C
      };
      namespace bits {
        namespace l {
          enum {
            POSITION = 20
          };
          enum {
            MASK = 0b1111 << POSITION
          };
          namespace states {
            enum E {
              SEQUENCE_OF_1_REGULAR_CONVERSION = 0 << POSITION,
              SEQUENCE_OF_2_REGULAR_CONVERSIONS = 1 << POSITION,
              SEQUENCE_OF_3_REGULAR_CONVERSIONS = 2 << POSITION,
              SEQUENCE_OF_4_REGULAR_CONVERSIONS = 3 << POSITION,
              SEQUENCE_OF_5_REGULAR_CONVERSIONS = 4 << POSITION,
              SEQUENCE_OF_6_REGULAR_CONVERSIONS = 5 << POSITION,
              SEQUENCE_OF_7_REGULAR_CONVERSIONS = 6 << POSITION,
              SEQUENCE_OF_8_REGULAR_CONVERSIONS = 7 << POSITION,
              SEQUENCE_OF_9_REGULAR_CONVERSIONS = 8 << POSITION,
              SEQUENCE_OF_10_REGULAR_CONVERSIONS = 9 << POSITION,
              SEQUENCE_OF_11_REGULAR_CONVERSIONS = 10 << POSITION,
              SEQUENCE_OF_12_REGULAR_CONVERSIONS = 11 << POSITION,
              SEQUENCE_OF_13_REGULAR_CONVERSIONS = 12 << POSITION,
              SEQUENCE_OF_14_REGULAR_CONVERSIONS = 13 << POSITION,
              SEQUENCE_OF_15_REGULAR_CONVERSIONS = 14 << POSITION,
              SEQUENCE_OF_16_REGULAR_CONVERSIONS = 15 << POSITION,
            };
          }  // namespace states

        }  // namespace l
      }  // namespace bits
    }  // namespace sqr1

    namespace sqr2 {
      enum {
        OFFSET = 0x30
      };
    }  // namespace sqr2

    namespace sqr3 {
      enum {
        OFFSET = 0x34
      };
    }  // namespace sqr3

    namespace sqr {
      enum {
        MASK = 0b11111
      };
      enum {
        POSITION = 5
      };
    }  // namespace sqr3

    namespace jsqr {
      enum {
        OFFSET = 0x38
      };
      namespace bits {
        namespace jl {
          enum {
            POSITION = 20
          };
          enum {
            MASK = 0b11 << POSITION
          };
          namespace states {
            enum E {
              SEQUENCE_OF_1_INJECTED_CONVERSION = 0 << POSITION,
              SEQUENCE_OF_2_INJECTED_CONVERSIONS = 1 << POSITION,
              SEQUENCE_OF_3_INJECTED_CONVERSIONS = 2 << POSITION,
              SEQUENCE_OF_4_INJECTED_CONVERSIONS = 3 << POSITION,
            };
          }  // namespace states
        }  // namespace jl
      }  // namespace bits
    }  // namespace jsqr

    namespace jsq {
      enum {
        MASK = 0b11111
      };
      enum {
        POSITION = 5
      };
    }  // namespace jsq

    namespace jdr1 {
      enum {
        OFFSET = 0x3C
      };
    }  // namespace jdr1

    namespace jdr2 {
      enum {
        OFFSET = 0x40
      };
    }  // namespace jdr2

    namespace jdr3 {
      enum {
        OFFSET = 0x44
      };
    }  // namespace jdr3

    namespace jdr4 {
      enum {
        OFFSET = 0x48
      };
    }  // namespace jdr4

    namespace dr {
      enum {
        OFFSET = 0x4C
      };
    }  // namespace dr

    namespace csr {
      enum {
        OFFSET = 0x00
      };
    }  // namespace csr

    namespace ccr {
      enum {
        OFFSET = 0x04
      };
    }  // namespace ccr

    namespace cdr {
      enum {
        OFFSET = 0x08
      };
    }  // namespace cdr

  }  // namespace registers
}  // namespace adc
