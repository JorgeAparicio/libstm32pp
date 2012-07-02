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

namespace tim {
  namespace basic {
    struct Registers {
        __RW
        u32 CR1;  // 0x00: Control 1
        __RW
        u32 CR2;  // 0x04: Control 2
        u32 _RESERVED0;
        __RW
        u32 DIER;  // 0x0C: DMA/Interrupt enable
        __RW
        u32 SR;  // 0x10: Status
        __RW
        u32 EGR;  // 0x14: Event generation
        u32 _RESERVED1[3];
        __RW
        u32 CNT;  // 0x24: Counter
        __RW
        u32 PSC;  // 0x28: Prescaler
        __RW
        u32 ARR;  // 0x2C: Auto-reload
    };

    namespace address {
      enum E {
        TIM6 = alias::address::APB1 + 0x1000,
        TIM7 = alias::address::APB1 + 0x1400,
      };
    }  // namespace address
  }  // namespace basic

  namespace generalPurpose1 {
    struct Registers {
        __RW
        u32 CR1;    // 0x00: Control 1
        __RW
        u32 CR2;    // 0x04: Control 2
        __RW
        u32 SMCR;   // 0x08: Slave mode control
        __RW
        u32 DIER;   // 0x0C: DMA/Interrupt enable
        __RW
        u32 SR;     // 0x10: Status
        __RW
        u32 EGR;    // 0x14: Event generation
        __RW
        u32 CCMR[2];  // 0x18, 0x1C: Capture/Compare mode
        __RW
        u32 CCER;   // 0x20: Capture/Compare enable
        __RW
        u32 CNT;    // 0x24: Counter
        __RW
        u32 PSC;    // 0x28: Prescaler
        __RW
        u32 ARR;    // 0x2C: Auto-reload
        u32 _RESERVED0;
        __RW
        u32 CCR[4];   // 0x34-0x40: Capture/Compare
        u32 _RESERVED1;
        __RW
        u32 DCR;    // 0x48: DMA control
        __RW
        u32 DMAR;   // 0x4C: DMA address for full transfer
        __RW
        u32 OR;     // 0x50: Option
    };

    namespace address {
      enum E {
        TIM2 = alias::address::APB1 + 0x0000,
        TIM3 = alias::address::APB1 + 0x0400,
        TIM4 = alias::address::APB1 + 0x0800,
        TIM5 = alias::address::APB1 + 0x0C00,
      };
    }  // namespace address
  }  // namespace generalPurpose1

  namespace generalPurpose2 {
    struct Registers {
        __RW
        u32 CR1;  // 0x00: Control 1
        __RW
        u32 CR2;  // 0x04: Control 2
        __RW
        u32 SMCR;  // 0x08: Slave mode control
        __RW
        u32 DIER;  // 0x0C: DMA/Interrupt enable
        __RW
        u32 SR;  // 0x10: Status
        __RW
        u32 EGR;  // 0x14: Event generation
        __RW
        u32 CCMR1;  // 0x18: Capture/Compare mode 1
        u32 _RESERVED0;
        __RW
        u32 CCER;  // 0x20: Capture/Compare enable
        __RW
        u32 CNT;  // 0x24: Counter
        __RW
        u32 PSC;  // 0x28: Prescaler
        __RW
        u32 ARR;  // 0x2C: Auto-reload
        u32 _RESERVED1;
        __RW
        u32 CCR1;  // 0x34: Capture/Compare 1
        __RW
        u32 CCR2;  // 0x38: Capture/Compare 2
    };

    namespace address {
      enum E {
        TIM9 = alias::address::APB2 + 0x4000,
        TIM12 = alias::address::APB1 + 0x1800,
      };
    }  // namespace address
  }  // namespace generalPurpose2

  namespace generalPurpose3 {
    struct Registers {
        __RW
        u32 CR1;  // 0x00: Control 1
        u32 _RESERVED0[2];
        __RW
        u32 DIER;  // 0x0C: DMA/Interrupt enable
        __RW
        u32 SR;  // 0x10: Status
        __RW
        u32 EGR;  // 0x14: Event generation
        __RW
        u32 CCMR1;  // 0x18: Capture/Compare mode 1
        u32 _RESERVED1;
        __RW
        u32 CCER;  // 0x20: Capture/Compare enable
        __RW
        u32 CNT;  // 0x24: Counter
        __RW
        u32 PSC;  // 0x28: Prescaler
        __RW
        u32 ARR;  // 0x2C: Auto-reload
        u32 _RESERVED2;
        __RW
        u32 CCR1;  // 0x34: Capture/Compare 1
        u32 _RESERVED3[6];
        __RW
        u32 OR;  // 0x50: Option
    };

    namespace address {
      enum E {
#ifdef STM32F1XX
        TIM10 = alias::address::APB2 + 0x5000,
        TIM11 = alias::address::APB2 + 0x5400,
        #else
        TIM10 = alias::address::APB2 + 0x4400,
        TIM11 = alias::address::APB2 + 0x4800,

#endif
        TIM13 = alias::address::APB1 + 0x1C00,
        TIM14 = alias::address::APB1 + 0x2000,
      };
    }  // namespace address
  }  // namespace generalPurpose3

#ifdef VALUE_LINE
  namespace generalPurpose4 {
    struct Registers {
        __RW
        u32 CR1;  // 0x00: Control 1
        __RW
        u32 CR2;  // 0x04: Control 2
        u32 _RESERVED0;
        __RW
        u32 DIER;  // 0x0C: DMA/Interrupt enable
        __RW
        u32 SR;  // 0x10: Status
        __RW
        u32 EGR;  // 0x14: Event generation
        __RW
        u32 CCMR1;  // 0x18: Capture/Compare mode 1
        u32 _RESERVED1;
        __RW
        u32 CCER;  // 0x20: Capture/Compare enable
        __RW
        u32 CNT;  // 0x24: Counter
        __RW
        u32 PSC;  // 0x28: Prescaler
        __RW
        u32 ARR;  // 0x2C: Auto-reload
        __RW
        u32 RCR;  // 0x30: Repetition counter
        __RW
        u32 CCR1;  // 0x34: Capture/Compare 1
        u32 _RESERVED2[3];
        __RW
        u32 BDTR;  // 0x44: Break and dead-time
        __RW
        u32 DCR;  // 0x48: DMA control
        __RW
        u32 DMAR;  // 0x4C: DMA address for full transfer
    };

    namespace address {
      enum E {
        TIM15 = alias::address::APB2 + 0x4000,
        TIM16 = alias::address::APB2 + 0x4400,
        TIM17 = alias::address::APB2 + 0x4800,
      };
    }  // namespace address
  }  // namespace generalPurpose4
#endif

  namespace advancedControl {
    struct Registers {
        __RW
        u32 CR1;  // 0x00: Control 1
        __RW
        u32 CR2;  // 0x04: Control 2
        __RW
        u32 SMCR;  // 0x08: Slave mode control
        __RW
        u32 DIER;  // 0x0C: DMA/Interrupt enable
        __RW
        u32 SR;  // 0x10: Status
        __RW
        u32 EGR;  // 0x14: Event generation
        __RW
        u32 CCMR1;  // 0x18: Capture/Compare mode 1
        __RW
        u32 CCMR2;  // 0x1C: Capture/Compare mode 2
        __RW
        u32 CCER;  // 0x20: Capture/Compare enable
        __RW
        u32 CNT;  // 0x24: Counter
        __RW
        u32 PSC;  // 0x28: Prescaler
        __RW
        u32 ARR;  // 0x2C: Auto-reload
        __RW
        u32 RCR;  // 0x30: Repetition counter
        __RW
        u32 CCR1;  // 0x34: Capture/Compare 1
        __RW
        u32 CCR2;  // 0x38: Capture/Compare 2
        __RW
        u32 CCR3;  // 0x3C: Capture/Compare 3
        __RW
        u32 CCR4;  // 0x40: Capture/Compare 4
        __RW
        u32 BDTR;  // 0x44: Break and dead-time
        __RW
        u32 DCR;  // 0x48: DMA control
        __RW
        u32 DMAR;  // 0x4C: DMA address for full transfer
    };
    namespace address {
      enum E {
#ifdef STM32F1XX
        TIM1 = alias::address::APB2 + 0x2C00,
        TIM8 = alias::address::APB2 + 0x3400,
#else
      TIM1 = alias::address::APB2 + 0x0000,
      TIM8 = alias::address::APB2 + 0x0400,
#endif
      };
    }  // namespace address
  }  // namespace advancedControl

  // TODO General-Purpose and Advanced-Control-Timers register bits
  namespace registers {
    namespace cr1 {
      enum {
        OFFSET = 0x00
      };

      namespace bits {
        namespace cen {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              COUNTER_DISABLED = 0 << POSITION,
              COUNTER_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace cen

        namespace udis {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              UPDATE_EVENT_ENABLED = 0 << POSITION,
              UPDATE_EVENT_DISABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace udis

        namespace urs {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              UPDATE_REQUEST_SOURCE_ALL = 0 << POSITION,
              UPDATE_REQUEST_SOURCE_OVERFLOW_UNDERFLOW = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace urs

        namespace opm {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DONT_STOP_COUNTER_AT_NEXT_UPDATE_EVENT = 0 << POSITION,
              STOP_COUNTER_AT_NEXT_UPDATE_EVENT = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace opm

        namespace arpe {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              AUTO_RELOAD_UNBUFFERED = 0 << POSITION,
              AUTO_RELOAD_BUFFERED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace arpe
      }  // namespace bits
    }  // namespace cr1

    namespace cr2 {
      enum {
        OFFSET = 0x04
      };

      namespace bits {
        namespace mms {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 0b111 << POSITION
          };
          namespace states {
            enum E {
              RESET = 0 << POSITION,
              ENABLE = 1 << POSITION,
              UPDATE = 2 << POSITION,
            };
          }  // namespace states
        }  // namespace mms
      }  // namespace bits
    }  // namespace cr2

    namespace dier {
      enum {
        OFFSET = 0x0C
      };

      namespace bits {
        namespace uie {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              INTERRUPT_DISABLED = 0 << POSITION,
              INTERUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace uie

        namespace ude {
          enum {
            POSITION = 8
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
        }  // namespace ude
      }  // namespace bits
    }  // namespace dier

    namespace sr {
      enum {
        OFFSET = 0x10
      };

      namespace bits {
        namespace uif {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_UPDATE_OCURRED = 0 << POSITION,
              UPDATE_INTERRUPT_PENDING = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace uif
      }  // namespace bits
    }  // namespace sr

    namespace egr {
      enum {
        OFFSET = 0x14
      };

      namespace bits {
        namespace ug {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_ACTION = 0 << POSITION,
              GENERATE_AN_UPDATE = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ug
      }  // namespace bits
    }  // namespace egr
  }  // namespace registers
}  // namespace tim
