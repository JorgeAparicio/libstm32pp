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

namespace rng {
  struct Registers {
      __RW
      u32 CR; /* 0x00: Control  */
      __RW
      u32 SR; /* 0x04: Status   */
      __RW
      u32 DR; /* 0x08: Data     */
  };

  enum {
    ADDRESS = alias::address::AHB2 + 0x60800
  };

  namespace registers {
    namespace cr {
      enum {
        OFFSET = 0x00
      };
      namespace bits {
        namespace rngen {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              RANDOM_NUMBER_GENERATOR_DISABLED = 0 << POSITION,
              RANDOM_NUMBER_GENERATOR_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace rngen

        namespace ie {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              INTERRUPT_DISABLED = 0 << POSITION,
              INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ie
      }  // namespace bits
    }  // namespace cr

    namespace sr {
      enum {
        OFFSET = 0x04
      };
      namespace bits {
        namespace drdy {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_VALID_RANDOM_DATA_READY = 0 << POSITION,
              VALID_RANDOM_DATA_READY = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace drdy

        namespace cecs {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              CLOCK_OK = 0 << POSITION,
              CLOCK_ERROR_DETECTED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace cecs

        namespace secs {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SEED_OK = 0 << POSITION,
              SEED_ERROR_DETECTED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace secs

        namespace ceis {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              CLOCK_OK = 0 << POSITION,
              CLOCK_ERROR_DETECTED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ceis

        namespace seis {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SEED_OK = 0 << POSITION,
              SEED_ERROR_DETECTED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace seis
      }  // namespace bits
    }  // namespace sr

    namespace dr {
      enum {
        OFFSET = 0x08
      };
    }  // namespace dr
  }  // namespace registers
}  // namespace rng
