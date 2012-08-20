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
  enum {
    ADDRESS = alias::AHB2 + 0x60800
  };

  struct Registers {
      __RW
      u32 CR; /* 0x00: Control  */
      __RW
      u32 SR; /* 0x04: Status   */
      __RW
      u32 DR; /* 0x08: Data     */
  };

  namespace cr {
    enum {
      OFFSET = 0x00
    };
    namespace rngen {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        RANDOM_NUMBER_GENERATOR_DISABLED = 0 << POSITION,
        RANDOM_NUMBER_GENERATOR_ENABLED = 1 << POSITION,
      };
    }  // namespace rngen

    namespace ie {
      enum {
        POSITION = 3,
        MASK = 1 << POSITION
      };
      enum States {
        INTERRUPT_DISABLED = 0 << POSITION,
        INTERRUPT_ENABLED = 1 << POSITION,
      };
    }  // namespace ie
  }  // namespace cr

  namespace sr {
    enum {
      OFFSET = 0x04
    };
    namespace drdy {
      enum {
        POSITION = 0,
        MASK = 1 << POSITION
      };
      enum States {
        NO_VALID_RANDOM_DATA_READY = 0 << POSITION,
        VALID_RANDOM_DATA_READY = 1 << POSITION,
      };
    }  // namespace drdy

    namespace cecs {
      enum {
        POSITION = 1,
        MASK = 1 << POSITION
      };
      enum States {
        CLOCK_OK = 0 << POSITION,
        CLOCK_ERROR_DETECTED = 1 << POSITION,
      };
    }  // namespace cecs

    namespace secs {
      enum {
        POSITION = 2,
        MASK = 1 << POSITION
      };
      enum States {
        SEED_OK = 0 << POSITION,
        SEED_ERROR_DETECTED = 1 << POSITION,
      };
    }  // namespace secs

    namespace ceis {
      enum {
        POSITION = 5,
        MASK = 1 << POSITION
      };
      enum States {
        CLOCK_OK = 0 << POSITION,
        CLOCK_ERROR_DETECTED = 1 << POSITION,
      };
    }  // namespace ceis

    namespace seis {
      enum {
        POSITION = 6,
        MASK = 1 << POSITION
      };
      enum States {
        SEED_OK = 0 << POSITION,
        SEED_ERROR_DETECTED = 1 << POSITION,
      };
    }  // namespace seis
  }  // namespace sr

  namespace dr {
    enum {
      OFFSET = 0x08
    };
  }  // namespace dr
}  // namespace rng
