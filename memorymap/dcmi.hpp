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

namespace dcmi {
  enum {
    ADDRESS = alias::AHB2 + 0x50000
  };

  struct Registers {
      __RW
      u32 CR;  // 0x00: Control 1
      __RW
      u32 SR;  // 0x04: Status
      __RW
      u32 RISR;  // 0x08: Raw interrupt status
      __RW
      u32 IER;  // 0x0C: Interrupt enable
      __RW
      u32 MISR;  // 0x10: Masked interrupt status
      __RW
      u32 ICR;  // 0x14: Interrupt clear
      __RW
      u32 ESCR;  // 0x18: Embedded synchronization code
      __RW
      u32 ESUR;  // 0x1C: Embedded synchronization unmask
      __RW
      u32 CWSTRTR;  // 0x20: Crop window start
      __RW
      u32 CWSIZER;  // 0x24: Crop window size
      __RW
      u32 DR;  // 0x28: Data
  };

  namespace cr {
    enum {
      OFFSET = 0x00
    };
    namespace capture {
      enum {
        POSITION = 0,
        MASK = 0b1 << POSITION
      };

      enum States {
        CAPTURE_DISABLED = 0 << POSITION,
        CAPTURE_ENABLED = 1 << POSITION,
      };
    }  // namespace capture

    namespace cm {
      enum {
        POSITION = 1,
        MASK = 0b1 << POSITION
      };

      enum States {
        CONTINUOUS_GRAB_MODE = 0 << POSITION,
        SNAPSHOT_MODE = 1 << POSITION,
      };
    }  // namespace cm

    namespace crop {
      enum {
        POSITION = 2,
        MASK = 0b1 << POSITION
      };

      enum States {
        FULL_IMAGE_IS_CAPTURED = 0 << POSITION,
        CROPPED_IMAGE_IS_CAPTURED = 1 << POSITION,
      };
    }  // namespace crop

    namespace jpeg {
      enum {
        POSITION = 3,
        MASK = 0b1 << POSITION
      };

      enum States {
        UNCOMPRESSED_VIDEO_FORMAT = 0 << POSITION,
        JPEG_DATA_FORMAT = 1 << POSITION,
      };
    }  // namespace jpeg

    namespace ess {
      enum {
        POSITION = 4,
        MASK = 0b1 << POSITION
      };

      enum States {
        HARDWARE_SYNCHRONIZATION = 0 << POSITION,
        EMBEDDED_SYNCHRONIZATION = 1 << POSITION,
      };
    }  // namespace ess

    namespace pckpol {
      enum {
        POSITION = 5,
        MASK = 0b1 << POSITION
      };

      enum States {
        CAPTURE_ON_FALLING_EDGE = 0 << POSITION,
        CAPTURE_ON_RISING_EDGE = 1 << POSITION,
      };
    }  // namespace pckpol

    namespace hspol {
      enum {
        POSITION = 6,
        MASK = 0b1 << POSITION
      };

      enum States {
        DATA_CAPTURED_ON_HSYNC_HIGH = 0 << POSITION,
        DATA_CAPTURED_ON_HSYNC_LOW = 1 << POSITION,
      };
    }  // namespace hspol

    namespace vspol {
      enum {
        POSITION = 7,
        MASK = 0b1 << POSITION
      };

      enum States {
        DATA_CAPTURED_ON_VSYNC_HIGH = 0 << POSITION,
        DATA_CAPTURED_ON_VSYNC_LOW = 1 << POSITION,
      };
    }  // namespace vspol

    namespace fcrc {
      enum {
        POSITION = 8,
        MASK = 0b11 << POSITION
      };

      enum States {
        ALL_FRAMES_ARE_CAPTURED = 0 << POSITION,
        EVERY_ALTERNATE_FRAME_CAPTURED = 1 << POSITION,
        ONE_FRAME_IN_FOUR_FRAME_CAPTURED = 2 << POSITION
      };
    }  // namespace fcrc

    namespace edm {
      enum {
        POSITION = 10,
        MASK = 0b11 << POSITION
      };

      enum States {
        EVERY_PIXEL_CLOCK_CAPTURES_8_BITS = 0 << POSITION,
        EVERY_PIXEL_CLOCK_CAPTURES_10_BITS = 1 << POSITION,
        EVERY_PIXEL_CLOCK_CAPTURES_12_BITS = 2 << POSITION,
        EVERY_PIXEL_CLOCK_CAPTURES_14_BITS = 3 << POSITION
      };
    }  // namespace edm

    namespace enable {
      enum {
        POSITION = 14,
        MASK = 0b1 << POSITION
      };

      enum States {
        DCMI_DISABLED = 0 << POSITION,
        DCMI_ENABLED = 1 << POSITION
      };
    }  // namespace enable
  }  // namespace cr

  namespace sr {
    enum {
      OFFSET = 0x04
    };
    namespace hsync {
      enum {
        POSITION = 0,
        MASK = 0b1 << POSITION
      };

      enum States {
        ACTIVE_LINE = 0 << POSITION,
        SYNCHRONIZATION_BETWEEN_LINES = 1 << POSITION
      };
    }  // namespace hsync

    namespace vsync {
      enum {
        POSITION = 1,
        MASK = 0b1 << POSITION
      };

      enum States {
        ACTIVE_FRAME = 0 << POSITION,
        SYNCHRONIZATION_BETWEEN_FRAMES = 1 << POSITION
      };
    }  // namespace vsync

    namespace fne {
      enum {
        POSITION = 2,
        MASK = 0b1 << POSITION
      };

      enum States {
        FIFO_EMPTY = 0 << POSITION,
        FIFO_CONTAINS_VALID_DATA = 1 << POSITION
      };
    }  // namespace fne
  }  // namespace sr

  namespace risr {
    enum {
      OFFSET = 0x08
    };
    namespace frame {
      enum {
        POSITION = 0,
        MASK = 0b1 << POSITION
      };

      enum States {
        NO_NEW_CAPTURE = 0 << POSITION,
        A_FRAME_HAS_BEEN_CAPTURED = 1 << POSITION
      };
    }  // namespace frame

    namespace ovr {
      enum {
        POSITION = 1,
        MASK = 0b1 << POSITION
      };

      enum States {
        NO_DATA_BUFFER_OVERRUN_OCURRED = 0 << POSITION,
        A_DATA_BUFFER_OVERRUN_OCCURED = 1 << POSITION
      };
    }  // namespace ovr

    namespace err {
      enum {
        POSITION = 2,
        MASK = 0b1 << POSITION
      };

      enum States {
        NO_SYNCHRONIZATION_ERROR = 0 << POSITION,
        EMBEDDED_SYNCHRONIZATION_ERROR = 1 << POSITION
      };
    }  // namespace err

    namespace vsync {
      enum {
        POSITION = 3,
        MASK = 0b1 << POSITION
      };

      enum States {
        NO_NEW_FRAME_DETECTED = 0 << POSITION,
        NEW_FRAME_SYNCHRONIZATION_OCCURRED = 1 << POSITION
      };
    }  // namespace vsync

    namespace line {
      enum {
        POSITION = 4,
        MASK = 0b1 << POSITION
      };

      enum States {
        NO_NEW_LINE_DETECTED = 0 << POSITION,
        NEW_LINE_SYNCHRONIZATION_OCCURRED = 1 << POSITION
      };
    }  // namespace line
  }  // namespace risr

  namespace ier {
    enum {
      OFFSET = 0x0C
    };
    namespace frame {
      enum {
        POSITION = 0,
        MASK = 0b1 << POSITION
      };

      enum States {
        CAPTURE_COMPLETE_INTERRUPT_DISABLED = 0 << POSITION,
        CAPTURE_COMPLETE_INTERRUPT_ENABLED = 1 << POSITION
      };
    }  // namespace frame

    namespace ovr {
      enum {
        POSITION = 1,
        MASK = 0b1 << POSITION
      };

      enum States {
        OVERRUN_ERROR_INTERRUPT_DISABLED = 0 << POSITION,
        OVERRUN_ERROR_INTERRUPT_ENABLED = 1 << POSITION
      };
    }  // namespace ovr

    namespace err {
      enum {
        POSITION = 2,
        MASK = 0b1 << POSITION
      };

      enum States {
        SYNCHRONIZATION_ERROR_INTERRUPT_DISABLED = 0 << POSITION,
        SYNCHRONIZATION_ERROR_INTERRUPT_ENABLED = 1 << POSITION
      };
    }  // namespace err

    namespace vsync {
      enum {
        POSITION = 3,
        MASK = 0b1 << POSITION
      };

      enum States {
        NEW_FRAME_SYNCHRONIZATION_INTERRUPT_DISABLED = 0 << POSITION,
        NEW_FRAME_SYNCHRONIZATION_INTERRUPT_ENABLED = 1 << POSITION
      };
    }  // namespace vsync

    namespace line {
      enum {
        POSITION = 4,
        MASK = 0b1 << POSITION
      };

      enum States {
        NEW_LINE_RECEIVED_INTERRUPT_DISABLED = 0 << POSITION,
        NEW_LINE_RECEIVED_INTERRUPT_ENABLED = 1 << POSITION
      };
    }  // namespace line
  }  // namespace ier

  namespace misr {
    enum {
      OFFSET = 0x10
    };
    namespace frame {
      enum {
        POSITION = 0,
        MASK = 0b1 << POSITION
      };

      enum States {
        CAPTURE_COMPLETE_INTERRUPT_NOT_GENERATED = 0 << POSITION,
        CAPTURE_COMPLETE_INTERRUPT_GENERATED = 1 << POSITION
      };
    }  // namespace frame

    namespace ovr {
      enum {
        POSITION = 1,
        MASK = 0b1 << POSITION
      };

      enum States {
        OVERRUN_ERROR_INTERRUPT_NOT_GENERATED = 0 << POSITION,
        OVERRUN_ERROR_INTERRUPT_GENERATED = 1 << POSITION
      };
    }  // namespace ovr

    namespace err {
      enum {
        POSITION = 2,
        MASK = 0b1 << POSITION
      };

      enum States {
        SYNCHRONIZATION_ERROR_INTERRUPT_NOT_GENERATED = 0 << POSITION,
        SYNCHRONIZATION_ERROR_INTERRUPT_GENERATED = 1 << POSITION
      };
    }  // namespace err

    namespace vsync {
      enum {
        POSITION = 3,
        MASK = 0b1 << POSITION
      };

      enum States {
        NEW_FRAME_SYNCHRONIZATION_INTERRUPT_NOT_GENERATED = 0 << POSITION,
        NEW_FRAME_SYNCHRONIZATION_INTERRUPT_GENERATED = 1 << POSITION
      };
    }  // namespace vsync

    namespace line {
      enum {
        POSITION = 4,
        MASK = 0b1 << POSITION
      };

      enum States {
        NEW_LINE_RECEIVED_INTERRUPT_NOT_GENERATED = 0 << POSITION,
        NEW_LINE_RECEIVED_INTERRUPT_GENERATED = 1 << POSITION
      };
    }  // namespace line
  }  // namespace misr

  namespace icr {
    enum {
      OFFSET = 0x14
    };
    namespace frame {
      enum {
        POSITION = 0,
        MASK = 0b1 << POSITION
      };

      enum States {
        NO_EFFECT = 0 << POSITION,
        CLEARS_CAPTURE_COMPLETE_INTERRUPT_FLAG = 1 << POSITION
      };
    }  // namespace frame

    namespace ovr {
      enum {
        POSITION = 1,
        MASK = 0b1 << POSITION
      };

      enum States {
        NO_EFFECT = 0 << POSITION,
        CLEARS_OVERRUN_ERROR_INTERRUPT_FLAG = 1 << POSITION
      };
    }  // namespace ovr

    namespace err {
      enum {
        POSITION = 2,
        MASK = 0b1 << POSITION
      };

      enum States {
        NO_EFFECT = 0 << POSITION,
        CLEARS_SYNCHRONIZATION_ERROR_INTERRUPT_FLAG = 1 << POSITION
      };
    }  // namespace err

    namespace vsync {
      enum {
        POSITION = 3,
        MASK = 0b1 << POSITION
      };

      enum States {
        NO_EFFECT = 0 << POSITION,
        CLEARS_NEW_FRAME_SYNCHRONIZATION_INTERRUPT_FLAG = 1 << POSITION
      };
    }  // namespace vsync

    namespace line {
      enum {
        POSITION = 4,
        MASK = 0b1 << POSITION
      };

      enum States {
        NO_EFFECT = 0 << POSITION,
        CLEARS_NEW_LINE_RECEIVED_INTERRUPT_FLAG = 1 << POSITION
      };
    }  // namespace line
  }  // namespace icr

  namespace escr {
    enum {
      OFFSET = 0x18
    };
  // TODO DCMI ESCR bits
  }// namespace escr

  namespace esur {
    enum {
      OFFSET = 0x1C
    };
  // TODO DCMI ESUR bits
  }// namespace esur

  namespace cwstrtr {
    enum {
      OFFSET = 0x20
    };
    namespace hoffcnt {
      enum {
        POSITION = 0,
        MASK = 0b1111111111111 << POSITION
      };
    }  // namespace capcnt

    namespace vst {
      enum {
        POSITION = 16,
        MASK = 0b111111111111 << POSITION
      };
    }  // namespace vline
  }  // namespace cwstrtr

  namespace cwsizer {
    enum {
      OFFSET = 0x28
    };
    namespace capcnt {
      enum {
        POSITION = 0,
        MASK = 0b1111111111111 << POSITION
      };
    }  // namespace capcnt

    namespace vline {
      enum {
        POSITION = 16,
        MASK = 0b1111111111111 << POSITION
      };
    }  // namespace vline
  }  // namespace cwsizer

  namespace dr {
    enum {
      OFFSET = 0x2C
    };
  }  // namespace dr
}  // namespace dcmi
