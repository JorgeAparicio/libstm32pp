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

namespace lsm303dlhc {
  namespace accelerometer {
    enum {
      ADDRESS = 0b0011001
    };

    namespace ctrl1 {
      enum {
        ADDRESS = 0x20
      };

      namespace xen {
        enum {
          POSITION = 0,
          MASK = 1 << POSITION
        };
        enum States {
          XXIS_DISABLED = 0 << POSITION,
          XXIS_ENABLED = 1 << POSITION,
        };
      }  // namespace xen

      namespace yen {
        enum {
          POSITION = 1,
          MASK = 1 << POSITION
        };
        enum States {
          YXIS_DISABLED = 0 << POSITION,
          YXIS_ENABLED = 1 << POSITION,
        };

      }  // namespace yen

      namespace zen {
        enum {
          POSITION = 2,
          MASK = 1 << POSITION
        };
        enum States {
          ZXIS_DISABLED = 0 << POSITION,
          ZXIS_ENABLED = 1 << POSITION,
        };
      }  // namespace zen

      namespace lpen {
        enum {
          POSITION = 3,
          MASK = 1 << POSITION
        };
        enum States {
          NORMAL_MODE = 0 << POSITION,
          LOW_POWER_MODE = 1 << POSITION,
        };
      }  // namespace lpen

      namespace odr {
        enum {
          POSITION = 4,
          MASK = 0b1111 << POSITION
        };
        enum States {
          POWER_DOWN_MODE = 0 << POSITION,
          DATA_RATE_1HZ = 1 << POSITION,
          DATA_RATE_10HZ = 2 << POSITION,
          DATA_RATE_25HZ = 3 << POSITION,
          DATA_RATE_50HZ = 4 << POSITION,
          DATA_RATE_100HZ = 5 << POSITION,
          DATA_RATE_200HZ = 6 << POSITION,
          DATA_RATE_400HZ = 7 << POSITION,
          LOW_POWER_DATA_RATE_1620HZ = 8 << POSITION,
          NORMAL_DATA_RATE_1344HZ = 9 << POSITION,
          LOW_POWER_DATA_RATE_MODE_5376HZ = 9 << POSITION,
        };
      }  // namespace odr
    }  // namespace ctrl1

    namespace ctrl2 {
      enum {
        ADDRESS = 0x21
      };

      namespace hpis1 {
        enum {
          POSITION = 0,
          MASK = 1 << POSITION
        };
        enum States {
          HIGH_PASS_FILTER_DISABLED_ON_INTERRUPT1 = 0 << POSITION,
          HIGH_PASS_FILTER_ENABLED_ON_INTERRUPT1 = 1 << POSITION,
        };
      }  // namespace hpis1

      namespace hpis2 {
        enum {
          POSITION = 1,
          MASK = 1 << POSITION
        };
        enum States {
          HIGH_PASS_FILTER_DISABLED_ON_INTERRUPT2 = 0 << POSITION,
          HIGH_PASS_FILTER_ENABLED_ON_INTERRUPT2 = 1 << POSITION,
        };
      }  // namespace hpis2

      namespace hpclick {
        enum {
          POSITION = 2,
          MASK = 1 << POSITION
        };
        enum States {
          HIGH_PASS_FILTER_DISABLED_ON_CLICK = 0 << POSITION,
          HIGH_PASS_FILTER_ENABLED_ON_CLICK = 1 << POSITION,
        };
      }  // namespace hpclick

      namespace fds {
        enum {
          POSITION = 3,
          MASK = 1 << POSITION
        };
        enum States {
          HIGH_PASS_FILTER_NOTPPLIED_TO_DATA = 0 << POSITION,
          HIGH_PASS_FILTERPPLIED_TO_DATA = 1 << POSITION,
        };
      }  // namespace fds

      namespace hpcf {
        enum {
          POSITION = 4,
          MASK = 0b11 << POSITION
        };
        enum States {
          CUT_OFF_FREQUENCY_1 = 0 << POSITION,
          CUT_OFF_FREQUENCY_2 = 1 << POSITION,
          CUT_OFF_FREQUENCY_3 = 2 << POSITION,
          CUT_OFF_FREQUENCY_4 = 3 << POSITION,
        };
      }  // namespace hpcf

      namespace hpm {
        enum {
          POSITION = 6,
          MASK = 0b11 << POSITION
        };
        enum States {
          NORMAL_MODE_WITH_RESET = 0 << POSITION,
          FILTER_WITH_REFERENCE = 1 << POSITION,
          NORMAL_MODE = 2 << POSITION,
          AUTORESET_ON_INTERRUPT = 3 << POSITION,
        };
      }  // namespace hpm
    }  // namespace ctrl2

    namespace ctrl3 {
      enum {
        ADDRESS = 0x22
      };
      namespace i1_overrun {
        enum {
          POSITION = 1,
          MASK = 1 << POSITION
        };
        enum States {
          FIFO_OVERRUN_INTERRUPT1_DISABLED = 0 << POSITION,
          FIFO_OVERRUN_INTERRUPT1_ENABLED = 1 << POSITION,
        };
      }  // namespace i1_overrun

      namespace i1_wtm {
        enum {
          POSITION = 2,
          MASK = 1 << POSITION
        };
        enum States {
          FIFO_WATERMARK_INTERRUPT1_DISABLED = 0 << POSITION,
          FIFO_WATERMARK_INTERRUPT1_ENABLED = 1 << POSITION,
        };
      }  // namespace i1_wtm

      namespace i1_drdy2 {
        enum {
          POSITION = 3,
          MASK = 1 << POSITION
        };
        enum States {
          DATA_READY2_INTERRUPT1_DISABLED = 0 << POSITION,
          DATA_READY2_INTERRUPT1_ENABLED = 1 << POSITION,
        };
      }  // namespace i1_drdy2

      namespace i1_drdy1 {
        enum {
          POSITION = 4,
          MASK = 1 << POSITION
        };
        enum States {
          DATA_READY1_INTERRUPT1_DISABLED = 0 << POSITION,
          DATA_READY1_INTERRUPT1_ENABLED = 1 << POSITION,
        };
      }  // namespace i1_drdy1

      namespace i1oi2 {
        enum {
          POSITION = 5,
          MASK = 1 << POSITION
        };
        enum States {
          AOI2_INTERRUPT1_DISABLED = 0 << POSITION,
          AOI2_INTERRUPT1_ENABLED = 1 << POSITION,
        };
      }  // namespace i1oi2

      namespace i1oi1 {
        enum {
          POSITION = 6,
          MASK = 1 << POSITION
        };
        enum States {
          AOI1_INTERRUPT1_DISABLED = 0 << POSITION,
          AOI1_INTERRUPT1_ENABLED = 1 << POSITION,
        };
      }  // namespace i1oi1

      namespace i1_click {
        enum {
          POSITION = 7,
          MASK = 1 << POSITION
        };
        enum States {
          CLICK_INTERRUPT1_DISABLED = 0 << POSITION,
          CLICK_INTERRUPT1_ENABLED = 1 << POSITION,
        };
      }  // namespace i1_click
    }  // namespace ctrl3

    namespace ctrl4 {
      enum {
        ADDRESS = 0x23
      };

      namespace sim {
        enum {
          POSITION = 0,
          MASK = 1 << POSITION
        };
        enum States {
          SPI_4_WIRE_INTERFACE = 0 << POSITION,
          SPI_3_WIRE_INTERFACE = 1 << POSITION,
        };
      }  // namespace sim

      namespace hr {
        enum {
          POSITION = 3,
          MASK = 1 << POSITION
        };
        enum States {
          HIGH_RESOLUTION_DISABLED = 0 << POSITION,
          HIGH_RESOLUTION_ENABLED = 1 << POSITION,
        };
      }  // namespace hr

      namespace fs {
        enum {
          POSITION = 4,
          MASK = 0b11 << POSITION
        };
        enum States {
          RANGE_PLUS_MINUS_2G = 0 << POSITION,
          RANGE_PLUS_MINUS_4G = 1 << POSITION,
          RANGE_PLUS_MINUS_8G = 2 << POSITION,
          RANGE_PLUS_MINUS_16G = 3 << POSITION,
        };
      }  // namespace fs

      namespace ble {
        enum {
          POSITION = 6,
          MASK = 1 << POSITION
        };
        enum States {
          LITTLE_ENDIAN_DATA_FORMAT = 0 << POSITION,
          BIG_ENDIAN_DATA_FORMAT = 1 << POSITION,
        };
      }  // namespace ble

      namespace bdu {
        enum {
          POSITION = 7,
          MASK = 1 << POSITION
        };
        enum States {
          CONTINUOUS_UPDATE = 0 << POSITION,
          DATA_BLOCKED_UNTIL_READ = 1 << POSITION,
        };
      }  // namespace bdu
    }  // namespace ctrl4

    namespace ctrl5 {
      enum {
        ADDRESS = 0x24
      };

      namespace d4d_int2 {
        enum {
          POSITION = 0,
          MASK = 1 << POSITION
        };
        enum States {
          _4D_DETECTION_INTERRUPT2_DISABLED = 0 << POSITION,
          _4D_DETECTION_INTERRUPT2_ENABLED = 1 << POSITION,
        };
      }  // namespace d4d_int2

      namespace lir_int2 {
        enum {
          POSITION = 1,
          MASK = 1 << POSITION
        };
        enum States {
          LATCH_INTERRUPT2_DISABLED = 0 << POSITION,
          LATCH_INTERRUPT2_ENABLED = 1 << POSITION,
        };
      }  // namespace lir_int2

      namespace d4d_int1 {
        enum {
          POSITION = 2,
          MASK = 1 << POSITION
        };
        enum States {
          _4D_DETECTION_INTERRUPT1_DISABLED = 0 << POSITION,
          _4D_DETECTION_INTERRUPT1_ENABLED = 1 << POSITION,
        };
      }  // namespace d4d_int1

      namespace lir_int1 {
        enum {
          POSITION = 3,
          MASK = 1 << POSITION
        };
        enum States {
          LATCH_INTERRUPT1_DISABLED = 0 << POSITION,
          LATCH_INTERRUPT1_ENABLED = 1 << POSITION,
        };
      }  // namespace lir_int1

      namespace fifo_en {
        enum {
          POSITION = 6,
          MASK = 1 << POSITION
        };
        enum States {
          FIFO_DISABLED = 0 << POSITION,
          FIFO_ENABLED = 1 << POSITION,
        };
      }  // namespace fifo_en

      namespace boot {
        enum {
          POSITION = 7,
          MASK = 1 << POSITION
        };
        enum States {
          DONT_REBOOT_MEMORY_CONTENT = 0 << POSITION,
          REBOOT_MEMORY_CONTENT = 1 << POSITION,
        };
      }  // namespace boot
    }  // namespace ctrl5

    namespace ctrl6 {
      enum {
        ADDRESS = 0x25
      };

      namespace h_lactive {
        enum {
          POSITION = 1,
          MASK = 1 << POSITION
        };
        enum States {
          INTERRUPTCTIVE_HIGH = 0 << POSITION,
          INTERRUPTCTIVE_LOW = 1 << POSITION,
        };
      }  // namespace h_lactive
    }  // namespace ctrl6

    namespace reference {
      enum {
        ADDRESS = 0x26
      };
    }  // namespace reference

    namespace status {
      enum {
        ADDRESS = 0x27
      };

      namespace xda {
        enum {
          POSITION = 0,
          MASK = 1 << POSITION
        };
        enum States {
          XXIS_NEW_DATA_NOTVAILABLE = 0 << POSITION,
          XXIS_NEW_DATAVAILABLE = 1 << POSITION,
        };
      }  // namespace xda

      namespace yda {
        enum {
          POSITION = 1,
          MASK = 1 << POSITION
        };
        enum States {
          YXIS_NEW_DATA_NOTVAILABLE = 0 << POSITION,
          YXIS_NEW_DATAVAILABLE = 1 << POSITION,
        };
      }  // namespace yda

      namespace zda {
        enum {
          POSITION = 2,
          MASK = 1 << POSITION
        };
        enum States {
          ZXIS_NEW_DATA_NOTVAILABLE = 0 << POSITION,
          ZXIS_NEW_DATAVAILABLE = 1 << POSITION,
        };
      }  // namespace zda

      namespace zyxda {
        enum {
          POSITION = 3,
          MASK = 1 << POSITION
        };
        enum States {
          XYZXIS_NEW_DATA_NOTVAILABLE = 0 << POSITION,
          XYZXIS_NEW_DATAVAILABLE = 1 << POSITION,
        };
      }  // namespace zyxda

      namespace xor_ {
        enum {
          POSITION = 4,
          MASK = 1 << POSITION
        };
        enum States {
          NO_OVERRUN = 0 << POSITION,
          XXIS_DATA_OVERRUN = 1 << POSITION,
        };
      }  // namespace xor_

      namespace yor {
        enum {
          POSITION = 5,
          MASK = 1 << POSITION
        };
        enum States {
          NO_OVERRUN = 0 << POSITION,
          YXIS_DATA_OVERRUN = 1 << POSITION,
        };
      }  // namespace yor

      namespace zor {
        enum {
          POSITION = 6,
          MASK = 1 << POSITION
        };
        enum States {
          NO_OVERRUN = 0 << POSITION,
          ZXIS_DATA_OVERRUN = 1 << POSITION,
        };
      }  // namespace zor

      namespace zyxor {
        enum {
          POSITION = 7,
          MASK = 1 << POSITION
        };
        enum States {
          NO_OVERRUN = 0 << POSITION,
          XYZXIS_DATA_OVERRUN = 1 << POSITION,
        };
      }  // namespace zyxor
    }  // namespace status

    namespace out_x_l {
      enum {
        ADDRESS = 0x28
      };
    }  // namespace out_x_l

    namespace out_x_h {
      enum {
        ADDRESS = 0x29
      };
    }  // namespace out_x_h

    namespace out_y_l {
      enum {
        ADDRESS = 0x2A
      };
    }  // namespace out_y_l

    namespace out_y_h {
      enum {
        ADDRESS = 0x2B
      };
    }  // namespace out_y_h

    namespace out_z_l {
      enum {
        ADDRESS = 0x2C
      };
    }  // namespace out_z_l

    namespace out_z_h {
      enum {
        ADDRESS = 0x2D
      };
    }  // namespace out_z_h

    namespace fifo_ctrl {
      enum {
        ADDRESS = 0x2E
      };

      namespace tr {
        enum {
          POSITION = 5,
          MASK = 1 << POSITION
        };
        enum States {
          TRIGGER_EVENT_LINKED_TO_INTERRUPT1 = 0 << POSITION,
          TRIGGER_EVENT_LINKED_TO_INTERRUPT2 = 1 << POSITION,
        };
      }  // namespace tr

      namespace fm {
        enum {
          POSITION = 6,
          MASK = 0b11 << POSITION
        };
        enum States {
          BYPASS_MODE = 0 << POSITION,
          FIFO_MODE = 1 << POSITION,
          STREAM_MODE = 2 << POSITION,
          TRIGGER_MODE = 3 << POSITION,
        };
      }  // namespace fm
    }  // namespace fifo_ctrl

    namespace fifo_src {
      enum {
        ADDRESS = 0x2F
      };
    }  // namespace fifo_src

    namespace int1_cfg {
      enum {
        ADDRESS = 0x30
      };

      namespace xlie_xdowne {
        enum {
          POSITION = 0,
          MASK = 1 << POSITION
        };
        enum States {
          XXIS_DOWN_LOW_INTERRUPT1_DISABLED = 0 << POSITION,
          XXIS_DOWN_LOW_INTERRUPT1_ENABLED = 1 << POSITION,
        };
      }  // namespace xlie_xdowne

      namespace xhie_xupe {
        enum {
          POSITION = 1,
          MASK = 1 << POSITION
        };
        enum States {
          XXIS_UP_HIGH_INTERRUPT1_DISABLED = 0 << POSITION,
          XXIS_UP_HIGH_INTERRUPT1_ENABLED = 1 << POSITION,
        };
      }  // namespace xhie_xupe

      namespace ylie_ydowne {
        enum {
          POSITION = 2,
          MASK = 1 << POSITION
        };
        enum States {
          YXIS_DOWN_LOW_INTERRUPT1_DISABLED = 0 << POSITION,
          YXIS_DOWN_LOW_INTERRUPT1_ENABLED = 1 << POSITION,
        };
      }  // namespace ylie_ydowne

      namespace yhie_yupe {
        enum {
          POSITION = 3,
          MASK = 1 << POSITION
        };
        enum States {
          YXIS_UP_HIGH_INTERRUPT1_DISABLED = 0 << POSITION,
          YXIS_UP_HIGH_INTERRUPT1_ENABLED = 1 << POSITION,
        };
      }  // namespace yhie_yupe

      namespace zlie_zdowne {
        enum {
          POSITION = 4,
          MASK = 1 << POSITION
        };
        enum States {
          ZXIS_DOWN_LOW_INTERRUPT1_DISABLED = 0 << POSITION,
          ZXIS_DOWN_LOW_INTERRUPT1_ENABLED = 1 << POSITION,
        };
      }  // namespace zlie_zdowne

      namespace zhie_zupe {
        enum {
          POSITION = 5,
          MASK = 1 << POSITION
        };
        enum States {
          ZXIS_UP_HIGH_INTERRUPT1_DISABLED = 0 << POSITION,
          ZXIS_UP_HIGH_INTERRUPT1_ENABLED = 1 << POSITION,
        };
      }  // namespace zhie_zupe

      namespace _6d {
        enum {
          POSITION = 6,
          MASK = 1 << POSITION
        };
        enum States {
          _6_DIRECTION_FUNCTION_DISABLED = 0 << POSITION,
          _6_DIRECTION_FUNCTION_ENABLED = 1 << POSITION,
        };
      }  // namespace _6d

      namespace aoi {
        enum {
          POSITION = 7,
          MASK = 1 << POSITION
        };
        enum States {
          _6_DIRECTION_FUNCTION_DISABLED = 0 << POSITION,
          _6_DIRECTION_FUNCTION_ENABLED = 1 << POSITION,
        };
      }  // namespace aoi
    }  // namespace int1_cfg

    namespace int1_src {
      enum {
        ADDRESS = 0x31
      };

      namespace xl {
        enum {
          POSITION = 0,
          MASK = 1 << POSITION
        };
        enum States {
          NO_INTERRUPT_OCCURRED = 0 << POSITION,
          X_LOW_INTERRUPT1_OCCURRED = 1 << POSITION,
        };
      }  // namespace xl

      namespace xh {
        enum {
          POSITION = 1,
          MASK = 1 << POSITION
        };
        enum States {
          NO_INTERRUPT_OCCURRED = 0 << POSITION,
          X_HIGH_INTERRUPT1_OCCURRED = 1 << POSITION,
        };
      }  // namespace xh

      namespace yl {
        enum {
          POSITION = 2,
          MASK = 1 << POSITION
        };
        enum States {
          NO_INTERRUPT_OCCURRED = 0 << POSITION,
          Y_LOW_INTERRUPT1_OCCURRED = 1 << POSITION,
        };
      }  // namespace yl

      namespace yh {
        enum {
          POSITION = 3,
          MASK = 1 << POSITION
        };
        enum States {
          NO_INTERRUPT_OCCURRED = 0 << POSITION,
          Y_HIGH_INTERRUPT1_OCCURRED = 1 << POSITION,
        };
      }  // namespace yh

      namespace zl {
        enum {
          POSITION = 4,
          MASK = 1 << POSITION
        };
        enum States {
          NO_INTERRUPT_OCCURRED = 0 << POSITION,
          Z_LOW_INTERRUPT1_OCCURRED = 1 << POSITION,
        };
      }  // namespace zl

      namespace zh {
        enum {
          POSITION = 5,
          MASK = 1 << POSITION
        };
        enum States {
          NO_INTERRUPT_OCCURRED = 0 << POSITION,
          Z_HIGH_INTERRUPT1_OCCURRED = 1 << POSITION,
        };
      }  // namespace zh

      namespace ia {
        enum {
          POSITION = 6,
          MASK = 1 << POSITION
        };
        enum States {
          NO_INTERRUPT_OCURRED = 0 << POSITION,
          ONE_OR_MORE_INTERRUPS1_OCCURRED = 1 << POSITION,
        };
      }  // namespace ia
    }  // namespace int1_src

    namespace int1_ths {
      enum {
        ADDRESS = 0x32
      };
    }  // namespace int1_ths

    namespace int1_duration {
      enum {
        ADDRESS = 0x33
      };
    }  // namespace int1_duration

    namespace int2_cfg {
      enum {
        ADDRESS = 0x34
      };

      namespace xlie {
        enum {
          POSITION = 0,
          MASK = 1 << POSITION
        };
        enum States {
          XXIS_LOW_INTERRUPT2_DISABLED = 0 << POSITION,
          XXIS_LOW_INTERRUPT2_ENABLED = 1 << POSITION,
        };
      }  // namespace xlie

      namespace xhie {
        enum {
          POSITION = 1,
          MASK = 1 << POSITION
        };
        enum States {
          XXIS_HIGH_INTERRUPT2_DISABLED = 0 << POSITION,
          XXIS_HIGH_INTERRUPT2_ENABLED = 1 << POSITION,
        };
      }  // namespace xhie

      namespace ylie {
        enum {
          POSITION = 2,
          MASK = 1 << POSITION
        };
        enum States {
          YXIS_LOW_INTERRUPT_DISABLED = 0 << POSITION,
          YXIS_LOW_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace ylie

      namespace yhie {
        enum {
          POSITION = 3,
          MASK = 1 << POSITION
        };
        enum States {
          YXIS_UP_HIGH_INTERRUPT_DISABLED = 0 << POSITION,
          YXIS_UP_HIGH_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace yhie

      namespace zlie {
        enum {
          POSITION = 4,
          MASK = 1 << POSITION
        };
        enum States {
          ZXIS_LOW_INTERRUPT_DISABLED = 0 << POSITION,
          ZXIS_LOW_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace zlie

      namespace zhie {
        enum {
          POSITION = 5,
          MASK = 1 << POSITION
        };
        enum States {
          ZXIS_HIGH_INTERRUPT_DISABLED = 0 << POSITION,
          ZXIS_HIGH_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace zhie

      namespace _6d {
        enum {
          POSITION = 6,
          MASK = 1 << POSITION
        };
        enum States {
          _6_DIRECTION_FUNCTION_DISABLED = 0 << POSITION,
          _6_DIRECTION_FUNCTION_ENABLED = 1 << POSITION,
        };
      }  // namespace _6d

      namespace aoi {
        enum {
          POSITION = 7,
          MASK = 1 << POSITION
        };
        enum States {
          AND_COMBINATION_OF_INTERRUPTS = 0 << POSITION,
          OR_COMBINATION_OF_INTERRUPTS = 1 << POSITION,
        };
      }  // namespace aoi
    }  // namespace int2_cfg

    namespace int2_src {
      enum {
        ADDRESS = 0x35
      };

      namespace xl {
        enum {
          POSITION = 0
        };
        enum {
          MASK = 1 << POSITION
        };
        namespace states {
          enum E {
            NO_INTERRUPT_OCCURRED = 0 << POSITION,
            X_LOW_INTERRUPT2_OCCURRED = 1 << POSITION,
          };
        }  // namespace states
      }  // namespace xl

      namespace xh {
        enum {
          POSITION = 1
        };
        enum {
          MASK = 1 << POSITION
        };
        namespace states {
          enum E {
            NO_INTERRUPT_OCCURRED = 0 << POSITION,
            X_HIGH_INTERRUPT2_OCCURRED = 1 << POSITION,
          };
        }  // namespace states
      }  // namespace xh

      namespace yl {
        enum {
          POSITION = 2
        };
        enum {
          MASK = 1 << POSITION
        };
        namespace states {
          enum E {
            NO_INTERRUPT_OCCURRED = 0 << POSITION,
            Y_LOW_INTERRUPT2_OCCURRED = 1 << POSITION,
          };
        }  // namespace states
      }  // namespace yl

      namespace yh {
        enum {
          POSITION = 3
        };
        enum {
          MASK = 1 << POSITION
        };
        namespace states {
          enum E {
            NO_INTERRUPT_OCCURRED = 0 << POSITION,
            Y_HIGH_INTERRUPT2_OCCURRED = 1 << POSITION,
          };
        }  // namespace states
      }  // namespace yh

      namespace zl {
        enum {
          POSITION = 4
        };
        enum {
          MASK = 1 << POSITION
        };
        namespace states {
          enum E {
            NO_INTERRUPT_OCCURRED = 0 << POSITION,
            Z_LOW_INTERRUPT2_OCCURRED = 1 << POSITION,
          };
        }  // namespace states
      }  // namespace zl

      namespace zh {
        enum {
          POSITION = 5
        };
        enum {
          MASK = 1 << POSITION
        };
        namespace states {
          enum E {
            NO_INTERRUPT_OCCURRED = 0 << POSITION,
            Z_HIGH_INTERRUPT2_OCCURRED = 1 << POSITION,
          };
        }  // namespace states
      }  // namespace zh

      namespace ia {
        enum {
          POSITION = 6
        };
        enum {
          MASK = 1 << POSITION
        };
        enum States {
          NO_INTERRUPT_OCURRED = 0 << POSITION,
          ONE_OR_MORE_INTERRUPS2_OCCURRED = 1 << POSITION,
        };
      }  // namespace ia
    }  // namespace int2_src

    namespace int2_ths {
      enum {
        ADDRESS = 0x36
      };
    }  // namespace int2_ths

    namespace int2_duration {
      enum {
        ADDRESS = 0x37
      };
    }  // namespace int2_duration

    namespace click_cfg {
      enum {
        ADDRESS = 0x38
      };

      namespace xs {
        enum {
          POSITION = 0,
          MASK = 1 << POSITION
        };
        enum States {
          XXIS_SINGLE_CLICK_INTERRUPT_DISABLED = 0 << POSITION,
          XXIS_SINGLE_CLICK_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace xs

      namespace xd {
        enum {
          POSITION = 1,
          MASK = 1 << POSITION
        };
        enum States {
          XXIS_DOUBLE_CLICK_INTERRUPT_DISABLED = 0 << POSITION,
          XXIS_DOUBLE_CLICK_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace xd

      namespace ys {
        enum {
          POSITION = 2,
          MASK = 1 << POSITION
        };
        enum States {
          YXIS_SINGLE_CLICK_INTERRUPT_DISABLED = 0 << POSITION,
          YXIS_SINGLE_CLICK_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace ys

      namespace yd {
        enum {
          POSITION = 3,
          MASK = 1 << POSITION
        };
        enum States {
          YXIS_DOUBLE_CLICK_INTERRUPT_DISABLED = 0 << POSITION,
          YXIS_DOUBLE_CLICK_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace yd

      namespace zs {
        enum {
          POSITION = 4,
          MASK = 1 << POSITION
        };
        enum States {
          ZXIS_SINGLE_CLICK_INTERRUPT_DISABLED = 0 << POSITION,
          ZXIS_SINGLE_CLICK_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace zs

      namespace zd {
        enum {
          POSITION = 5,
          MASK = 1 << POSITION
        };
        enum States {
          ZXIS_DOUBLE_CLICK_INTERRUPT_DISABLED = 0 << POSITION,
          ZXIS_DOUBLE_CLICK_INTERRUPT_ENABLED = 1 << POSITION,
        };
      }  // namespace zd
    }  // namespace click_cfg

    namespace click_src {
      enum {
        ADDRESS = 0x39
      };

      namespace x {
        enum {
          POSITION = 0,
          MASK = 1 << POSITION
        };
        enum States {
          NO_INTERRUPT_OCCURRED = 0 << POSITION,
          X_HIGH_INTERRUPT_OCCURRED = 1 << POSITION,
        };
      }  // namespace x

      namespace y {
        enum {
          POSITION = 1,
          MASK = 1 << POSITION
        };
        enum States {
          NO_INTERRUPT_OCCURRED = 0 << POSITION,
          Y_HIGH_INTERRUPT_OCCURRED = 1 << POSITION,
        };
      }  // namespace y

      namespace z {
        enum {
          POSITION = 2,
          MASK = 1 << POSITION
        };
        enum States {
          NO_INTERRUPT_OCCURRED = 0 << POSITION,
          Z_HIGH_INTERRUPT_OCCURRED = 1 << POSITION,
        };
      }  // namespace z

      namespace sgn {
        enum {
          POSITION = 3,
          MASK = 1 << POSITION
        };
        enum States {
          POSITIVE_DETECTION = 0 << POSITION,
          NEGATIVE_DETECTION = 1 << POSITION,
        };
      }  // namespace sgn

      namespace sclick {
        enum {
          POSITION = 4,
          MASK = 1 << POSITION
        };
        enum States {
          SINGLE_CLICK_DETECTION_DISABLED = 0 << POSITION,
          SINGLE_CLICK_DETECTION_ENABLED = 1 << POSITION,
        };
      }  // namespace sclick

      namespace dclick {
        enum {
          POSITION = 5,
          MASK = 1 << POSITION
        };
        enum States {
          DOUBLE_CLICK_DETECTION_DISABLED = 0 << POSITION,
          DOUBLE_CLICK_DETECTION_ENABLED = 1 << POSITION,
        };
      }  // namespace dclick

      namespace ia {
        enum {
          POSITION = 6,
          MASK = 1 << POSITION
        };
        enum States {
          NO_INTERRUPT_OCCURRED = 0 << POSITION,
          ONE_OR_MORE_INTERRUPT_OCCURRED = 1 << POSITION,
        };
      }  // namespace ia
    }  // namespace click_src

    namespace click_ths {
      enum {
        ADDRESS = 0x3A
      };
    }  // namespace click_ths

    namespace time_limit {
      enum {
        ADDRESS = 0x3B
      };
    }  // namespace time_limit

    namespace time_latency {
      enum {
        ADDRESS = 0x3C
      };
    }  // namespace time_latency

    namespace time_window {
      enum {
        ADDRESS = 0x3D
      };
    }  // namespace time_window
  } // namespace accelerometere

  namespace magnetometer {
    enum {
      ADDRESS = 0b0011110
    };

    namespace cra {
      enum {
        ADDRESS = 0x00
      };

      namespace do_ {
        enum {
          POSITION = 2,
          MASK = 0b111 << POSITION
        };
        enum States {
          _0_DOT_75_HZ = 0 << POSITION,
          _1_DOT_5_HZ = 1 << POSITION,
          _3_HZ = 2 << POSITION,
          _7_DOT_5_HZ = 3 << POSITION,
          _15_HZ = 4 << POSITION,
          _30_HZ = 5 << POSITION,
          _75_HZ = 6 << POSITION,
          _220_HZ = 7 << POSITION,
        };
      }  // namespace do_

      namespace temp_en {
        enum {
          POSITION = 7,
          MASK = 1 << POSITION
        };
        enum States {
          TEMPERATURE_SENSOR_DISABLED = 0 << POSITION,
          TEMPERATURE_SENSOR_ENABLED = 1 << POSITION,
        };
      }  // namespace temp_en
    }  // namespace cra

    namespace crb {
      enum {
        ADDRESS = 0x01
      };

      namespace gn {
        enum {
          POSITION = 5,
          MASK = 0b111 << POSITION
        };
        enum States {
          PLUS_MINUS_1_DOT_3_GAUSS = 1 << POSITION,
          PLUS_MINUS_1_DOT_9_GAUSS = 2 << POSITION,
          PLUS_MINUS_2_DOT_5_GAUSS = 3 << POSITION,
          PLUS_MINUS_4_GAUSS = 4 << POSITION,
          PLUS_MINUS_4_DOT_7_GAUSS = 5 << POSITION,
          PLUS_MINUS_5_DOT_6_GAUSS = 6 << POSITION,
          PLUS_MINUS_8_DOT_1_GAUSS = 7 << POSITION,
        };
      }  // namespace gn
    }  // namespace crb

    namespace mr {
      enum {
        ADDRESS = 0x02
      };

      namespace md {
        enum {
          POSITION = 0,
          MASK = 0b11 << POSITION
        };
        enum States {
          CONTINOUS_CONVERSION = 0 << POSITION,
          SINGLE_CONVERSION = 1 << POSITION,
          SLEEP_MODE = 2 << POSITION,
        };
      }  // namespace md
    }  // namespace mr

    namespace out_x_h {
      enum {
        ADDRESS = 0x03
      };
    }  // namespace out_x_h

    namespace out_x_l {
      enum {
        ADDRESS = 0x04
      };
    }  // namespace out_x_l

    namespace out_z_h {
      enum {
        ADDRESS = 0x05
      };
    } // namespace out_z_h

    namespace out_z_l {
      enum {
        ADDRESS = 0x06
      };
    }  // namespace out_z_l

    namespace out_y_h {
      enum {
        ADDRESS = 0x07
      };
    }  // namespace out_y_h

    namespace out_y_l {
      enum {
        ADDRESS = 0x08
      };
    }  // namespace out_y_l

    namespace sr {
      enum {
        ADDRESS = 0x09
      };

      namespace drdy {
        enum {
          POSITION = 0,
          MASK = 1 << POSITION
        };
        enum States {
          NO_NEW_MEASUREMENT_ISVAILABLE = 0 << POSITION,
          NEW_MEASUREMENT_IS_READY = 1 << POSITION,
        };
      }  // namespace drdy

      namespace lock {
        enum {
          POSITION = 1,
          MASK = 1 << POSITION
        };
        enum States {
          DATAISTER_UNLOCKED = 0 << POSITION,
          DATAISTER_LOCKED = 1 << POSITION,
        };
      }  // namespace lock
    }  // namespace sr

    namespace ira {
      enum {
        ADDRESS = 0x0A
      };
    }  // namespace ira

    namespace irb {
      enum {
        ADDRESS = 0x0B
      };
    }  // namespace irb

    namespace irc {
      enum {
        ADDRESS = 0x0C
      };
    }  // namespace irc

    namespace temp_out_h {
      enum {
        ADDRESS = 0x31
      };
    }  // namespace temp_out_h

    namespace temp_out_l {
      enum {
        ADDRESS = 0x32
      };
    }  // namespace temp_out_l
  }  // namespace magnetometer
}  // namespace lsm303dlhc
