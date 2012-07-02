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
  namespace address {
    enum E {
      ACCELEROMETER = 0b0011001,
      MAGNETOMETER = 0b0011110,
    };
  }  // namespace address

  namespace registers {
    namespace ctrl_reg1_a {
      enum {
        ADDRESS = 0x20
      };

      namespace bits {
        namespace xen {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              X_AXIS_DISABLED = 0 << POSITION,
              X_AXIS_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace xen

        namespace yen {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              Y_AXIS_DISABLED = 0 << POSITION,
              Y_AXIS_ENABLED = 1 << POSITION,
            };
          }  // namespace states

        }  // namespace yen

        namespace zen {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              Z_AXIS_DISABLED = 0 << POSITION,
              Z_AXIS_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace zen

        namespace lpen {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NORMAL_MODE = 0 << POSITION,
              LOW_POWER_MODE = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace lpen

        namespace odr {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 0b1111 << POSITION
          };
          namespace states {
            enum E {
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
          }  // namespace states
        }  // namespace odr
      }  // namespace bits
    }  // namespace ctrl_reg1_a

    namespace ctrl_reg2_a {
      enum {
        ADDRESS = 0x21
      };

      namespace bits {
        namespace hpis1 {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              HIGH_PASS_FILTER_DISABLED_ON_INTERRUPT1 = 0 << POSITION,
              HIGH_PASS_FILTER_ENABLED_ON_INTERRUPT1 = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace hpis1

        namespace hpis2 {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              HIGH_PASS_FILTER_DISABLED_ON_INTERRUPT2 = 0 << POSITION,
              HIGH_PASS_FILTER_ENABLED_ON_INTERRUPT2 = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace hpis2

        namespace hpclick {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              HIGH_PASS_FILTER_DISABLED_ON_CLICK = 0 << POSITION,
              HIGH_PASS_FILTER_ENABLED_ON_CLICK = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace hpclick

        namespace fds {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              HIGH_PASS_FILTER_NOT_APPLIED_TO_DATA = 0 << POSITION,
              HIGH_PASS_FILTER_APPLIED_TO_DATA = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace fds

        namespace hpcf {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 0b11 << POSITION
          };
          namespace states {
            enum E {
              CUT_OFF_FREQUENCY_1 = 0 << POSITION,
              CUT_OFF_FREQUENCY_2 = 1 << POSITION,
              CUT_OFF_FREQUENCY_3 = 2 << POSITION,
              CUT_OFF_FREQUENCY_4 = 3 << POSITION,
            };
          }  // namespace states
        }  // namespace hpcf

        namespace hpm {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 0b11 << POSITION
          };
          namespace states {
            enum E {
              NORMAL_MODE_WITH_RESET = 0 << POSITION,
              FILTER_WITH_REFERENCE = 1 << POSITION,
              NORMAL_MODE = 2 << POSITION,
              AUTORESET_ON_INTERRUPT = 3 << POSITION,
            };
          }  // namespace states
        }  // namespace hpm
      }  // namespace bits
    }  // namespace ctrl_reg2_a

    namespace ctrl_reg3_a {
      enum {
        ADDRESS = 0x22
      };

      namespace bits {
        namespace i1_overrun {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              FIFO_OVERRUN_INTERRUPT1_DISABLED = 0 << POSITION,
              FIFO_OVERRUN_INTERRUPT1_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace i1_overrun

        namespace i1_wtm {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              FIFO_WATERMARK_INTERRUPT1_DISABLED = 0 << POSITION,
              FIFO_WATERMARK_INTERRUPT1_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace i1_wtm

        namespace i1_drdy2 {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DATA_READY2_INTERRUPT1_DISABLED = 0 << POSITION,
              DATA_READY2_INTERRUPT1_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace i1_drdy2

        namespace i1_drdy1 {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DATA_READY1_INTERRUPT1_DISABLED = 0 << POSITION,
              DATA_READY1_INTERRUPT1_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace i1_drdy1

        namespace i1_aoi2 {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              AOI2_INTERRUPT1_DISABLED = 0 << POSITION,
              AOI2_INTERRUPT1_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace i1_aoi2

        namespace i1_aoi1 {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              AOI1_INTERRUPT1_DISABLED = 0 << POSITION,
              AOI1_INTERRUPT1_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace i1_aoi1

        namespace i1_click {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              CLICK_INTERRUPT1_DISABLED = 0 << POSITION,
              CLICK_INTERRUPT1_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace i1_click
      }  // namespace bits
    }  // namespace ctrl_reg3_a

    namespace ctrl_reg4_a {
      enum {
        ADDRESS = 0x23
      };

      namespace bits {
        namespace sim {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SPI_4_WIRE_INTERFACE = 0 << POSITION,
              SPI_3_WIRE_INTERFACE = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace sim

        namespace hr {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              HIGH_RESOLUTION_DISABLED = 0 << POSITION,
              HIGH_RESOLUTION_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace hr

        namespace fs {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 0b11 << POSITION
          };
          namespace states {
            enum E {
              RANGE_PLUS_MINUS_2G = 0 << POSITION,
              RANGE_PLUS_MINUS_4G = 1 << POSITION,
              RANGE_PLUS_MINUS_8G = 2 << POSITION,
              RANGE_PLUS_MINUS_16G = 3 << POSITION,
            };
          }  // namespace states
        }  // namespace fs

        namespace ble {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              LITTLE_ENDIAN_DATA_FORMAT = 0 << POSITION,
              BIG_ENDIAN_DATA_FORMAT = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ble

        namespace bdu {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              CONTINUOUS_UPDATE = 0 << POSITION,
              DATA_BLOCKED_UNTIL_READ = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace bdu
      }  // namespace bits
    }  // namespace ctrl_reg4_a

    namespace ctrl_reg5_a {
      enum {
        ADDRESS = 0x24
      };

      namespace bits {
        namespace d4d_int2 {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              _4D_DETECTION_INTERRUPT2_DISABLED = 0 << POSITION,
              _4D_DETECTION_INTERRUPT2_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace d4d_int2

        namespace lir_int2 {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              LATCH_INTERRUPT2_DISABLED = 0 << POSITION,
              LATCH_INTERRUPT2_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace lir_int2

        namespace d4d_int1 {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              _4D_DETECTION_INTERRUPT1_DISABLED = 0 << POSITION,
              _4D_DETECTION_INTERRUPT1_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace d4d_int1

        namespace lir_int1 {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              LATCH_INTERRUPT1_DISABLED = 0 << POSITION,
              LATCH_INTERRUPT1_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace lir_int1

        namespace fifo_en {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              FIFO_DISABLED = 0 << POSITION,
              FIFO_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace fifo_en

        namespace boot {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DONT_REBOOT_MEMORY_CONTENT = 0 << POSITION,
              REBOOT_MEMORY_CONTENT = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace boot
      }  // namespace bits
    }  // namespace ctrl_reg5_a

    namespace ctrl_reg6_a {
      enum {
        ADDRESS = 0x25
      };

      namespace bits {
        namespace h_lactive {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              INTERRUPT_ACTIVE_HIGH = 0 << POSITION,
              INTERRUPT_ACTIVE_LOW = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace h_lactive
      }  // namespace bits
    }  // namespace ctrl_reg6_a

    namespace reference_a {
      enum {
        ADDRESS = 0x26
      };
    }  // namespace reference_a

    namespace status_reg_a {
      enum {
        ADDRESS = 0x27
      };

      namespace bits {
        namespace xda {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              X_AXIS_NEW_DATA_NOT_AVAILABLE = 0 << POSITION,
              X_AXIS_NEW_DATA_AVAILABLE = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace xda

        namespace yda {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              Y_AXIS_NEW_DATA_NOT_AVAILABLE = 0 << POSITION,
              Y_AXIS_NEW_DATA_AVAILABLE = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace yda

        namespace zda {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              Z_AXIS_NEW_DATA_NOT_AVAILABLE = 0 << POSITION,
              Z_AXIS_NEW_DATA_AVAILABLE = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace zda

        namespace zyxda {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              XYZ_AXIS_NEW_DATA_NOT_AVAILABLE = 0 << POSITION,
              XYZ_AXIS_NEW_DATA_AVAILABLE = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace zyxda

        namespace xor_ {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_OVERRUN = 0 << POSITION,
              X_AXIS_DATA_OVERRUN = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace xor_

        namespace yor {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_OVERRUN = 0 << POSITION,
              Y_AXIS_DATA_OVERRUN = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace yor

        namespace zor {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_OVERRUN = 0 << POSITION,
              Z_AXIS_DATA_OVERRUN = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace zor

        namespace zyxor {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_OVERRUN = 0 << POSITION,
              XYZ_AXIS_DATA_OVERRUN = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace zyxor
      }  // namespace bits
    }  // namespace status_reg_a

    namespace out_x_l_a {
      enum {
        ADDRESS = 0x28
      };
    }  // namespace out_x_l_a

    namespace out_x_h_a {
      enum {
        ADDRESS = 0x29
      };
    }  // namespace out_x_h_a

    namespace out_y_l_a {
      enum {
        ADDRESS = 0x2A
      };
    }  // namespace out_y_l_a

    namespace out_y_h_a {
      enum {
        ADDRESS = 0x2B
      };
    }  // namespace out_y_h_a

    namespace out_z_l_a {
      enum {
        ADDRESS = 0x2C
      };
    }  // namespace out_z_l_a

    namespace out_z_h_a {
      enum {
        ADDRESS = 0x2D
      };
    }  // namespace out_z_h_a

    namespace fifo_ctrl_reg_a {
      enum {
        ADDRESS = 0x2E
      };

      namespace bits {
        namespace tr {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              TRIGGER_EVENT_LINKED_TO_INTERRUPT1 = 0 << POSITION,
              TRIGGER_EVENT_LINKED_TO_INTERRUPT2 = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace tr

        namespace fm {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 0b11 << POSITION
          };
          namespace states {
            enum E {
              BYPASS_MODE = 0 << POSITION,
              FIFO_MODE = 1 << POSITION,
              STREAM_MODE = 2 << POSITION,
              TRIGGER_MODE = 3 << POSITION,
            };
          }  // namespace states
        }  // namespace fm
      }  // namespace bits
    }  // namespace fifo_ctrl_reg_a

    namespace fifo_src_reg_a {
      enum {
        ADDRESS = 0x2F
      };
    }  // namespace fifo_src_reg_a

    namespace int1_cfg_a {
      enum {
        ADDRESS = 0x30
      };

      namespace bits {
        namespace xlie_xdowne {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              X_AXIS_DOWN_LOW_INTERRUPT1_DISABLED = 0 << POSITION,
              X_AXIS_DOWN_LOW_INTERRUPT1_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace xlie_xdowne

        namespace xhie_xupe {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              X_AXIS_UP_HIGH_INTERRUPT1_DISABLED = 0 << POSITION,
              X_AXIS_UP_HIGH_INTERRUPT1_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace xhie_xupe

        namespace ylie_ydowne {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              Y_AXIS_DOWN_LOW_INTERRUPT1_DISABLED = 0 << POSITION,
              Y_AXIS_DOWN_LOW_INTERRUPT1_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ylie_ydowne

        namespace yhie_yupe {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              Y_AXIS_UP_HIGH_INTERRUPT1_DISABLED = 0 << POSITION,
              Y_AXIS_UP_HIGH_INTERRUPT1_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace yhie_yupe

        namespace zlie_zdowne {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              Z_AXIS_DOWN_LOW_INTERRUPT1_DISABLED = 0 << POSITION,
              Z_AXIS_DOWN_LOW_INTERRUPT1_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace zlie_zdowne

        namespace zhie_zupe {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              Z_AXIS_UP_HIGH_INTERRUPT1_DISABLED = 0 << POSITION,
              Z_AXIS_UP_HIGH_INTERRUPT1_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace zhie_zupe

        namespace _6d {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              _6_DIRECTION_FUNCTION_DISABLED = 0 << POSITION,
              _6_DIRECTION_FUNCTION_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace _6d

        namespace aoi {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              _6_DIRECTION_FUNCTION_DISABLED = 0 << POSITION,
              _6_DIRECTION_FUNCTION_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace aoi
      }  // namespace bits
    }  // namespace int1_cfg_a

    namespace int1_src_a {
      enum {
        ADDRESS = 0x31
      };

      namespace bits {
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
              X_LOW_INTERRUPT1_OCCURRED = 1 << POSITION,
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
              X_HIGH_INTERRUPT1_OCCURRED = 1 << POSITION,
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
              Y_LOW_INTERRUPT1_OCCURRED = 1 << POSITION,
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
              Y_HIGH_INTERRUPT1_OCCURRED = 1 << POSITION,
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
              Z_LOW_INTERRUPT1_OCCURRED = 1 << POSITION,
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
              Z_HIGH_INTERRUPT1_OCCURRED = 1 << POSITION,
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
          namespace states {
            enum E {
              NO_INTERRUPT_OCURRED = 0 << POSITION,
              ONE_OR_MORE_INTERRUPS1_OCCURRED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ia
      }  // namespace bits
    }  // namespace int1_src_a

    namespace int1_ths_a {
      enum {
        ADDRESS = 0x32
      };
    }  // namespace int1_ths_a

    namespace int1_duration_a {
      enum {
        ADDRESS = 0x33
      };
    }  // namespace int1_duration_a

    namespace int2_cfg_a {
      namespace bits {
        enum {
          ADDRESS = 0x34
        };

        namespace xlie {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              X_AXIS_LOW_INTERRUPT2_DISABLED = 0 << POSITION,
              X_AXIS_LOW_INTERRUPT2_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace xlie

        namespace xhie {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              X_AXIS_HIGH_INTERRUPT2_DISABLED = 0 << POSITION,
              X_AXIS_HIGH_INTERRUPT2_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace xhie

        namespace ylie {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              Y_AXIS_LOW_INTERRUPT_DISABLED = 0 << POSITION,
              Y_AXIS_LOW_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ylie

        namespace yhie {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              Y_AXIS_UP_HIGH_INTERRUPT_DISABLED = 0 << POSITION,
              Y_AXIS_UP_HIGH_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace yhie

        namespace zlie {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              Z_AXIS_LOW_INTERRUPT_DISABLED = 0 << POSITION,
              Z_AXIS_LOW_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace zlie

        namespace zhie {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              Z_AXIS_HIGH_INTERRUPT_DISABLED = 0 << POSITION,
              Z_AXIS_HIGH_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace zhie

        namespace _6d {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              _6_DIRECTION_FUNCTION_DISABLED = 0 << POSITION,
              _6_DIRECTION_FUNCTION_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace _6d

        namespace aoi {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              AND_COMBINATION_OF_INTERRUPTS = 0 << POSITION,
              OR_COMBINATION_OF_INTERRUPTS = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace aoi
      }  // namespace bits
    }  // namespace int2_cfg_a

    namespace int2_src_a {
      enum {
        ADDRESS = 0x35
      };

      namespace bits {
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
          namespace states {
            enum E {
              NO_INTERRUPT_OCURRED = 0 << POSITION,
              ONE_OR_MORE_INTERRUPS2_OCCURRED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ia
      }  // namespace bits
    }  // namespace int2_src_a

    namespace int2_ths_a {
      enum {
        ADDRESS = 0x36
      };
    }  // namespace int2_ths_a

    namespace int2_duration_a {
      enum {
        ADDRESS = 0x37
      };
    }  // namespace int2_duration_a

    namespace click_cfg_a {
      enum {
        ADDRESS = 0x38
      };

      namespace bits {
        namespace xs {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              X_AXIS_SINGLE_CLICK_INTERRUPT_DISABLED = 0 << POSITION,
              X_AXIS_SINGLE_CLICK_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace xs

        namespace xd {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              X_AXIS_DOUBLE_CLICK_INTERRUPT_DISABLED = 0 << POSITION,
              X_AXIS_DOUBLE_CLICK_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace xd

        namespace ys {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              Y_AXIS_SINGLE_CLICK_INTERRUPT_DISABLED = 0 << POSITION,
              Y_AXIS_SINGLE_CLICK_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ys

        namespace yd {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              Y_AXIS_DOUBLE_CLICK_INTERRUPT_DISABLED = 0 << POSITION,
              Y_AXIS_DOUBLE_CLICK_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace yd

        namespace zs {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              Z_AXIS_SINGLE_CLICK_INTERRUPT_DISABLED = 0 << POSITION,
              Z_AXIS_SINGLE_CLICK_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace zs

        namespace zd {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              Z_AXIS_DOUBLE_CLICK_INTERRUPT_DISABLED = 0 << POSITION,
              Z_AXIS_DOUBLE_CLICK_INTERRUPT_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace zd
      }  // namespace bits
    }  // namespace click_cfg_a

    namespace click_src_a {
      enum {
        ADDRESS = 0x39
      };

      namespace bits {
        namespace x {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_INTERRUPT_OCCURRED = 0 << POSITION,
              X_HIGH_INTERRUPT_OCCURRED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace x

        namespace y {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_INTERRUPT_OCCURRED = 0 << POSITION,
              Y_HIGH_INTERRUPT_OCCURRED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace y

        namespace z {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_INTERRUPT_OCCURRED = 0 << POSITION,
              Z_HIGH_INTERRUPT_OCCURRED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace z

        namespace sgn {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              POSITIVE_DETECTION = 0 << POSITION,
              NEGATIVE_DETECTION = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace sgn

        namespace sclick {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              SINGLE_CLICK_DETECTION_DISABLED = 0 << POSITION,
              SINGLE_CLICK_DETECTION_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace sclick

        namespace dclick {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DOUBLE_CLICK_DETECTION_DISABLED = 0 << POSITION,
              DOUBLE_CLICK_DETECTION_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace dclick

        namespace ia {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              NO_INTERRUPT_OCCURRED = 0 << POSITION,
              ONE_OR_MORE_INTERRUPT_OCCURRED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace ia
      }  // namespace bits
    }  // namespace click_src_a

    namespace click_ths_a {
      enum {
        ADDRESS = 0x3A
      };
    }  // namespace click_ths_a

    namespace time_limit_a {
      enum {
        ADDRESS = 0x3B
      };
    }  // namespace time_limit_a

    namespace time_latency_a {
      enum {
        ADDRESS = 0x3C
      };
    }  // namespace time_latency_a

    namespace time_window_a {
      enum {
        ADDRESS = 0x3D
      };
    }  // namespace time_window_a

    namespace cra_reg_m {
      enum {
        ADDRESS = 0x00
      };

      namespace bits {
        namespace do_ {
          enum {
            POSITION = 2
          };
          enum {
            MASK = 0b111 << POSITION
          };
          namespace states {
            enum E {
              _0_DOT_75_HZ = 0 << POSITION,
              _1_DOT_5_HZ = 1 << POSITION,
              _3_HZ = 2 << POSITION,
              _7_DOT_5_HZ = 3 << POSITION,
              _15_HZ = 4 << POSITION,
              _30_HZ = 5 << POSITION,
              _75_HZ = 6 << POSITION,
              _220_HZ = 7 << POSITION,
            };
          }  // namespace states
        }  // namespace do_

        namespace temp_en {
          enum {
            POSITION = 7
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              TEMPERATURE_SENSOR_DISABLED = 0 << POSITION,
              TEMPERATURE_SENSOR_ENABLED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace temp_en
      }  // namespace bits
    }  // namespace cra_reg_m

    namespace crb_reg_m {
      enum {
        ADDRESS = 0x01
      };

      namespace bits {
        namespace gn {
          enum {
            POSITION = 5
          };
          enum {
            MASK = 0b111 << POSITION
          };
          namespace states {
            enum E {
              PLUS_MINUS_1_DOT_3_GAUSS = 1 << POSITION,
              PLUS_MINUS_1_DOT_9_GAUSS = 2 << POSITION,
              PLUS_MINUS_2_DOT_5_GAUSS = 3 << POSITION,
              PLUS_MINUS_4_GAUSS = 4 << POSITION,
              PLUS_MINUS_4_DOT_7_GAUSS = 5 << POSITION,
              PLUS_MINUS_5_DOT_6_GAUSS = 6 << POSITION,
              PLUS_MINUS_8_DOT_1_GAUSS = 7 << POSITION,
            };
          }  // namespace states
        }  // namespace gn
      }  // namespace bits
    }  // namespace crb_reg_m

    namespace mr_reg_m {
      enum {
        ADDRESS = 0x02
      };

      namespace bits {
        namespace md {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 0b11 << POSITION
          };
          namespace states {
            enum E {
              CONTINOUS_CONVERSION = 0 << POSITION,
              SINGLE_CONVERSION = 1 << POSITION,
              SLEEP_MODE = 2 << POSITION,
            };
          }  // namespace states
        }  // namespace md
      }  // namespace bits
    }  // namespace mr_reg_m

    namespace out_x_h_m {
      enum {
        ADDRESS = 0x03
      };
    }  // namespace out_x_h_m

    namespace out_x_l_m {
      enum {
        ADDRESS = 0x04
      };
    }  // namespace out_x_l_m

    namespace out_z_h_m {
      enum {
        ADDRESS = 0x05
      };
    }

    namespace out_z_l_m {
      enum {
        ADDRESS = 0x06
      };
    }  // namespace out_z_l_m

    namespace out_y_h_m {
      enum {
        ADDRESS = 0x07
      };
    }  // namespace out_y_h_m

    namespace out_y_l_m {
      enum {
        ADDRESS = 0x08
      };
    }  // namespace out_y_l_m

    namespace sr_reg_mg {
      enum {
        ADDRESS = 0x09
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
              NO_NEW_MEASUREMENT_IS_AVAILABLE = 0 << POSITION,
              NEW_MEASUREMENT_IS_READY = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace drdy

        namespace lock {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };
          namespace states {
            enum E {
              DATA_REGISTER_UNLOCKED = 0 << POSITION,
              DATA_REGISTER_LOCKED = 1 << POSITION,
            };
          }  // namespace states
        }  // namespace lock
      }  // namespace bits
    }  // namespace sr_reg_mg

    namespace ira_reg_mg {
      enum {
        ADDRESS = 0x0A
      };
    }  // namespace ira_reg_mg

    namespace irb_reg_mg {
      enum {
        ADDRESS = 0x0B
      };
    }  // namespace irb_reg_mg

    namespace irc_reg_mg {
      enum {
        ADDRESS = 0x0C
      };
    }  // namespace irc_reg_mg

    namespace temp_out_h_m {
      enum {
        ADDRESS = 0x31
      };
    }  // namespace temp_out_h_m

    namespace temp_out_l_m {
      enum {
        ADDRESS = 0x32
      };
    }  // namespace temp_out_l_m
  }  // namespace registers
}  // namespace lsm303dlhc

