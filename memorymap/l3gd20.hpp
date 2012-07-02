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

namespace l3gd20 {
  namespace address {
    enum E {
      GYRO1 = 0b1101010,
      GYRO2 = 0b1101011,
    };
  }  // namespace address

  namespace registers {
    namespace who_am_i {
      enum {
        ADDRESS = 0x0F
      };
    }  // namespace who_am_i

    namespace ctrl_reg1 {
      enum {
        ADDRESS = 0x20
      };

      namespace bits {
        namespace yen {
          enum {
            POSITION = 0
          };
          enum {
            MASK = 1 << POSITION
          };

          namespace states {
            enum E {
              Y_AXIS_DISABLED = 0 << POSITION,
              Y_AXIS_ENABLED = 1 << POSITION
            };
          }  // namespace states
        }  // namespace yen

        namespace xen {
          enum {
            POSITION = 1
          };
          enum {
            MASK = 1 << POSITION
          };

          namespace states {
            enum E {
              X_AXIS_DISABLED = 0 << POSITION,
              X_AXIS_ENABLED = 1 << POSITION
            };
          }  // namespace states
        }  // namespace xen

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
              Z_AXIS_ENABLED = 1 << POSITION
            };
          }  // namespace states
        }  // namespace zen

        namespace pd {
          enum {
            POSITION = 3
          };
          enum {
            MASK = 1 << POSITION
          };

          namespace states {
            enum E {
              POWER_DOWN = 0 << POSITION,
              NORMAL_MODE = 1 << POSITION
            };
          }  // namespace states
        }  // namespace pd

        namespace bw {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 0b11 << POSITION
          };
        }  // namespace bw

        namespace odr {
          enum {
            POSITION = 6
          };
          enum {
            MASK = 0b11 << POSITION
          };

          namespace states {
            enum E {
              DATA_RATE_95HZ = 0b00 << POSITION,
              DATA_RATE_190HZ = 0b01 << POSITION,
              DATA_RATE_380HZ = 0b10 << POSITION,
              DATA_RATE_760HZ = 0b11 << POSITION
            };
          }  // namespace states
        }  // namespace odr

        namespace bw_odr {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 0b1111 << POSITION
          };

          namespace states {
            enum E {
              DATA_RATE_95HZ_CUTOFF_12_DOT_5 = 0b0000 << POSITION,
              DATA_RATE_95HZ_CUTOFF_25 = 0b0001 << POSITION,
              DATA_RATE_190HZ_CUTOFF_12_DOT_5 = 0b0100 << POSITION,
              DATA_RATE_190HZ_CUTOFF_25 = 0b0101 << POSITION,
              DATA_RATE_190HZ_CUTOFF_50 = 0b0110 << POSITION,
              DATA_RATE_190HZ_CUTOFF_70 = 0b0111 << POSITION,
              DATA_RATE_380HZ_CUTOFF_20 = 0b1000 << POSITION,
              DATA_RATE_380HZ_CUTOFF_25 = 0b1001 << POSITION,
              DATA_RATE_380HZ_CUTOFF_50 = 0b1010 << POSITION,
              DATA_RATE_380HZ_CUTOFF_100 = 0b1011 << POSITION,
              DATA_RATE_760HZ_CUTOFF_30 = 0b1100 << POSITION,
              DATA_RATE_760HZ_CUTOFF_35 = 0b1101 << POSITION,
              DATA_RATE_760HZ_CUTOFF_50 = 0b1110 << POSITION,
              DATA_RATE_760HZ_CUTOFF_100 = 0b1111 << POSITION,
            };
          }  // namespace states
        }  // namespace bw_odr
      }  // namespace bits
    }  // namespace ctrl_reg1

    namespace ctrl_reg2 {
      enum {
        ADDRESS = 0x21
      };
    }  // namespace ctrl_reg2

    namespace ctrl_reg3 {
      enum {
        ADDRESS = 0x22
      };
    }  // namespace ctrl_reg3

    namespace ctrl_reg4 {
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
              SPI_3_WIRE_INTERFACE = 1 << POSITION
            };
          }  // namespace states
        }  // namespace sim

        namespace fs {
          enum {
            POSITION = 4
          };
          enum {
            MASK = 0b11 << POSITION
          };

          namespace states {
            enum E {
              SCALE_250_DPS = 0b00 << POSITION,
              SCALE_500_DPS = 0b01 << POSITION,
              SCALE_2000_DPS = 0b10 << POSITION
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
              LITTLE_ENDIAN_FORMAT = 0 << POSITION,
              BIG_ENDIAN_FORMAT = 1 << POSITION
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
              DATA_BLOCKED_UNTIL_READ = 1 << POSITION
            };
          }  // namespace states
        }  // namespace bdu
      }  // namespace bits
    }  // namespace ctrl_reg4

    namespace ctrl_reg5 {
      enum {
        ADDRESS = 0x24
      };
    }  // namespace ctrl_reg5

    namespace out_temp {
      enum {
        ADDRESS = 0x26
      };
    }  // namespace out_temp

    namespace status_reg {
      enum {
        ADDRESS = 0x27
      };
    }  // namespace status_reg

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

    namespace fifo_ctrl_reg {
      enum {
        ADDRESS = 0x2E
      };
    }  // namespace fifo_ctrl_reg

    namespace fifo_src_reg {
      enum {
        ADDRESS = 0x2F
      };
    }  // namespace fifo_src_reg

    namespace int1_cfg {
      enum {
        ADDRESS = 0x30
      };
    }  // namespace int1_cfg

    namespace int1_scr {
      enum {
        ADDRESS = 0x31
      };
    }  // namespace int1_scr

    namespace int1_tsh_xh {
      enum {
        ADDRESS = 0x32
      };
    }  // namespace int1_tsh_xl

    namespace int1_tsh_xl {
      enum {
        ADDRESS = 0x33
      };
    }  // namespace int1_tsh_xl

    namespace int1_tsh_yh {
      enum {
        ADDRESS = 0x34
      };
    }  // namespace int1_tsh_yh

    namespace int1_tsh_yl {
      enum {
        ADDRESS = 0x35
      };
    }  // namespace int1_tsh_yl

    namespace int1_tsh_zh {
      enum {
        ADDRESS = 0x36
      };
    }  // namespace int1_tsh_zh

    namespace int1_tsh_zl {
      enum {
        ADDRESS = 0x37
      };
    }  // namespace int1_tsh_zl

    namespace int1_duration {
      enum {
        ADDRESS = 0x38
      };
    }  // namespace int1_duration

  }  // namespace registers
}  // namespace l3gd20

