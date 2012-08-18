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

namespace cec {
  struct Registers {
      __RW
      u32 CFGR;  // 0x00: Configuration
      __RW
      u32 OAR;   // 0x04: Own address
      __RW
      u32 PRES;  // 0x08: Prescaler
      __RW
      u32 ESR;   // 0x0C: Error status
      __RW
      u32 CSR;   // 0x10: Control and status
      __RW
      u32 TXD;   // 0x14: Tx data
      __RW
      u32 RXD;   // 0x18: Rx data
  };

  enum {
    ADDRESS = alias::APB1 + 0x7800
  };

  namespace cfgr {
    enum {
      OFFSET = 0x00
    };
  }  // namespace cfgr

  namespace oar {
    enum {
      OFFSET = 0x04
    };
  }  // namespace oar

  namespace pres {
    enum {
      OFFSET = 0x08
    };
  }  // namespace pres

  namespace esr {
    enum {
      OFFSET = 0x0C
    };
  }  // namespace esr

  namespace csr {
    enum {
      OFFSET = 0x10
    };
  }  // namespace csr

  namespace txd {
    enum {
      OFFSET = 0x14
    };
  }  // namespace txd

  namespace rxd {
    enum {
      OFFSET = 0x18
    };
  }  // namespace rxd
}  // namespace cec
