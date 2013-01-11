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

namespace fsmc {
  enum {
    ADDRESS = alias::FSMC
  };

  struct Registers {
      struct { // SRAM/NOR-Flash chip-select register
          __RW
          u32 BCR; // control
          __RW
          u32 BTR; // timing
      } NorPsram[4];
      u32 _RESERVED0[20]; // 0x20 - 0x5C
      struct {
          __RW
          u32 PCR; // 0x60: PC Card/NAND Flash control register 2
          __RW
          u32 SR; // 0x64: FIFO status and interrupt register 2
          __RW
          u32 PMEM; // 0x68: Common memory space timing register 2
          __RW
          u32 PATT; // 0x6C: Attribute memory space timing register 2
          u32 _RESERVED1;
          __RW
          u32 ECCR; // 0x74: ECC result register 2
          u32 _RESERVED2[2];
      } Nand[2];
      struct {
          __RW
          u32 PCR; // 0xA0: PC Card/NAND Flash control register 4
          __RW
          u32 SR; // 0xA4: FIFO status and interrupt register 4
          __RW
          u32 PMEM; // 0xA8: Common memory space timing register 4
          __RW
          u32 PATT; // 0xAC: Attribute memory space timing register 4
          __RW
          u32 PIO; // 0xB0: I/O space timing register 4
      } Pccard;
      u32 _RESERVED3[19];
      struct {
        __RW
        u32 BWTR; // 0x104: SRAM/NOR-Flash write timing register 1
        u32 _RESERVED4;
      } NorPsramTiming[4];
  };

// TODO FSMC register bits
}// namespace fsmc
