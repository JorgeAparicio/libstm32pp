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

namespace eth {
  enum {
#ifdef STM32F1XX
    ADDRESS = alias::AHB + 0x8000
#else
    ADDRESS = alias::AHB1 + 0x8000
#endif
  };

  struct Registers {
#ifndef STM32F1XX
      __RW
      u32 MACCR; // 0x0000: MAC configuration
      __RW
      u32 MACFFR; // 0x0004: MAC frame filter
      __RW
      u32 MACHTHR; // 0x0008: MAC hash table high
      __RW
      u32 MACHTLR; // 0x000C: MAC hash table low
      __RW
      u32 MACMIIAR; // 0x0010: MAC MII address
      __RW
      u32 MACMIIDR; // 0x0014: MAC MII data
      __RW
      u32 MACFCR; // 0x0018: MAC flow control
      __RW
      u32 MACVLANTR; // 0x001C: MAC VLAN tag
      u32 _RESERVED0[2];
      __RW
      u32 MACRWUFFR; // 0x0028: MAC remote wakeup frame filter
      __RW
      u32 MACPMTCSR; // 0x002C: MAC PMT control and status
      u32 _RESERVED1;
      __RW
      u32 MACDBGR; // 0x0034: MAC debug
      __RW
      u32 MACSR; // 0x0038: MAC interrupt status
      __RW
      u32 MACIMR; // 0x003C: MAC interrupt mask
      __RW
      u32 MACA0HR; // 0x0040: MAC address 0 high
      __RW
      u32 MACA0LR; // 0x0044: MAC address 0 low
      __RW
      u32 MACA1HR; // 0x0048: MAC address 1 high
      __RW
      u32 MACA1LR; // 0x004C: MAC address 1 low
      __RW
      u32 MACA2HR; // 0x0050: MAC address 2 high
      __RW
      u32 MACA2LR; // 0x0054: MAC address 2 low
      __RW
      u32 MACA3HR; // 0x0058: MAC address 3 high
      __RW
      u32 MACA3LR; // 0x005C: MAC address 3 low
      u32 _RESERVED2[40];
      __RW
      u32 MMCCR; // 0x0100: MMC control
      __RW
      u32 MMCRIR; // 0x0104: MMC receive interrupt
      __RW
      u32 MMCTIR; // 0x0108: MMC transmit interrupt
      __RW
      u32 MMCRIMR; // 0x010C: MMC receive interrupt mask
      __RW
      u32 MMCTIMR; // 0x0110: MMC transmit interrupt mask
      u32 _RESERVED3[14];
      __RW
      u32 MMCTGFSCCR; // 0x014C: MMC transmitted good frames after a single collision counter
      __RW
      u32 MMCTGFMSCCR; // 0x0150: MMC transmitted good frames after more than a single collision counter
      u32 _RESERVED4[5];
      __RW
      u32 MMCTGFCR; // 0x0168: MMC transmitted good frames counter
      u32 _RESERVED5[10];
      __RW
      u32 MMCRFCECR; // 0x0194: MMC received frames with CRC error counter
      __RW
      u32 MMCRFAECR; // 0x0198: MMC received frames with alignment error counter
      u32 _RESERVED6[10];
      __RW
      u32 MMCRGUFCR; // 0x01C4: MMC received good unicast frames counter
      u32 _RESERVED7[334];
      __RW
      u32 PTPTSCR; // 0x0700: PTP time stamp control
      __RW
      u32 PTPSSIR; // 0x0704: PTP subsecond increment
      __RW
      u32 PTPTSHR; // 0x0708: PTP time stamp high
      __RW
      u32 PTPTSLR; // 0x070C: PTP time stamp low
      __RW
      u32 PTPTSHUR; // 0x0710: PTP time stamp high update
      __RW
      u32 PTPTSLUR; // 0x0714: PTP time stamp low update
      __RW
      u32 PTPTSAR; // 0x0718: PTP time stamp addend
      __RW
      u32 PTPTTHR; // 0x071C: PTP target time high
      __RW
      u32 PTPTTLR; // 0x0720: PTP target time low
      u32 _RESERVED8;
      __RW
      u32 PTPTSSR; // 0x0728: PTP time stamp status
      __RW
      u32 PTPPPSCR; // 0x072C: PTP PPS control
      u32 _RESERVED9[564];
      __RW
      u32 DMABMR; // 0x1000: DMA bus mode
      __RW
      u32 DMATPDR; // 0x1004: DMA transmit poll demand
      __RW
      u32 DMARPDR; // 0x1008: DMA receive poll demand
      __RW
      u32 DMARDLAR; // 0x100C: DMA receive descriptor list address
      __RW
      u32 DMATDLAR; // 0x1010: DMA transmit descriptor list address
      __RW
      u32 DMASR; // 0x1014: DMA status
      __RW
      u32 DMAOMR; // 0x1018: DMA operation mode
      __RW
      u32 DMAIER; // 0x101C: DMA interrupt enable
      __RW
      u32 DMAMFBOCR; // 0x1020: DMA missed frame and buffer overflow counter
      __RW
      u32 DMARSWTR; // 0x1024: DMA receive status watchdog timer
      u32 _RESERVED10[8];
      __RW
      u32 DMACHTDR; // 0x1048: DMA current host transmit descriptor
      __RW
      u32 DMACHRDR; // 0x104C: DMA current host receive descriptor
      __RW
      u32 DMACHTBAR; // 0x1050: DMA current host transmit buffer
      __RW
      u32 DMACHRBAR; // 0x1054: DMA current host receive buffer
#else
      __RW
      u32 MACCR; // 0x0000: MAC configuration
      __RW
      u32 MACFFR;// 0x0004: MAC frame filter
      __RW
      u32 MACHTHR;// 0x0008: MAC hash table high
      __RW
      u32 MACHTLR;// 0x000C: MAC hash table low
      __RW
      u32 MACMIIAR;// 0x0010: MAC MII address
      __RW
      u32 MACMIIDR;// 0x0014: MAC MII data
      __RW
      u32 MACFCR;// 0x0018: MAC flow control
      __RW
      u32 MACVLANTR;// 0x001C: MAC VLAN tag
      u32 _RESERVED0[2];
      __RW
      u32 MACRWUFFR;// 0x0028: MAC remote wakeup frame filter
      __RW
      u32 MACPMTCSR;// 0x002C: MAC PMT control and status
      u32 _RESERVED1[2];
      __RW
      u32 MACSR;// 0x0038: MAC interrupt status
      __RW
      u32 MACIMR;// 0x003C: MAC interrupt mask
      __RW
      u32 MACA0HR;// 0x0040: MAC address 0 high
      __RW
      u32 MACA0LR;// 0x0044: MAC address 0 low
      __RW
      u32 MACA1HR;// 0x0048: MAC address 1 high
      __RW
      u32 MACA1LR;// 0x004C: MAC address 1 low
      __RW
      u32 MACA2HR;// 0x0050: MAC address 2 high
      __RW
      u32 MACA2LR;// 0x0054: MAC address 2 low
      __RW
      u32 MACA3HR;// 0x0058: MAC address 3 high
      __RW
      u32 MACA3LR;// 0x005C: MAC address 3 low
      u32 _RESERVED2[40];
      __RW
      u32 MMCCR;// 0x0100: MMC control
      __RW
      u32 MMCRIR;// 0x0104: MMC receive interrupt
      __RW
      u32 MMCTIR;// 0x0108: MMC transmit interrupt
      __RW
      u32 MMCRIMR;// 0x010C: MMC receive interrupt mask
      __RW
      u32 MMCTIMR;// 0x0110: MMC transmit interrupt mask
      u32 _RESERVED3[14];
      __RW
      u32 MMCTGFSCCR;// 0x014C: MMC transmitted good frames after a single collision counter
      __RW
      u32 MMCTGFMSCCR;// 0x0150: MMC transmitted good frames after more than a single collision counter
      u32 _RESERVED4[5];
      __RW
      u32 MMCTGFCR;// 0x0168: MMC transmitted good frames counter
      u32 _RESERVED5[10];
      __RW
      u32 MMCRFCECR;// 0x0194: MMC received frames with CRC error counter
      __RW
      u32 MMCRFAECR;// 0x0198: MMC received frames with alignment error counter
      u32 _RESERVED6[10];
      __RW
      u32 MMCRGUFCR;// 0x01C4: MMC received good unicast frames counter
      u32 _RESERVED7[334];
      __RW
      u32 PTPTSCR;// 0x0700: PTP time stamp control
      __RW
      u32 PTPSSIR;// 0x0704: PTP subsecond increment
      __RW
      u32 PTPTSHR;// 0x0708: PTP time stamp high
      __RW
      u32 PTPTSLR;// 0x070C: PTP time stamp low
      __RW
      u32 PTPTSHUR;// 0x0710: PTP time stamp high update
      __RW
      u32 PTPTSLUR;// 0x0714: PTP time stamp low update
      __RW
      u32 PTPTSAR;// 0x0718: PTP time stamp addend
      __RW
      u32 PTPTTHR;// 0x071C: PTP target time high
      __RW
      u32 PTPTTLR;// 0x0720: PTP target time low
      u32 _RESERVED8[567];
      __RW
      u32 DMABMR;// 0x1000: DMA bus mode
      __RW
      u32 DMATPDR;// 0x1004: DMA transmit poll demand
      __RW
      u32 DMARPDR;// 0x1008: DMA receive poll demand
      __RW
      u32 DMARDLAR;// 0x100C: DMA receive descriptor list address
      __RW
      u32 DMATDLAR;// 0x1010: DMA transmit descriptor list address
      __RW
      u32 DMASR;// 0x1014: DMA status
      __RW
      u32 DMAOMR;// 0x1018: DMA operation mode
      __RW
      u32 DMAIER;// 0x101C: DMA interrupt enable
      __RW
      u32 DMAMFBOCR;// 0x1020: DMA missed frame and buffer overflow counter
      u32 _RESERVED9[9];
      __RW
      u32 DMACHTDR;// 0x1048: DMA current host transmit descriptor
      __RW
      u32 DMACHRDR;// 0x104C: DMA current host receive descriptor
      __RW
      u32 DMACHTBAR;// 0x1050: DMA current host transmit buffer
      __RW
      u32 DMACHRBAR;// 0x1054: DMA current host receive buffer
#endif
  };

// TODO ETH register bits
}// namespace eth
