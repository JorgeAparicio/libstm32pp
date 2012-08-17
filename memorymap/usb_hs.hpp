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

namespace usb_hs {
  struct Registers {
      struct {
          __RW
          u32 GOTGCTL;  // 0x000: Control and status
          __RW
          u32 GOTGINT;  // 0x004: Interrupt
          __RW
          u32 GAHBCFG;  // 0x008: AHB configuration
          __RW
          u32 GUSBCFG;  // 0x00C: USB configuration
          __RW
          u32 GRSTCTL;  // 0x010: Reset
          __RW
          u32 GINTSTS;  // 0x014: Core interrupt
          __RW
          u32 GINTMSK;  // 0x018: Interrupt mask
          __R
          u32 GRXSTSR;  // 0x01C: Receive status debug read
          __RW
          u32 GRXSTSP;  // 0x020: Status read and pop
          __RW
          u32 GRXFSIZ;  // 0x024: Receive FIFO size
          union {
              __RW
              u32 GNPTXFSIZ;  // 0x028: Non-periodic transmit FIFO size
              __RW
              u32 TX0FSIZ;  // 0x028: Endpoint 0 transmit FIFO size
          };
          __RW
          u32 GNPTXSTS;  // 0x02C: Non-periodic transmit FIFO/queue status
          __RW
          u32 GI2CCTL;  // 0x030: I2C access
          u32 _RESERVED0;
          __RW
          u32 GCCFG;  // 0x038: General core configuration
          __RW
          u32 CID;  // 0x03C: Core ID
          u32 _RESERVED1[48];
          __RW
          u32 HPTXFSIZ;  // 0x100: Host periodic transmit FIFO size
          __RW
          u32 DIEPTXF[7];  // 0x104 + 0x4 * I: IN endpoint transmit FIFO size
      } Global;
      u32 _RESERVED0[184];
      struct {
          __RW
          u32 HCFG;  // 0x400: Configuration
          __RW
          u32 HFIR;  // 0x404: Frame interval
          __RW
          u32 HFNUM;  // 0x408: Frame number/frame time remaining
          u32 _RESERVED0;
          __RW
          u32 HPTXSTS;  // 0x410: Periodic transmit FIFO/queue status
          __RW
          u32 HAINT;  // 0x414: All channels interrupt
          __RW
          u32 HAINTMSK;  // 0x418: All channels interrupt mask
          u32 _RESERVED1[9];
          __RW
          u32 HPRT;  // 0x440: Port control and status
          u32 _RESERVED2[47];
          struct {
              __RW
              u32 HCCHAR;  // 0x500 + 0x20 * I: Characteristics
              __RW
              u32 HCSPLT;  // 0x504 + 0x20 * I: Split control
              __RW
              u32 HCINT;  // 0x508 + 0x20 * I: Interrupt
              __RW
              u32 HCINTMSK;  // 0x50C + 0x20 * I: Interrupt mask
              __RW
              u32 HCTSIZ;  // 0x510 + 0x20 * I: Transfer size
              __RW
              u32 HCDMA;  // 0x514 + 0x20 * I: DMA address
              u32 _RESERVED[2];
          } Channel[12];
      } Host;
      u32 _RESERVED1[96];
      struct {
          __RW
          u32 DCFG;  // 0x800: Configuration
          __RW
          u32 DCTL;  // 0x804: Control
          __RW
          u32 DSTS;  // 0x808: Status
          u32 _RESERVED0;
          __RW
          u32 DIEPMSK;  // 0x810: IN endpoint common interrupt mask
          __RW
          u32 DOEPMSK;  // 0x814: OUT endpoint common interrupt mask
          __RW
          u32 DAINT;  // 0x818: All endpoints interrupt
          __RW
          u32 DAINTMSK;  // 0x81C: All endpoints interrupt mask
          u32 _RESERVED1[2];
          __RW
          u32 DVBUSDIS;  // 0x828: VBUS discharge time
          __RW
          u32 DVBUSPULSE;  // 0x82C: VBUS pulsing time
          __RW
          u32 DTHRCTL;  // 0x830: Threshold control
          __RW
          u32 DIEPEMPMSK;  // 0x834: IN endpoint FIFO empty interrupt mask
          __RW
          u32 DEACHINT;  // 0x838: Each endpoint interrupt
          __RW
          u32 DEACHINTMSK;  // 0x83C: Each endpoint interrupt mask
          __RW
          u32 DIEPEACHMSK1;  // 0x840: Each IN endpoint-1 interupt
          __RW
          u32 _RESERVED2[15];
          u32 DOEPACHMSK1;  // 0x880: Each OUT endpoint-1 interrupt
          u32 _RESERVED3[31];
          struct {
              __RW
              u32 DIEPCTL;  // 0x900 + 0x20 * I(0..7): Control
              u32 __RESERVED0;
              __RW
              u32 DIEPINT;  // 0x908 + 0x20 * I(0..7): Interrupt
              u32 __RESERVED1;
              __RW
              u32 DIEPTSIZ;  // 0x910 + 0x20 * I(0..3): Transfer size
              __RW
              u32 DIEPDMA;  // 0x914 + 0x20 * I(1..5): DMA address
              __RW
              u32 DTXFSTS;  // 0x918 + 0x20 * I(0..5): Transmit FIFO status
              u32 __RESERVED2;
          } InEndpoint[7];
          u32 _RESERVED4[72];
          struct {
              __RW
              u32 DOEPCTL;  // 0xB00 + 0x20 * I(0..3): Control
              u32 __RESERVED0;
              __RW
              u32 DOEPINT;  // 0xB08 + 0x20 * I(0..7): Interrupt
              u32 __RESERVED1;
              __RW
              u32 DOEPTSIZ;  // 0xB10 + 0x20 * I(0..5): Transfer size
              __RW
              u32 DOEPDMA;  // 0xB14 + 0x20 * I(1..5): DMA address
              u32 _RESERVED2[2];
          } OutEndpoint[7];
      } Device;
      u32 _RESERVED2[136];
      __RW
      u32 PCGCCTL;  // 0xE00: Power and clock gating
  };

  enum {
    ADDRESS = alias::AHB1 + 0x20000
  };

  namespace registers {
  // TODO USB_HS register bits
  }// namespace registers
}  // namespace usb_hs
