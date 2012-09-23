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

#include "../include/peripheral/rcc.hpp"
#include "../include/core/nvic.hpp"

namespace dcmi {
  /**
   * @brief Enables the DCMI's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  void Functions::enableClock()
  {
    RCC::enableClocks<
        rcc::ahb2enr::DCMI
    >();
  }

  /**
   * @brief Disables the DCMI's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  void Functions::disableClock()
  {
    RCC::disableClocks<
        rcc::ahb2enr::DCMI
    >();
  }

  /**
   * @brief Unmasks all the DCMI interrupts.
   */
  void Functions::unmaskInterrupts()
  {
    NVIC::enableIrq<
        nvic::irqn::DCMI
    >();
  }

  /**
   * @brief Masks all the DCMI interrupts.
   */
  void Functions::maskInterrupts()
  {
    NVIC::disableIrq<
        nvic::irqn::DCMI
    >();
  }

  /**
   * @brief Enables the DCMI.
   */
  void Functions::enablePeripheral()
  {
    DCMI_REGS->CR |= cr::enable::DCMI_ENABLED;
  }

  /**
   * @brief Disables the DCMI.
   */
  void Functions::disablePeripheral()
  {
    DCMI_REGS->CR &= ~cr::enable::DCMI_ENABLED;
  }

  /**
   * @brief Stars the frame capture.
   */
  void Functions::startCapture()
  {
    DCMI_REGS->CR |= cr::capture::CAPTURE_ENABLED;
  }

  /**
   * @brief Stops the frame capture.
   */
  void Functions::stopCapture()
  {
    DCMI_REGS->CR &= ~cr::capture::CAPTURE_ENABLED;
  }

  /**
   * @brief Returns true if the DCMI is in line synchronization.
   */
  bool Functions::isInLineSynchronization()
  {
    return (DCMI_REGS->SR & sr::hsync::MASK);
  }

  /**
   * @brief Returns true if the DCMI is in frame synchronization.
   */
  bool Functions::isInFrameSynchronization()
  {
    return (DCMI_REGS->SR & sr::vsync::MASK);
  }

  /**
   * @brief Returns true if a buffer overrun error occurred.
   */
  bool Functions::hasBufferOverrunOccurred()
  {
    return (DCMI_REGS->RISR & risr::ovr::MASK);
  }

  /**
   * @brief Returns true if a synchronization error occurred.
   */
  bool Functions::hasErrorSynchronizationOccurred()
  {
    return (DCMI_REGS->RISR & risr::err::MASK);
  }

  /**
   * @brief Enables the capture complete interrupt.
   */
  void Functions::enableCaptureCompleInterrupt()
  {
    DCMI_REGS->IER |= ier::frame::
        CAPTURE_COMPLETE_INTERRUPT_ENABLED;
  }

  /**
   * @brief Disables the capture complete interrupt.
   */
  void Functions::disableCaptureCompleteInterrupt()
  {
    DCMI_REGS->IER &= ~ier::frame::
        CAPTURE_COMPLETE_INTERRUPT_ENABLED;
  }

  /**
   * @brief Clears the capture complete interrupt flag.
   */
  void Functions::clearCaptureCompleteFlag()
  {
    DCMI_REGS->ICR = icr::frame::
        CLEARS_CAPTURE_COMPLETE_INTERRUPT_FLAG;
  }

  /**
   * @brief Enables the buffer overrun interrupt.
   */
  void Functions::enableBufferOverrunInterrupt()
  {
    DCMI_REGS->IER |= ier::ovr::
        OVERRUN_ERROR_INTERRUPT_ENABLED;
  }

  /**
   * @brief Disables the buffer overrun interrupt.
   */
  void Functions::disableBufferOverrunInterrupt()
  {
    DCMI_REGS->IER &= ~ier::ovr::
        OVERRUN_ERROR_INTERRUPT_ENABLED;
  }

  /**
   * @brief Clears the buffer overrun interrupt flag.
   */
  void Functions::clearBufferOverrunFlag()
  {
    DCMI_REGS->ICR = icr::ovr::
        CLEARS_OVERRUN_ERROR_INTERRUPT_FLAG;
  }

  /**
   * @brief Enables the synchronization error interrupt.
   */
  void Functions::enableSynchronizationErrorInterrupt()
  {
    DCMI_REGS->IER |= ier::err::
        SYNCHRONIZATION_ERROR_INTERRUPT_ENABLED;
  }

  /**
   * @brief Disables the synchronization error interrupt.
   */
  void Functions::disableSynchronizationErrorInterrupt()
  {
    DCMI_REGS->IER &= ~ier::err::
        SYNCHRONIZATION_ERROR_INTERRUPT_ENABLED;
  }

  /**
   * @brief Clears the synchronization error interrupt flag.
   */
  void Functions::clearSynchronizationErrorFlag()
  {
    DCMI_REGS->ICR = icr::err::
        CLEARS_SYNCHRONIZATION_ERROR_INTERRUPT_FLAG;
  }

  /**
   * @brief Enables the vertical synchronization interrupt.
   */
  void Functions::enableVerticalSynchronizationInterrupt()
  {
    DCMI_REGS->IER |= ier::vsync::
        NEW_FRAME_SYNCHRONIZATION_INTERRUPT_ENABLED;
  }

  /**
   * @brief Disables the vertical synchronization interrupt.
   */
  void Functions::disableVerticalSynchronizationInterrupt()
  {
    DCMI_REGS->IER &= ~ier::vsync::
        NEW_FRAME_SYNCHRONIZATION_INTERRUPT_ENABLED;
  }

  /**
   * @brief Clears the vertical synchronization interrupt flag.
   */
  void Functions::clearVerticalSynchronizationFlag()
  {
    DCMI_REGS->ICR = icr::vsync::
        CLEARS_NEW_FRAME_SYNCHRONIZATION_INTERRUPT_FLAG;
  }

  /**
   * @brief Enables the line received interrupt.
   */
  void Functions::enableLineReceivedInterrupt()
  {
    DCMI_REGS->IER |= ier::line::
        NEW_LINE_RECEIVED_INTERRUPT_ENABLED;
  }

  /**
   * @brief Disables the line received interrupt.
   */
  void Functions::disableLineReceivedInterrupt()
  {
    DCMI_REGS->IER &= ~ier::line::
        NEW_LINE_RECEIVED_INTERRUPT_ENABLED;
  }

  /**
   * @brief Clears the line received interrupt flag.
   */
  void Functions::clearLineReceivedFlag()
  {
    DCMI_REGS->ICR = icr::line::
        CLEARS_NEW_LINE_RECEIVED_INTERRUPT_FLAG;
  }

  /**
   * @brief Clears all the interrupt flags.
   */
  void Functions::clearAllFlags()
  {
    DCMI_REGS->ICR = 0b11111;
  }

  /**
   * @brief Configures the DCMI.
   */
  void Functions::configure(
      cr::capture::States CAPTURE,
      cr::cm::States CM,
      cr::crop::States CROP,
      cr::jpeg::States JPEG,
      cr::ess::States ESS,
      cr::pckpol::States PCKPOL,
      cr::hspol::States HSPOL,
      cr::vspol::States VSPOL,
      cr::fcrc::States FCRC,
      cr::edm::States EDM,
      cr::enable::States ENABLE)
  {
    DCMI_REGS->CR = CAPTURE + CM + CROP + JPEG + ESS + PCKPOL + HSPOL + VSPOL
        + FCRC + EDM + ENABLE;
  }

  /**
   * @brief Configures the size of the cropped image.
   * @note  Image indices start at 0.
   */
  template<
      u32 Left,
      u32 Top,
      u32 Width,
      u32 Height,
      Format FORMAT
  >
  void Functions::setCropDimensions()
  {
    static_assert(
        (Left < 2048) && (Top < 2048) &&
        (Width >= 1) && (Width <= 2048) &&
        (Height >= 1) && (Height <= 2048),
        "Max image size is 2048 x 2048 pixels"
    );

    DCMI_REGS->CWSTRTR =
        ((Left * FORMAT) << cwstrtr::hoffcnt::POSITION) +
            (Top << cwstrtr::vst::POSITION);

    DCMI_REGS->CWSIZER =
        ((FORMAT * Width - 1) << cwsizer::capcnt::POSITION) +
            ((Height - 1) << cwsizer::vline::POSITION);
  }
}  // namespace dcmi
