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

namespace dcmi {
  /**
   * @brief Enables the DCMI's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  void Functions::enableClock()
  {
    RCC::enableClocks<
      rcc::registers::ahb2enr::bits::DCMI
    >();
  }

  /**
   * @brief Disables the DCMI's clock.
   * @note  Registers can't be written when the clock is disabled.
   */
  void Functions::disableClock()
  {
    RCC::disableClocks<
      rcc::registers::ahb2enr::bits::DCMI
    >();
  }

  /**
   * @brief Enables the DCMI.
   */
  void Functions::enablePeripheral()
  {
    _DCMI->CR |= registers::cr::bits::enable::states::DCMI_ENABLED;
  }

  /**
   * @brief Disables the DCMI.
   */
  void Functions::disablePeripheral()
  {
    _DCMI->CR &= ~registers::cr::bits::enable::states::DCMI_ENABLED;
  }

  /**
   * @brief Stars the frame capture.
   */
  void Functions::startCapture()
  {
    _DCMI->CR |= registers::cr::bits::capture::states::CAPTURE_ENABLED;
  }

  /**
   * @brief Stops the frame capture.
   */
  void Functions::stopCapture()
  {
    _DCMI->CR &= ~registers::cr::bits::capture::states::CAPTURE_ENABLED;
  }

  /**
   * @brief Returns true if the DCMI is in line synchronization.
   */
  bool Functions::isInLineSynchronization()
  {
    return (_DCMI->SR & registers::sr::bits::hsync::MASK);
  }

  /**
   * @brief Returns true if the DCMI is in frame synchronization.
   */
  bool Functions::isInFrameSynchronization()
  {
    return (_DCMI->SR & registers::sr::bits::vsync::MASK);
  }

  /**
   * @brief Returns true if a buffer overrun error occurred.
   */
  bool Functions::hasBufferOverrunOccurred()
  {
    return (_DCMI->RISR & registers::risr::bits::ovr::MASK);
  }

  /**
   * @brief Returns true if a synchronization error occurred.
   */
  bool Functions::hasErrorSynchronizationOccurred()
  {
    return (_DCMI->RISR & registers::risr::bits::err::MASK);
  }

  /**
   * @brief Enables the capture complete interrupt.
   */
  void Functions::enableCaptureCompleInterrupt()
  {
    _DCMI->IER |= registers::ier::bits::frame::states::
        CAPTURE_COMPLETE_INTERRUPT_ENABLED;
  }

  /**
   * @brief Disables the capture complete interrupt.
   */
  void Functions::disableCaptureCompleteInterrupt()
  {
    _DCMI->IER &= ~registers::ier::bits::frame::states::
        CAPTURE_COMPLETE_INTERRUPT_ENABLED;
  }

  /**
   * @brief Clears the capture complete interrupt flag.
   */
  void Functions::clearCaptureCompleteFlag()
  {
    _DCMI->ICR = registers::icr::bits::frame::states::
        CLEARS_CAPTURE_COMPLETE_INTERRUPT_FLAG;
  }

  /**
   * @brief Enables the buffer overrun interrupt.
   */
  void Functions::enableBufferOverrunInterrupt()
  {
    _DCMI->IER |= registers::ier::bits::ovr::states::
        OVERRUN_ERROR_INTERRUPT_ENABLED;
  }

  /**
   * @brief Disables the buffer overrun interrupt.
   */
  void Functions::disableBufferOverrunInterrupt()
  {
    _DCMI->IER &= ~registers::ier::bits::ovr::states::
        OVERRUN_ERROR_INTERRUPT_ENABLED;
  }

  /**
   * @brief Clears the buffer overrun interrupt flag.
   */
  void Functions::clearBufferOverrunFlag()
  {
    _DCMI->ICR = registers::icr::bits::ovr::states::
        CLEARS_OVERRUN_ERROR_INTERRUPT_FLAG;
  }

  /**
   * @brief Enables the synchronization error interrupt.
   */
  void Functions::enableSynchronizationErrorInterrupt()
  {
    _DCMI->IER |= registers::ier::bits::err::states::
        SYNCHRONIZATION_ERROR_INTERRUPT_ENABLED;
  }

  /**
   * @brief Disables the synchronization error interrupt.
   */
  void Functions::disableSynchronizationErrorInterrupt()
  {
    _DCMI->IER &= ~registers::ier::bits::err::states::
        SYNCHRONIZATION_ERROR_INTERRUPT_ENABLED;
  }

  /**
   * @brief Clears the synchronization error interrupt flag.
   */
  void Functions::clearSynchronizationErrorFlag()
  {
    _DCMI->ICR = registers::icr::bits::err::states::
        CLEARS_SYNCHRONIZATION_ERROR_INTERRUPT_FLAG;
  }

  /**
   * @brief Enables the vertical synchronization interrupt.
   */
  void Functions::enableVerticalSynchronizationInterrupt()
  {
    _DCMI->IER |= registers::ier::bits::vsync::states::
        NEW_FRAME_SYNCHRONIZATION_INTERRUPT_ENABLED;
  }

  /**
   * @brief Disables the vertical synchronization interrupt.
   */
  void Functions::disableVerticalSynchronizationInterrupt()
  {
    _DCMI->IER &= ~registers::ier::bits::vsync::states::
        NEW_FRAME_SYNCHRONIZATION_INTERRUPT_ENABLED;
  }

  /**
   * @brief Clears the vertical synchronization interrupt flag.
   */
  void Functions::clearVerticalSynchronizationFlag()
  {
    _DCMI->ICR = registers::icr::bits::vsync::states::
        CLEARS_NEW_FRAME_SYNCHRONIZATION_INTERRUPT_FLAG;
  }

  /**
   * @brief Enables the line received interrupt.
   */
  void Functions::enableLineReceivedInterrupt()
  {
    _DCMI->IER |= registers::ier::bits::line::states::
        NEW_LINE_RECEIVED_INTERRUPT_ENABLED;
  }

  /**
   * @brief Disables the line received interrupt.
   */
  void Functions::disableLineReceivedInterrupt()
  {
    _DCMI->IER &= ~registers::ier::bits::line::states::
        NEW_LINE_RECEIVED_INTERRUPT_ENABLED;
  }

  /**
   * @brief Clears the line received interrupt flag.
   */
  void Functions::clearLineReceivedFlag()
  {
    _DCMI->ICR = registers::icr::bits::line::states::
        CLEARS_NEW_LINE_RECEIVED_INTERRUPT_FLAG;
  }

  /**
   * @brief Clears all the interrupt flags.
   */
  void Functions::clearAllFlags()
  {
    _DCMI->ICR = 0b11111;
  }

  /**
   * @brief Configures the DCMI.
   */
  template<
      registers::cr::bits::capture::states::E CAPTURE,
      registers::cr::bits::cm::states::E CM,
      registers::cr::bits::crop::states::E CROP,
      registers::cr::bits::jpeg::states::E JPEG,
      registers::cr::bits::ess::states::E ESS,
      registers::cr::bits::pckpol::states::E PCKPOL,
      registers::cr::bits::hspol::states::E HSPOL,
      registers::cr::bits::vspol::states::E VSPOL,
      registers::cr::bits::fcrc::states::E FCRC,
      registers::cr::bits::edm::states::E EDM,
      registers::cr::bits::enable::states::E ENABLE
  >
  void Functions::configure()
  {
    _DCMI->CR = CAPTURE + CM + CROP + JPEG + ESS + PCKPOL + HSPOL + VSPOL + FCRC
        + EDM + ENABLE;
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
      format::E FORMAT
  >
  void Functions::setCropDimensions()
  {
    static_assert(
        (Left < 2048) && (Top < 2048) &&
        (Width >= 1) && (Width <= 2048) &&
        (Height >= 1) && (Height <= 2048),
        "Max image size is 2048 x 2048 pixels"
    );

    _DCMI->CWSTRTR =
        ((Left * FORMAT) << registers::cwstrtr::bits::hoffcnt::POSITION) +
            (Top << registers::cwstrtr::bits::vst::POSITION);

    _DCMI->CWSIZER =
        ((FORMAT * Width - 1) << registers::cwsizer::bits::capcnt::POSITION) +
            ((Height - 1) << registers::cwsizer::bits::vline::POSITION);
  }
}  // namespace dcmi
