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

#include "system_call.hpp"
#include <errno.h>
#include <sys/file.h>
#undef errno
extern int errno;

extern "C" {

  int _write(int, char *, int);

  /**
   * @brief Close a file
   * @note  Minimal implementation
   */
  int _close(int file)
  {
    return -1;
  }

  /**
   * @brief End program execution with no cleanup processing
   */
  void _exit(int status)
  {
    _write(1, "exit", 4);
    while (1) {
    }
  }

  /**
   * @brief Status of an open file
   * @note  Minimal implementation
   * @note  All files are regarded as character special devices
   */
  int _fstat(int file, struct stat *st)
  {
    st->st_mode = S_IFCHR;
    return 0;
  }

  /**
   * @brief Return process ID
   * @note  Minimal implementation
   */
  int _getpid(void)
  {
    return 1;
  }

  /**
   * @brief Query whether output stream is a terminal.
   * @note Minimal implementation
   * @note Only support output to stdout
   */
  int _isatty(int file)
  {
    return 1;
  }

  /**
   * @brief Send a signal
   * @note  Minimal implementation
   */
  int _kill(int pid, int sig)
  {
    errno = EINVAL;
    return -1;
  }

  /**
   * @brief Set position in a file
   * @note  Minimal implementation
   */
  int _lseek(int file, int ptr, int dir)
  {
    return 0;
  }

  /**
   * @brief Read from a file
   * @note  Minimal implementation
   */
  int _read(int file, char *ptr, int len)
  {
    return 0;
  }

  /**
   * @brief Increase program space
   * @note  For an stand-alone system
   */
  caddr_t _sbrk(int incr)
  {
    extern char __bss_end__;
    static char *heapEnd = &__bss_end__;

    char *previousHeapEnd = heapEnd;

    register caddr_t stackPointer asm ("sp");

    if (heapEnd + incr > stackPointer) {
      _write(1, "Heap and stack collision\n", 25);
      _exit(0);
    }

    heapEnd += incr;

    return (caddr_t) previousHeapEnd;
  }

  /**
   * @brief Write to a file
   */
  int _write(int file, char *ptr, int len)
  {
    // TODO write should stream information via a serial port
    return len;
  }

} // extern "C"
