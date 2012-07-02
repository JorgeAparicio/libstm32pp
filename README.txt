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

ABOUT
=====

libstm32pp is a template peripheral library for the STM32F1/STM32F2/STM32F4
family of microcontrollers(uC).

This library is written in C++, and uses heavily template metaprogramming,
statics assertions, templates and enumerators.

All these features provide a memory efficient implementation, compile-time
error-checking and strong typing.

INSTALLATION AND USE
====================

There is no installation, this is a header file only library.

To use this library, simply link the libstm32pp/include folder to your source.

The following files are important:

+ device_select.hpp - Here the microcontroller family must be selected.
+ clock.hpp - Here the uC clocks can be configured.
+ interrupt_cpp.hpp - This is actually a source file that configures the uC's
                      interrupts.
+ system_call_cpp.hpp - This is actually a source file that implements the
                        newlib hardware dependant functions.

Check the "demo" folder for more information.

An editor with auto-complete feature is highly recommended for development using
this library.

If you need an open source framework for development of these families of uCs, 
you should check the bareCortexM project:

(https://github.com/JorgeAparicio/bareCortexM)

It's a framework based on the Eclipse IDE and the GCC toolchain, for bare metal
development of Cortex™ M Series processors.

LICENSE
=======

This project is licensed under the:

GNU Lesser General Public License (LGPL) version 3.

See COPYING.GPL3.txt and COPYING.LGPL3.txt for more details. 

DISCLAIMER
==========

Cortex™ is a trademark of ARM Limited. ST, STM32 are trademarks of 
STMicroelectronics. All other trademarks are property of their respective
owners.
