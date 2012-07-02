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

/*******************************************************************************
 *
 *                      Select your target device below.
 *
 ******************************************************************************/

/** STM32F1 Device? ***********************************************************/
//#define STM32F1XX
/********* Comment the macro above to answer no, otherwise your answer is yes */

#ifdef STM32F1XX

/** Value Line Device? ********************************************************/
//#define VALUE_LINE
/********* Comment the macro above to answer no, otherwise your answer is yes */

#ifndef VALUE_LINE

/** Connectivity Line Device? *************************************************/
//#define CONNECTIVITY_LINE
/********* Comment the macro above to answer no, otherwise your answer is yes */

#endif // VALUE_LINE
#if not defined VALUE_LINE && \
    not defined CONNECTIVITY_LINE

/** XL-Density Device? ********************************************************/
//#define XL_DENSITY
/********* Comment the macro above to answer no, otherwise your answer is yes */

#endif //!VALUE_LINE && !CONNECTIVITY_LINE
#endif // STM32F1XX
#ifndef STM32F1XX

/** STM32F2 Device? ***********************************************************/
//#define STM32F2XX
/********* Comment the macro above to answer no, otherwise your answer is yes */

#endif
#if not defined STM32F1XX && \
    not defined STM32F2XX

/** STM32F4 Device? ***********************************************************/
#define STM32F4XX
/********* Comment the macro above to answer no, otherwise your answer is yes */

#endif // !STM32F1XX && !STM32F2XX
/*******************************************************************************
 *
 *               DON'T MODIFY ANYTHING BELOW THIS COMMENT BLOCK
 *
 ******************************************************************************/

// No more than one device family allowed
#if (defined STM32F1XX && defined STM32F2XX) || \
    (defined STM32F1XX && defined STM32F4XX) || \
    (defined STM32F2XX && defined STM32F4XX) || \
    (defined VALUE_LINE && defined CONNECTIVITY_LINE) || \
    (defined VALUE_LINE && defined XL_DENSITY) || \
    (defined CONNECTIVITY_LINE && defined XL_DENSITY)
#error "More than one device family selected."
#endif
