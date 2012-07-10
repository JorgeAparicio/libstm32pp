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

// Must select at least one family
#if not defined STM32F1XX && \
    not defined STM32F2XX && \
    not defined STM32F4XX
#error "Must select at least one family."
#endif

// No more than one device family allowed
#if (defined STM32F1XX && defined STM32F2XX) || \
    (defined STM32F1XX && defined STM32F4XX) || \
    (defined STM32F2XX && defined STM32F4XX) || \
    (defined VALUE_LINE && defined CONNECTIVITY_LINE) || \
    (defined VALUE_LINE && defined XL_DENSITY) || \
    (defined CONNECTIVITY_LINE && defined XL_DENSITY)
#error "More than one device family selected."
#endif


