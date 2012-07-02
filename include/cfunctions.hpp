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
#include "defs.hpp"

/**
 * @brief Summation of elements
 */

template <int... S>
struct cSum;

template <>
struct cSum<> {
    enum {
      value = 0
    };
};

template <int first, int... rest>
struct cSum<first, rest...> {
    enum {
      value = cSum<rest...>::value + first,
    };
};

/**
 * @brief Product of elements
 */
template <int... F>
struct cProduct;

template <>
struct cProduct<> {
    enum {
      value = 1
    };
};

template <int first, int... rest>
struct cProduct<first, rest...> {
    enum {
      value = first * cProduct<rest...>::value,
    };
};

/**
 * @brief Computes b ^ n.
 */
template <int b, int n>
struct cPow {
    enum {
      value = b * cPow<b, n - 1>::value,
    };
};

template <int b>
struct cPow<b, 0> {
    enum {
      value = 1,
    };
};

/**
 * @brief Computes the factorial
 */
constexpr u32 cFactorial(u32 const N)
{
  return N > 0? N * cFactorial(N - 1) : 1;
}

/**
 * @brief Prints as an error
 */
template<int x> struct _;
#define cPrint(x) _<x> __
