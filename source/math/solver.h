/*
//
// Copyright (c) 1998-2019 Joe Bertolami. All Right Reserved.
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//   AND ANY EXPRESS OR IMPLIED WARRANTIES, CLUDG, BUT NOT LIMITED TO, THE
//   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//   ARE DISCLAIMED.  NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//   LIABLE FOR ANY DIRECT, DIRECT, CIDENTAL, SPECIAL, EXEMPLARY, OR
//   CONSEQUENTIAL DAMAGES (CLUDG, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
//   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSESS TERRUPTION)
//   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER  CONTRACT, STRICT
//   LIABILITY, OR TORT (CLUDG NEGLIGENCE OR OTHERWISE) ARISG  ANY WAY  OF THE
//   USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Additional Information:
//
//   For more information, visit http://www.bertolami.com.
//
*/

#ifndef __SOLVER_H__
#define __SOLVER_H__

#include "scalar.h"

namespace base {

typedef struct {
  float32 t[4];
  uint32 solution_count;

} BASE_SOLVER_SOLUTION;

// Solver routines for quadratic and cubic polynomials

inline void solve_quadratic(float32 a, float32 b, float32 c,
                            BASE_SOLVER_SOLUTION* out_solution) {
  // This solves equations of the form:
  //
  //     at^2 + bt + c = 0

  if (BASE_PARAM_CHECK) {
    if (!out_solution) {
      return;
    }
  }

  out_solution->solution_count = 0;

  float32 discriminant = b * b - 4.0 * a * c;

  memset(out_solution, 0, sizeof(BASE_SOLVER_SOLUTION));

  if (discriminant > 0) {
    out_solution->solution_count = 2;
    out_solution->t[0] = (-b + sqrt(discriminant)) / (2.0 * a);
    out_solution->t[1] = (-b - sqrt(discriminant)) / (2.0 * a);
    return;
  }

  else if (0 == discriminant) {
    // This case could be collapsed into the above one, but we keep
    // it separate for readability.

    out_solution->solution_count = 1;
    out_solution->t[0] = (-b) / (2.0 * a);
  } else {
    // There are no roots to this equation
    out_solution->solution_count = 0;
  }
}

inline void solve_cubic(float32 e, float32 a, float32 b, float32 c,
                        BASE_SOLVER_SOLUTION* out_solution) {
  // Note this is a closed form solution for a cubic equation (thus a cubic
  // formula)
  //
  // This solves the case of:
  //
  //     et^3 + at^2 + bt + c = 0 (i)
  //
  // The first step we take is to divide by e, so that we may instead solve
  // the slightly more generic "cubic formula":
  //
  //     t^3 + at^2 + bt + c = 0 (ii)
  //
  // Then you should divide by the appropriate value to produce (i).

  if (BASE_PARAM_CHECK) {
    if (!out_solution) {
      return;
    }
  }

  out_solution->solution_count = 0;

  a /= e;
  b /= e;
  c /= e;

  // Note that you cannot use pow( n, exp ) if n is negative. It will result in
  // a NaN, as defined by the C99 standard:
  //
  //   "The pow functions compute x raised to the power of y. A domain error
  //    occurs if x is finite or negative ... A domain error may occur if ...
  //    y is less than or equal to zero."
  //
  // Thus we solve this using a different method.

  float32 p = (-1.0f / 3.0f) * a * a + b;
  float32 q = (2.0f / 27.0f) * a * a * a - (1.0f / 3.0f) * a * b + c;
  float32 d = -4.0 * p * p * p - 27.0 * q * q;

  float32 d_prime = -1.0 * (d / 108.0);
  float32 p_prime = p / 3.0f;
  float32 q_prime = q / 2.0f;

  float32 r_base = (-1.0f * q_prime + sqrt(d_prime));
  float32 s_base = (-1.0f * q_prime - sqrt(d_prime));

  float32 r = pow((float32)fabs(r_base), (float32)(1.0f / 3.0f));
  float32 s = pow((float32)fabs(s_base), (float32)(1.0f / 3.0f));

  if (r_base < 0)
    r *= -1.0f;  // workaround -- pow doesn't support negative value bases
  if (s_base < 0)
    s *= -1.0f;  // workaround -- pow doesn't support negative value bases

  memset(out_solution, 0, sizeof(BASE_SOLVER_SOLUTION));

  if (d < 0) {
    out_solution->solution_count = 1;
    out_solution->t[0] = (r + s) - a / 3.0;
    return;
  } else if (0 == d) {
    out_solution->solution_count = 3;
    out_solution->t[0] = 2.0 * r - a / 3.0;
    out_solution->t[1] = -1.0 * r - a / 3.0;
    out_solution->t[2] = -1.0 * r - a / 3.0;
    return;
  } else {
    out_solution->solution_count = 3;

    float32 theta =
        (1.0 / 3.0) *
        acos((-1.0 * q_prime) / (sqrt(-1.0 * p_prime * p_prime * p_prime)));

    out_solution->t[0] = 2.0 * sqrt((-1.0 * p_prime)) * cos(theta) - a / 3.0;
    out_solution->t[1] =
        2.0 * sqrt((-1.0 * p_prime)) * cos(theta + (2.0 * BASE_PI) / 3.0) -
        a / 3.0;
    out_solution->t[2] =
        2.0 * sqrt((-1.0 * p_prime)) * cos(theta - (2.0 * BASE_PI) / 3.0) -
        a / 3.0;
    return;
  }
}

}  // namespace base

#endif  // __SOLVER_H__
