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

#ifndef __SCALAR_H__
#define __SCALAR_H__

#include "base.h"

#define BASE_USE_FAST_32BIT_LOG2 (1)
#define base_required_bits(n) (log2((n)) + 1)
#define base_round_out(n, a) ((n) < 0 ? (n) - (a) : (n) + (a))

namespace base {

const uint8 log2_byte_lut[] = {
    0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4,
    4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
    5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
    6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
    7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
};

inline uint8 log2(uint8 value) { return log2_byte_lut[value]; }

inline uint8 log2(uint16 value) {
  if (value <= 0xFF) {
    return log2((uint8)value);
  }

  return 8 + log2((uint8)(value >> 8));
}

inline uint8 log2(uint32 value) {
#ifdef BASE_USE_FAST_32BIT_LOG2

  if (value <= 0xFFFF) {
    return log2((uint16)value);
  }

  return 16 + log2((uint16)(value >> 16));

#else

  if (0 == value) {
    return 0;
  }

  // This will provide accurate values for pow-2 aligned
  // inputs. For all else, the return will be truncated to the
  // nearest integer.

  uint32 result = 0;

  while (value >>= 1) {
    result++;
  }

  return result;

#endif
}

inline int8 sign(int8 value) {
  int8 is_non_zero = !!value;
  int8 missing_sign_bit = !(value & 0x80);

  // Branchless sign that returns zero for zero values.
  return (missing_sign_bit - !missing_sign_bit) * is_non_zero;
}

inline int16 sign(int16 value) {
  int16 is_non_zero = !!value;
  int16 missing_sign_bit = !(value & 0x8000);
  return (missing_sign_bit - !missing_sign_bit) * is_non_zero;
}

inline int32 sign(int32 value) {
  int32 is_non_zero = !!value;
  int32 missing_sign_bit = !(value & 0x80000000);
  return (missing_sign_bit - !missing_sign_bit) * is_non_zero;
}

inline float log2(float32 value) {
  int32 *int_value = reinterpret_cast<int *>(&value);
  float32 log_2 = (float32)(((*int_value >> 23) & 255) - 128);
  *int_value &= ~(255 << 23);
  *int_value += 127 << 23;
  log_2 += (-0.34484843f * value + 2.02466578f) * value - 0.67487759f;

  return (log_2);
}

inline int32 log2pow2(int32 value) {
  if (0 == value) return 0;

  // Log2 for pow2 aligned integers. Note that for performance
  // reasons, we do not validate the alignment of ilog.

  int32 result = 0;
  while (value >>= 1) result++;
  return result;
}

inline int8 abs(int8 value) {
  if (value == BASE_MIN_INT8) return BASE_MAX_INT8;

  return (value < 0 ? -value : value);
}

inline int16 abs(int16 value) {
  if (value == BASE_MIN_INT16) return BASE_MAX_INT16;

  return (value < 0 ? -value : value);
}

inline int32 abs(int32 value) {
  if (value == BASE_MIN_INT32) return BASE_MAX_INT32;

  return (value < 0 ? -value : value);
}

inline int16 clip_range(int16 value, int16 min, int16 max) {
  return (value < min ? min : (value > max ? max : value));
}

inline int32 clip_range(int32 value, int32 min, int32 max) {
  return (value < min ? min : (value > max ? max : value));
}

inline float32 clip_range(float32 value, float32 min, float32 max) {
  return (value < min ? min : (value > max ? max : value));
}

inline int16 saturate(int32 input) { return clip_range(input, 0, 255); }

inline float32 saturate(float32 input) { return clip_range(input, 0.0f, 1.0f); }

inline bool is_pow2(uint32 value) { return (0 == (value & (value - 1))); }

inline int32 rounded_div(int32 numer, int32 denom) {
  if ((numer & 0x80000000) ^ (denom & 0x80000000)) {
    return (numer - denom / 2) / denom;
  }

  return (numer + denom / 2) / denom;
}

inline int32 rounded_div_pow2(int32 numer, uint32 pos_denom) {
#if BASE_DEBUG
  if (!is_pow2(pos_denom)) {
    return 0;
  }
#endif

  if (numer & 0x80000000) {
    return (numer - (pos_denom >> 1)) >> (log2(pos_denom));
  }

  return (numer + (pos_denom >> 1)) >> (log2(pos_denom));
}

inline uint32 greater_multiple(uint32 value, uint32 multiple) {
  uint32 mod = value % multiple;

  if (0 != mod) {
    value += multiple - mod;
  }

  return value;
}

inline uint32 align(uint32 value, uint32 alignment) {
  return greater_multiple(value, alignment);
}

inline uint32 align16(uint32 value) {
  return (value & 0xF ? value + ~(value & 0xF) + 1 : value);
}

inline uint32 align8(uint32 value) {
  return (value & 0x7 ? value + ~(value & 0x7) + 1 : value);
}

inline uint32 align2(uint32 value) {
  if (is_pow2(value)) {
    return value;
  }

  int32 power = 0;

  while (value) {
    value >>= 1;
    power++;
  }

  return 1 << power;
}

inline float inv_sqrt(float32 f) {
  // Newton-Raphson approximation with a curiously awesome initial guess
  float32 half = 0.5f * f;
  int32 i = *reinterpret_cast<int32 *>(&f);
  i = 0x5f3759df - (i >> 1);
  f = *reinterpret_cast<float32 *>(&i);
  f = f * (1.5f - half * f * f);
  // f = f * (1.5f - half * f * f);   // if we want extra precision we do an
  // extra degree
  return f;
}

inline float sqrtf(float f) { return 1.0f / inv_sqrt(f); }

inline uint32 sqrt(uint32 f) { return (1.0f / inv_sqrt(f) + 0.5f); }

inline float32 closest_multiple(float32 value, float32 multiple) {
  float32 mod = fmod(value, multiple);

  if (mod > 0.0f) {
    if (mod >= multiple / 2) {
      value += (multiple - mod);
    } else
      value -= mod;
  } else {
    if (mod <= -1.0f * multiple / 2) {
      value -= (multiple - fabs(mod));
    } else
      value += fabs(mod);
  }

  return value;
}

inline int8 big_shift_left(uint8 *buffer, uint32 length, uint32 bits) {
  if (BASE_PARAM_CHECK) {
    if (!buffer || 0 == length) {
      return -1;
    }
  }

  if (0 == bits) {
    return 0;
  }

  uint32 shift_bytes = (bits >> 3);
  uint32 shift_bits = (bits % 8);

  if (shift_bytes) {
    memcpy(buffer + shift_bytes, buffer, length - shift_bytes);
    memset(buffer, 0, shift_bytes);
  }

  if (shift_bits) {
    uint8 carry_in = 0;
    uint8 carry_out = 0;

    for (uint32 i = shift_bytes; i < length; i++) {
      carry_in = carry_out;  // prime with our previous outflow
      carry_out = buffer[i] >> (8 - shift_bits);
      buffer[i] = (buffer[i] << shift_bits) | carry_in;
    }
  }

  return 0;
}

inline int8 big_shift_right(uint8 *buffer, uint32 length, uint32 bits) {
  if (BASE_PARAM_CHECK) {
    if (!buffer || 0 == length) {
      return -1;
    }
  }

  if (0 == bits) {
    return 0;
  }

  uint32 shift_bytes = (bits >> 3);
  uint32 shift_bits = (bits % 8);

  if (shift_bytes) {
    memcpy(buffer, buffer + shift_bytes, length - shift_bytes);
    memset(buffer + length - shift_bytes, 0, shift_bytes);
  }

  if (shift_bits) {
    uint8 carry_in = 0;
    uint8 carry_out = 0;

    for (uint32 i = length - 1; i >= shift_bytes; i++) {
      carry_in = carry_out;  // prime with our previous outflow
      carry_out = buffer[i] << (8 - shift_bits);
      buffer[i] = (buffer[i] >> shift_bits) | carry_in;
    }
  }

  return 0;
}

inline bool compare_epsilon(float32 a, float32 b, float32 ep) {
  if (a > (b - ep) && a < (b + ep)) {
    return true;
  }

  return false;
}

inline bool compare_epsilon(float32 a, float32 b) {
  return compare_epsilon(a, b, BASE_EPSILON);
}

}  // namespace base

#endif  // __SCALAR_H__