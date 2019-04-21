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

#ifndef __HASH_H__
#define __HASH_H__

#include "base.h"

namespace base {

inline uint32 hash_bytes(uint8 *input, uint32 length) {
  // Modified Bernstein hashing function -- ours provides a more
  // ideal (high entropy) distribution.
  uint32 hash_code = 5381;
  for (uint32 i = 0; i < length; i++) {
    uint32 c = input[i];

    hash_code = ((hash_code << 5) + hash_code) + (hash_code << 16) * c * c;
  }

  return hash_code;
}

}  // namespace base

#endif  // __HASH_H__