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

#ifndef __STATISTICS_H__
#define __STATISTICS_H__

#include <vector>
#include "base.h"
#include "matrix2.h"
#include "matrix3.h"
#include "vector2.h"
#include "vector3.h"
#include "vector4.h"

namespace base {

float32 compute_mean(float32* point_list, uint32 count);
vector2 compute_mean(vector2* point_list, uint32 count);
vector3 compute_mean(vector3* point_list, uint32 count);
vector4 compute_mean(vector4* point_list, uint32 count);

float32 compute_variance(float32* point_list, uint32 count);
vector2 compute_variance(vector2* point_list, uint32 count);
vector3 compute_variance(vector3* point_list, uint32 count);
vector4 compute_variance(vector4* point_list, uint32 count);

float32 compute_deviation(float32* point_list, uint32 count);
vector2 compute_deviation(vector2* point_list, uint32 count);
vector3 compute_deviation(vector3* point_list, uint32 count);
vector4 compute_deviation(vector4* point_list, uint32 count);

vector2 compute_covariance(vector2* point_listA, vector2* point_listB,
                           uint32 count);
vector3 compute_covariance(vector3* point_listA, vector3* point_listB,
                           uint32 count);
vector4 compute_covariance(vector4* point_listA, vector4* point_listB,
                           uint32 count);

void compute_linear_squares(vector2* point_list, uint32 count,
                            vector2* out_start, vector2* out_end);
void compute_linear_squares(vector3* point_list, uint32 count,
                            vector3* out_start, vector3* out_end);
void compute_linear_squares(vector4* point_list, uint32 count,
                            vector4* out_start, vector4* out_end);

}  // namespace base

#endif  // __STATISTICS_H__