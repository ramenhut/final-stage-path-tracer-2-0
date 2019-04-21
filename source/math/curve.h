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

#ifndef __CURVE_H__
#define __CURVE_H__

#include <vector>
#include "base.h"
#include "vector3.h"

namespace base {

// This system generates bezier curves using two, three, or four
// control points in a set. If fewer than two or greater than four
// points are specified, an error will result.

class curve {
 protected:
  ::std::vector<vector3> control_list;

 public:
  curve();
  curve(const curve& rhs);
  virtual ~curve();

  virtual void add_point(const vector3& p_point);
  virtual int8 evaluate(float32 fT, vector3* output);
  virtual void assign(const curve& input);

  const curve& operator=(const curve& rhs);
  bool operator==(const curve& rhs) const;
  bool operator!=(const curve& rhs) const;
};

class path {
  ::std::vector<curve> curve_list;

 public:
  path();
  ~path();

  void add_curve(const curve& input);
  uint32 query_occupancy() const;

  // evaluate expects fT to range from 0...1, and describe the entire
  // path (potentially across several curves). This function will in
  // turn call the individual curves' evaluate, which expects an fT
  // ranging from 0...1.
  //
  // Thus, we divide fT into equal buckets based on the number of curves
  // in m_CurveList. Whichever bucket fT falls into determines the curve
  // that needs to be accessed from our list. We then map the value's
  // location within the bucket to match the 0...1 range expected by the
  // selected curve.

  int8 evaluate(float32 fT, vector3* output);
  void assign(const path& input);
};

}  // namespace base

#endif  // __CURVE_H__