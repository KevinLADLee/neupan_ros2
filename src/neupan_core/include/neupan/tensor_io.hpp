/*
 * neupan_cpp: C++ port of the NeuPAN planner.
 *
 * Ported from NeuPAN (https://github.com/hanruihua/NeuPAN),
 * Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. See <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include "neupan/types.hpp"

namespace neupan {

// Simple named-tensor container file ("NPTF"): used for DUNE weights and
// for reference data dumped from the Python implementation.
//
// Layout (little endian):
//   magic 'NPTF' | uint32 version | uint32 count
//   per record: uint32 name_len | name bytes |
//               uint32 dtype (0=f32, 1=f64) | uint32 rows | uint32 cols |
//               row-major data
struct TensorFile {
  // All tensors are widened to double on load; original dtype is preserved
  // in `dtypes` (weights are stored/used as float32 to match PyTorch).
  std::map<std::string, Mat> tensors;
  std::map<std::string, int> dtypes;
  std::vector<std::string> order;  // record order in file

  static TensorFile load(const std::string& path);

  const Mat& at(const std::string& name) const;
  bool has(const std::string& name) const { return tensors.count(name) > 0; }
};

}  // namespace neupan
