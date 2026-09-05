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

#include "neupan/tensor_io.hpp"

#include <cstdint>
#include <fstream>
#include <stdexcept>

namespace neupan {

namespace {

template <typename T>
T readScalar(std::ifstream& f) {
  T v;
  f.read(reinterpret_cast<char*>(&v), sizeof(T));
  if (!f) throw std::runtime_error("tensor_io: unexpected end of file");
  return v;
}

}  // namespace

TensorFile TensorFile::load(const std::string& path) {
  std::ifstream f(path, std::ios::binary);
  if (!f) throw std::runtime_error("tensor_io: cannot open " + path);

  char magic[4];
  f.read(magic, 4);
  if (!f || std::string(magic, 4) != "NPTF")
    throw std::runtime_error("tensor_io: bad magic in " + path);

  const auto version = readScalar<uint32_t>(f);
  if (version != 1)
    throw std::runtime_error("tensor_io: unsupported version in " + path);

  TensorFile out;
  const auto count = readScalar<uint32_t>(f);
  for (uint32_t i = 0; i < count; ++i) {
    const auto nameLen = readScalar<uint32_t>(f);
    std::string name(nameLen, '\0');
    f.read(name.data(), nameLen);

    const auto dtype = readScalar<uint32_t>(f);
    const auto rows = readScalar<uint32_t>(f);
    const auto cols = readScalar<uint32_t>(f);

    Mat m(rows, cols);
    if (dtype == 0) {
      std::vector<float> buf(static_cast<size_t>(rows) * cols);
      f.read(reinterpret_cast<char*>(buf.data()), buf.size() * sizeof(float));
      for (uint32_t r = 0; r < rows; ++r)
        for (uint32_t c = 0; c < cols; ++c)
          m(r, c) = buf[static_cast<size_t>(r) * cols + c];
    } else if (dtype == 1) {
      std::vector<double> buf(static_cast<size_t>(rows) * cols);
      f.read(reinterpret_cast<char*>(buf.data()), buf.size() * sizeof(double));
      for (uint32_t r = 0; r < rows; ++r)
        for (uint32_t c = 0; c < cols; ++c)
          m(r, c) = buf[static_cast<size_t>(r) * cols + c];
    } else {
      throw std::runtime_error("tensor_io: unknown dtype in " + path);
    }
    if (!f) throw std::runtime_error("tensor_io: truncated data in " + path);

    out.tensors[name] = std::move(m);
    out.dtypes[name] = static_cast<int>(dtype);
    out.order.push_back(name);
  }
  return out;
}

const Mat& TensorFile::at(const std::string& name) const {
  auto it = tensors.find(name);
  if (it == tensors.end())
    throw std::runtime_error("tensor_io: missing tensor " + name);
  return it->second;
}

}  // namespace neupan
