// Verify a newly exported model using the same float32 MLP as the planner.
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "neupan/mlp.hpp"
#include "neupan/tensor_io.hpp"

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: neupan_verify_model MODEL.bin MODEL.bin.validation.nptf\n";
    return 2;
  }
  try {
    const auto model_file = neupan::TensorFile::load(argv[1]);
    const auto fixture = neupan::TensorFile::load(argv[2]);
    const auto model = neupan::MLP::fromTensors(model_file);
    for (const auto* key : {"meta.G", "meta.h"}) {
      const auto& actual = model_file.at(key);
      const auto& expected = fixture.at(key);
      if (actual.rows() != expected.rows() || actual.cols() != expected.cols() ||
          !actual.allFinite() || !expected.allFinite() ||
          (actual - expected).cwiseAbs().maxCoeff() > 1e-6)
        throw std::runtime_error("geometry mismatch");
    }
    const neupan::MatF points = fixture.at("points").cast<float>();
    const auto& expected = fixture.at("mu");
    if (points.rows() != 2 || points.cols() == 0 ||
        expected.rows() != model.outputDim() || expected.cols() != points.cols())
      throw std::runtime_error("invalid validation fixture shape");
    const neupan::Mat actual = model.forward(points).cast<double>();
    if (!actual.allFinite() || !expected.allFinite())
      throw std::runtime_error("non-finite MLP output");
    const double mu_error = (actual - expected).cwiseAbs().maxCoeff();
    const auto& g = model_file.at("meta.G");
    const auto& h = model_file.at("meta.h");
    if (g.rows() != actual.rows() || g.cols() != 2 || h.rows() != g.rows() || h.cols() != 1)
      throw std::runtime_error("invalid geometry shape");
    const neupan::Mat terms = (g * points.cast<double>()).colwise() - h.col(0);
    const double distance_error =
        ((actual - expected).array() * terms.array()).colwise().sum().abs().maxCoeff();
    std::cout << "points=" << points.cols() << " max_mu_error=" << mu_error
              << " max_distance_error=" << distance_error << '\n';
    if (mu_error > 1e-4 || distance_error > 1e-3)
      throw std::runtime_error("Python/C++ numerical tolerances exceeded");
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
