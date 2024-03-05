#pragma once

#include <vector>
#include <Eigen/Dense>

#include "drake/common/drake_assert.h"
#include "drake/common/yaml/yaml_io.h"

namespace dairlib::systems {

struct SoSFilterState {
  // input to the filter, stored with x[n-i] = x_(i)
  std::vector<Eigen::Vector3d> x_;
  std::vector<Eigen::Vector3d> y_;
};

/// Second order sections based SISO filter implementation
/// https://en.wikipedia.org/wiki/Digital_biquad_filter
class SoSFilter {

 public:
  SoSFilter(std::vector<Eigen::Vector3d> a, std::vector<Eigen::Vector3d> b) :
      n_(a.size()), a_(a), b_(b) {
    DRAKE_DEMAND(a.size() == b.size());
  };

  static SoSFilter MakeFromCoefficientYaml(const std::string& filename);

  SoSFilterState GetState(double init_val) const;

  double filter(SoSFilterState &filter_state, double input) const;

 private:

  const size_t n_;
  const std::vector<Eigen::Vector3d> a_;
  const std::vector<Eigen::Vector3d> b_;

};

}