#include "sos_filter.h"
#include "common/find_resource.h"
#include <map>

namespace dairlib::systems {

using Eigen::Vector3d;
using std::vector;

namespace {
double apply_filter(
    Vector3d &x, Vector3d &y, const Vector3d& a, const Vector3d& b, double u) {
  // first, shift the input and output buffer
  x.tail<2>() = x.head<2>();
  y.tail<2>() = y.head<2>();
  x(0) = u;

  // Compute difference equation
  // a0 y[n] + a1 y[n-1] + a2 y[n-2] = b0 x[n] + b1 x[n-1] + b2 x[n-2]
  double output = b.dot(x) - y.tail<2>().dot(a.tail<2>());
  y(0) = output / a(0);
  return y(0);
}
}

SoSFilter SoSFilter::MakeFromCoefficientYaml(const std::string &filename) {
  auto coeffs = drake::yaml::LoadYamlFile<
      std::map<std::string, vector<vector<double>>>>(
      FindResourceOrThrow(filename)
  );

  DRAKE_DEMAND(coeffs.count("a") == 1);
  DRAKE_DEMAND(coeffs.count("b") == 1);

  vector<Vector3d> a;
  vector<Vector3d> b;

  for (const auto& section: coeffs.at("a")) {
    DRAKE_DEMAND(section.size() == 3);
    a.push_back(Vector3d::Map(section.data()));
  }
  for (const auto& section: coeffs.at("b")) {
    DRAKE_DEMAND(section.size() == 3);
    b.push_back(Vector3d::Map(section.data()));
  }
  return SoSFilter(a, b);
}

double SoSFilter::filter(SoSFilterState &filter_state, double input) const {
  double y = input;
  for (size_t i = 0; i < n_; ++i) {
    y = apply_filter(
        filter_state.x_.at(i), filter_state.y_.at(i), a_.at(i), b_.at(i), y);
  }
  return y;
}

SoSFilterState SoSFilter::GetState(double init_val) const {
  return {
    vector<Vector3d>(n_, Vector3d::Constant(init_val)),
    vector<Vector3d>(n_, Vector3d::Constant(init_val))
  };
}

}