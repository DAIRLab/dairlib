#pragma once

#include <Eigen/Core>
#include <array>
namespace dairlib {
namespace systems {
struct Face {
  double area;
  Eigen::Vector3d normal;
  std::array<Eigen::Vector3d, 3> v;
};
}
}