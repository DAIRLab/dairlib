#pragma once

#include "drake/common/yaml/yaml_read_archive.h"

namespace dairlib {
namespace examples {
namespace plate_balancing {

/**
 * @brief Target configuration for the plate balancing example.
 *
 * Contains target positions and scaling factors for the plate balancing task.
 */
struct PlateBalancingTargetConfig {
  double near_target_threshold;  ///< Threshold for considering the target as
                                 ///< reached.
  std::vector<Eigen::Vector3d>
      first_target;  ///< List of 3D vectors for the first target positions.
  std::vector<Eigen::Vector3d>
      second_target;  ///< List of 3D vectors for the second target positions.
  std::vector<Eigen::Vector3d>
      third_target;  ///< List of 3D vectors for the third target positions.
  double x_scale;    ///< Scaling factor for x direction.
  double y_scale;    ///< Scaling factor for y direction.
  double z_scale;    ///< Scaling factor for z direction.

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(first_target));
    a->Visit(DRAKE_NVP(second_target));
    a->Visit(DRAKE_NVP(third_target));
    a->Visit(DRAKE_NVP(x_scale));
    a->Visit(DRAKE_NVP(y_scale));
    a->Visit(DRAKE_NVP(z_scale));
    a->Visit(DRAKE_NVP(near_target_threshold));
  }
};

}  // namespace plate_balancing
}  // namespace examples
}  // namespace dairlib