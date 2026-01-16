#pragma once
#include <Eigen/Dense>

#include "drake/common/yaml/yaml_read_archive.h"

namespace dairlib {
namespace examples {
namespace magna {

struct SingleOSCTargetPose {
  Eigen::Vector3d position;     // [x, y, z]
  Eigen::Vector4d orientation;  // [w, x, y, z] quaternion
  double gripper_pos_command;
  double dwell_seconds;
  // Optional fields for circular arc interpolation
  Eigen::Vector3d center;  // [x, y, z] - center of circular arc (optional)
  double radius =
      -1.0;  // Radius of circular arc (optional, negative means not set)

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(position));
    a->Visit(DRAKE_NVP(orientation));
    a->Visit(DRAKE_NVP(gripper_pos_command));
    a->Visit(DRAKE_NVP(dwell_seconds));
    a->Visit(DRAKE_NVP(center));
    a->Visit(DRAKE_NVP(radius));
  }
};

}  // namespace magna
}  // namespace examples
}  // namespace dairlib
