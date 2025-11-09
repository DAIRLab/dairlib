#pragma once

#include <vector>
#include <Eigen/Dense>
#include "drake/common/yaml/yaml_read_archive.h"

namespace dairlib {
namespace magna {

struct TargetPose {
  Eigen::Vector3d position;
  Eigen::Vector4d orientation;  // quaternion w,x,y,z
  double gripper_pos_command = 0.0;
  double dwell_seconds = 0.0;  // how long to wait at this target before moving
  // Optional fields for circular arc interpolation
  // If center and radius are both provided and valid, use circular arc interpolation
  // Otherwise, use straight line interpolation
  Eigen::Vector3d center = Eigen::Vector3d::Zero();  // Center of the circular arc
  double radius = -1.0;  // Radius of the circular arc (negative means not set)
};

struct TargetPoseParams {
  std::vector<double> position;  // [x, y, z]
  std::vector<double> orientation;  // [w, x, y, z] quaternion
  double gripper_pos_command;
  double dwell_seconds;
  // Optional fields for circular arc interpolation
  std::vector<double> center;  // [x, y, z] - center of circular arc (optional)
  double radius = -1.0;  // Radius of circular arc (optional, negative means not set)

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

struct TargetPosesParams {
  std::vector<TargetPoseParams> target_poses;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(target_poses));
  }

  /// Convert to vector of TargetPose for use in AssemblyController
  std::vector<TargetPose> ToTargetPoses() const {
    std::vector<TargetPose> result;
    for (const auto& tp : target_poses) {
      TargetPose target_pose;
      target_pose.position = Eigen::Vector3d(tp.position[0], tp.position[1], tp.position[2]);
      target_pose.orientation = Eigen::Vector4d(tp.orientation[0], tp.orientation[1], 
                                                 tp.orientation[2], tp.orientation[3]);
      target_pose.gripper_pos_command = tp.gripper_pos_command;
      target_pose.dwell_seconds = tp.dwell_seconds;
      // Set center and radius if provided
      if (tp.center.size() == 3 && tp.radius > 0.0) {
        target_pose.center = Eigen::Vector3d(tp.center[0], tp.center[1], tp.center[2]);
        target_pose.radius = tp.radius;
      }
      result.push_back(target_pose);
    }
    return result;
  }
};

}  // namespace magna
}  // namespace dairlib

