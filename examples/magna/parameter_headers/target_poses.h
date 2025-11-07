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
};

struct TargetPoseParams {
  std::vector<double> position;  // [x, y, z]
  std::vector<double> orientation;  // [w, x, y, z] quaternion
  double gripper_pos_command;
  double dwell_seconds;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(position));
    a->Visit(DRAKE_NVP(orientation));
    a->Visit(DRAKE_NVP(gripper_pos_command));
    a->Visit(DRAKE_NVP(dwell_seconds));
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
      result.push_back(target_pose);
    }
    return result;
  }
};

}  // namespace magna
}  // namespace dairlib

