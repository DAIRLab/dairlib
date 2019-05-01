#include <math.h>
#include "systems/framework/output_vector.h"

using Eigen::VectorXd;

namespace dairlib {

// The functions here assume that Cassie is quaternion floating based.
void GetBaseRollPitchYawPos(
  double & base_roll_pos, double & base_pitch_pos, double & base_yaw_pos,
  const VectorXd & current_position);
void getPelvisLocalRollPitchYawVel(
  double & base_roll_vel, double & base_pitch_vel, double & base_yaw_vel,
  const VectorXd & q, const VectorXd & v);
void QuaternionToEulerAngle(
  const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) ;
void RPYEulerAngleToQuaternion(
  const double& roll, const double& pitch, const double& yaw,
  Eigen::Quaterniond& q);
Eigen::Vector3d rotate3DVecFromGlobalToLocalByRPY(
  double roll, double pitch, double yaw, Eigen::Vector3d v);
Eigen::Vector3d rotate3DVecFromLocalToGlobalByRPY(
  double roll, double pitch, double yaw, Eigen::Vector3d v);

}  // namespace dairlib