#include "examples/Cassie/lipm_cp_control/control_utils.h"

namespace dairlib {

void GetBaseRollPitchYawPos(
  double & base_roll_pos, double & base_pitch_pos, double & base_yaw_pos,
  const VectorXd & current_position) {
  Eigen::Quaterniond q(current_position(3), current_position(4),
                       current_position(5), current_position(6));
  QuaternionToEulerAngle(q, base_roll_pos, base_pitch_pos, base_yaw_pos);
}

void getPelvisLocalRollPitchYawVel(
  double & base_roll_vel, double & base_pitch_vel, double & base_yaw_vel,
  const VectorXd & q, const VectorXd & v) {
  base_roll_vel = v(0);
  base_pitch_vel = v(1);
  base_yaw_vel = v(2);
}

// QuaternionToEulerAngle
void QuaternionToEulerAngle(
  const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw) {
  // Reference: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

  // roll (x-axis rotation)
  double sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
  double cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
  roll = atan2(sinr, cosr);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
  if (fabs(sinp) >= 1)
    pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
  double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
  yaw = atan2(siny, cosy);
}

// EulerAngleToQuaternion
// TODO: test this function (it's untested yet)
void RPYEulerAngleToQuaternion(
  const double& roll, const double& pitch, const double& yaw,
  Eigen::Quaterniond& q) {
  // Reference: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

  // roll
  Eigen::Quaterniond qR(cos(roll / 2), sin(roll / 2), 0,           0         );
  // pitch
  Eigen::Quaterniond qP(cos(pitch / 2), 0,          sin(pitch / 2), 0         );
  // yaw
  Eigen::Quaterniond qY(cos(yaw / 2),  0,          0,           sin(yaw / 2));

  // resultant quaternion
  q = qY * qP * qR;
}

// Rotate a vector in 3D space by roll pitch yaw
Eigen::Vector3d rotate3DVecFromGlobalToLocalByRPY(
  double roll, double pitch, double yaw, Eigen::Vector3d v) {
  // Rotation matrix by hand
  Eigen::Matrix3d rollMatrix;
  Eigen::Matrix3d pitchMatrix;
  Eigen::Matrix3d yawMatrix;
  rollMatrix << 1,  0,          0,
             0,  cos(roll), -sin(roll),
             0,  sin(roll),  cos(roll);
  pitchMatrix <<  cos(pitch), 0, sin(pitch),
              0,          1, 0,
              -sin(pitch), 0, cos(pitch);
  yawMatrix << cos(yaw), -sin(yaw),  0,
            sin(yaw),  cos(yaw),  0,
            0,         0,         1;
  Eigen::Matrix3d rotationMatrix = rollMatrix.transpose() *
                                   pitchMatrix.transpose() * yawMatrix.transpose();

  return rotationMatrix * v; // rotate from global to local frame
}

// Rotate a vector in 3D space by roll pitch yaw
Eigen::Vector3d rotate3DVecFromLocalToGlobalByRPY(
  double roll, double pitch, double yaw, Eigen::Vector3d v) {
  // Rotation matrix by hand
  Eigen::Matrix3d rollMatrix;
  Eigen::Matrix3d pitchMatrix;
  Eigen::Matrix3d yawMatrix;
  rollMatrix << 1,  0,          0,
             0,  cos(roll), -sin(roll),
             0,  sin(roll),  cos(roll);
  pitchMatrix <<  cos(pitch), 0, sin(pitch),
              0,          1, 0,
              -sin(pitch), 0, cos(pitch);
  yawMatrix << cos(yaw), -sin(yaw),  0,
            sin(yaw),  cos(yaw),  0,
            0,         0,         1;
  Eigen::Matrix3d rotationMatrix = yawMatrix * pitchMatrix * rollMatrix;

  return rotationMatrix * v; // rotate from local to global frame
}


}  // namespace dairlib
