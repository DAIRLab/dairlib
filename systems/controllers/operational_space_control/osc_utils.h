#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace dairlib {
namespace systems {
namespace controllers {

// After constructing the OSC, users need to connect trajectory output port to
// OSC input port. This following function gives the index of the input port.
int getInputPortIndex(OperationalSpaceControl* osc, std::string name) {
  std::vector<OscTrackingData> data_vec = osc->GetTrackingDataVector();

  // We'll construct the input port by the order of std::vector<OscTrackingData>
  int index = -1;
  for (int i = 0; i < data_vec.size(); i++) {
    if (data_vec[i].traj_name_.compare(name) == 0) {
      index = i;
    }
  }
  DRAKE_DEMAND(index >= 0);  // name didn't match any of the trajs' names
}

// Convert rotational matrix to Eular angle (roll pitch yaw)
void RotationalMatrixToRollPitchYaw(
  const Eigen::MatrixXd& mat, double* roll, double* pitch, double* yaw);

// Convert Quaternion to Eular angle (roll pitch yaw)
void QuaternionToRollPitchYaw(
  const Eigen::Quaterniond& q, double* roll, double* pitch, double* yaw);


}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
