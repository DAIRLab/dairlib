//TODO(yminchen):
// In constructor of OSC, you call checkConstraintSettings() function to check
// that the user set the cost correctly.
// This function takes in RBT and check the following things:
// - 

#pragma once

#include <Eigen/Dense>
#include <vector>

namespace dairlib {
namespace systems {
namespace controllers {

class OscCostSettings {
 public:
  OscCostSettings();

  OscCostSettings() {}  // Default constructor


  // Setters
  // TODO: when you set the joint accel cost, you should call a function which
  // checks if the user enter the right index.

  void SetAccelerationCost(int joint_velocity_index, double weight);
  void SetAccelerationCostForAllJoints(MatrixXd weight);

 protected:


 private:
  // Input cost
  bool with_input_cost_;
  MatrixXd W_input_;

  // Joint acceleration cost
  bool with_accelration_cost_;
  std::vector<int> joint_velocity_index_;
  std::vector<double> w_joint_acceleration_;  // cost weight
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
