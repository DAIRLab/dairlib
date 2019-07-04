//TODO(yminchen):
// In constructor of OSC, you call checkConstraintSettings() function to check
// that the user set the constraint correctly.
// This function takes in RBT and check the following things:
// - size of body_index_, pt_on_body_ and mu_ are the same
// - 

#pragma once

#include <Eigen/Dense>
#include <vector>

namespace dairlib {
namespace systems {
namespace controllers {

class OscConstraintSettings {
 public:
  OscConstraintSettings();

  OscConstraintSettings() {}  // Default constructor


 protected:


 private:
  // Input constraint
  bool with_input_constraints_;
  // std::vector<double> u_min_;
  // std::vector<double> u_max_;
  // Actually you can get the input limit from tree:
  //    umax_ = tree_without_->actuators[1].effort_limit_max_;

  // (flat ground) Contact constraints and friction cone cnostraints
  bool with_contact_constraints_;
  std::vector<int> body_index_;
  std::vector<Eigen::VectorXd> pt_on_body_;
  std::vector<double> mu_;  // Friction coefficients
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
