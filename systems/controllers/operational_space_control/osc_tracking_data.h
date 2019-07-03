#pragma once

#include <Eigen/Dense>
#include <string>

namespace dairlib {
namespace systems {
namespace controllers {

class OscTrackingData {
 public:
  OscTrackingData();

  OscTrackingData() {}  // Default constructor

  // A bunch of setters here



 protected:

 private:
  // Run this function in OSC constructor to make sure that users constructed
  // OscTrackingData correctly.
  bool CheckOscTrackingData();


  std::string traj_name_;
  int n_r_;  // dimension of the traj

  // Finite state machine
  bool use_with_finite_state_machine_;
  std::vector<int> state_indices_;
  // Whether to track the traj or not in each state in finite state machine
  std::vector<bool> do_track_;

  // Cost weights
  bool cost_weight_is_matrix_;
  double w_;
  Eigen::MatrixXd W_;

  // PD control gains
  double k_p_;
  double k_d_;

  // Whether or not users created a function themselves
  bool user_define_traj_function_ = false;
  OscUserDefinedTrajData user_defined_data_;

  // Whether or not the desired traj is from another leafsystem's output
  bool traj_is_from_input_port_;
  // You can use polymorphism, so you only need this in the code:
  //    drake::trajectories::Trajectory<double>* mother_traj;
  //    *mother_traj = readTrajFromInput();
  // value() and MakeDerivative() are the functions you need. (functions tested)

  // Whether or not the traj is in operational space control
  bool traj_in_operational_space_;
  // Tracking in operational space:
  int body_index_;
  Eigen::VectorXd pt_on_body_;
  // Tracking in joint space:
  int joint_position_index_;
  int joint_velocity_index_;
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
