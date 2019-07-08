#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace dairlib {
namespace systems {
namespace controllers {

class OscTrackingData {
 public:
  OscTrackingData();

  OscTrackingData() {}  // Default constructor

  // A bunch of setters here

  // 
  virtual Eigen::VectorXd CalceDesiredAccel() = 0;

 protected:
  // PD control gains
  double* k_p_ = nullptr;
  double* k_d_ = nullptr;
  Eigen::MatrixXd* K_p_ = nullptr;
  Eigen::MatrixXd* K_d_ = nullptr;
  //TODO: note that the users don't need to track position only, it could be
  // velocity only or acceleration only.

 private:
  // Run this function in OSC constructor to make sure that users constructed
  // OscTrackingData correctly.
  bool CheckOscTrackingData();

  std::string traj_name_;
  int n_r_;  // dimension of the traj

  // Finite state machine
  std::vector<int>* state_indices_ = nullptr;
  // Whether to track the traj or not in each state in finite state machine
  std::vector<bool>* do_track_ = nullptr;

  // Cost weights
  bool cost_weight_is_matrix_;
  double w_;
  Eigen::MatrixXd W_;

  // The source of desired traj
  TrackingDataSource tracking_data_source_;

  // A period when we don't apply control
  double* no_control_period_ = nullptr;  // Starting at the time when fsm switches state.
                                         // Unit: seconds.
}


class TrackingDataSource {
 public:
  TrackingDataSource();

  TrackingDataSource() {}  // Default constructor

  bool traj_from_input_port_;
  bool traj_stored_in_osc_;
    // You can use polymorphism, so you only need this in the code:
    //    drake::trajectories::Trajectory<double>* mother_traj;
    //    *mother_traj = readTrajFromInput();
    // value() and MakeDerivative() are the functions you need. (functions tested)

  bool traj_in_drake_traj_class_;
  bool traj_in_eigen_vector_class_;

  VectorXd* desired_pos_ = nullptr;
  VectorXd* desired_vel_ = nullptr;
  VectorXd* desired_accel_ = nullptr;
}



class TranslationalTaskSpaceTrackingData : public OscTrackingData {
 public:
  TranslationalTaskSpaceTrackingData();

  TranslationalTaskSpaceTrackingData() {}  // Default constructor

  // A bunch of setters here

  Eigen::VectorXd CalceDesiredAccel() override;

 protected:

 private:
  std::vector<int>* body_index_ = nullptr;
  std::vector<Eigen::VectorXd>* pt_on_body_ = nullptr;
}

class RotationalTaskSpaceTrackingData : public OscTrackingData {
 public:
  RotationalTaskSpaceTrackingData();

  RotationalTaskSpaceTrackingData() {}  // Default constructor

  // A bunch of setters here

  Eigen::VectorXd CalceDesiredAccel() override;

 protected:

 private:
  std::vector<int>* body_index_ = nullptr;
  std::vector<Eigen::VectorXd>* pt_on_body_ = nullptr;

  // The RBT and desired position should use quaternion in case of
  // non-uniqueness RPY.
  // TODO: (need to test this) You should convert the relative quaternion to
  // roll-pitch-yaw, and do pd control.
  // In this frame work, you can have differet cost wieght and gains for RPY.
}


class JointSpaceTrackingData : public OscTrackingData {
 public:
  JointSpaceTrackingData();

  JointSpaceTrackingData() {}  // Default constructor

  // A bunch of setters here

  Eigen::VectorXd CalceDesiredAccel() override;

 protected:

 private:
  std::vector<int>* joint_position_index_ = nullptr;
  std::vector<int>* joint_velocity_index_ = nullptr;
}


class AbstractTrackingData : public OscTrackingData {
 public:
  AbstractTrackingData();

  AbstractTrackingData() {}  // Default constructor

  // A bunch of setters here

  Eigen::VectorXd CalceDesiredAccel() override;

 protected:

 private:

}



}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
