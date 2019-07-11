#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace dairlib {
namespace systems {
namespace controllers {

// OscTrackingData is a virtual class
class OscTrackingData {
 public:
  OscTrackingData(int n_r, std::string traj_name);

  OscTrackingData() {}  // Default constructor

  // Updater and getter
  void UpdateFeedbackOutput(Eigen::VectorXd x, RigidBodyTree<double>* tree);
  void UpdateDesiredOutput(
    const drake::trajectories::Trajectory<double>* mother_traj, double t);
  Eigen::VectorXd GetDesiredOutputWithPdControl(Eigen::VectorXd q);
  Eigen::MatrixXd GetWeight(int finite_state_machine_state){return W_};

  // Setters
  void SetPGain(Eigen::MatrixXd K_p) {K_p_ = K_p;}
  void SetDGain(Eigen::MatrixXd K_d) {K_d_ = K_d;}
  void SetWeight(Eigen::MatrixXd W);

  // Add finite state machine state
  void AddFiniteMachineState(int state){
    state_to_do_tracking_.push_back(state);
  }

  // Add constant trajectory
  void AddConstantTraj(Eigen::VectorXd v,
                       drake::systems::DiagramBuilder<double> & builder);

  // Run this function in OSC constructor to make sure that users constructed
  // OscTrackingData correctly.
  bool CheckOscTrackingData();

 protected:
  // Feedback output, jacobian and dJ/dt * v
  Eigen::VectorXd y_;
  Eigen::MatrixXd J_;
  Eigen::VectorXd JdotTimesV_;

 private:
  std::string name_;
  int n_r_;  // dimension of the traj

  // Getters/updaters of feedback output, jacobian and dJ/dt * v
  Eigen::VectorXd GetOutput() {return y_}
  Eigen::VectorXd GetJ() {return J_}
  Eigen::VectorXd GetJdotTimesV() {return JdotTimesV_}
  virtual void UpdateOutput(const Eigen::VectorXd& x,
                            RigidBodyTree<double>* tree) = 0;
  virtual void UpdateJ(Eigen::VectorXd x) = 0;
  virtual void UpdateJdotTimesV(Eigen::VectorXd x) = 0;

  // Desired output
  Eigen::VectorXd y_des_;
  Eigen::VectorXd dy_des_;
  Eigen::VectorXd ddy_des_;

  // The states of finite state machine where the tracking is enabled
  // If `state_to_do_tracking_` is empty, then the tracking is always on.
  std::vector<int> state_to_do_tracking_;

  // PD control gains
  Eigen::MatrixXd K_p_;
  Eigen::MatrixXd K_d_;

  // Cost weights
  Eigen::MatrixXd W_;

  // The source of desired traj
  bool traj_has_exp_ = false;
  // You can use polymorphism, so you only need this in the code:
  //    drake::trajectories::Trajectory<double>* mother_traj;
  //    *mother_traj = readTrajFromInput();
  // value() and MakeDerivative() are the functions you need. (functions tested)

  // A period when we don't apply control
  // (starting at the time when fsm switches to a new state)
  double period_of_no_control_ = 0;  // Unit: seconds
}

class TranslationalTaskSpaceTrackingData : public OscTrackingData {
 public:
  TranslationalTaskSpaceTrackingData();

  TranslationalTaskSpaceTrackingData() {}  // Default constructor

  // A bunch of setters here

  void UpdateOutput(const Eigen::VectorXd& x,
                    RigidBodyTree<double>* tree) override;
  void UpdateJ(Eigen::VectorXd x) override;
  void UpdateJdotTimesV(Eigen::VectorXd x) override;

 protected:

 private:
  std::vector<int> body_index_;
  std::vector<Eigen::VectorXd> pt_on_body_;
}

class RotationalTaskSpaceTrackingData : public OscTrackingData {
 public:
  RotationalTaskSpaceTrackingData();

  RotationalTaskSpaceTrackingData() {}  // Default constructor

  // A bunch of setters here

  void UpdateOutput(const Eigen::VectorXd& x,
                    RigidBodyTree<double>* tree) override;
  void UpdateJ(Eigen::VectorXd x) override;
  void UpdateJdotTimesV(Eigen::VectorXd x) override;

 protected:

 private:
  std::vector<int> body_index_;
  std::vector<Eigen::VectorXd> pt_on_body_;

  // TODO: can use this to get J and JdotTimesV
  // https://drake.mit.edu/doxygen_cxx/class_rigid_body_tree.html#a3f4ec4dc3b053f6420a5fd449ff4d2c3


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

  void UpdateOutput(const Eigen::VectorXd& x,
                    RigidBodyTree<double>* tree) override;
  void UpdateJ(Eigen::VectorXd x) override;
  void UpdateJdotTimesV(Eigen::VectorXd x) override;

 protected:

 private:
  std::vector<int> joint_position_index_;
  std::vector<int> joint_velocity_index_;
}


class AbstractTrackingData : public OscTrackingData {
 public:
  AbstractTrackingData();

  AbstractTrackingData() {}  // Default constructor

  // A bunch of setters here

  void UpdateOutput(const Eigen::VectorXd& x,
                    RigidBodyTree<double>* tree) override;
  void UpdateJ(Eigen::VectorXd x) override;
  void UpdateJdotTimesV(Eigen::VectorXd x) override;

 protected:

 private:

}



}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
