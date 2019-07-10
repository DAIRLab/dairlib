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

  void UpdateTrackingData(Eigen::VectorXd x, RigidBodyTree<double>* tree){
    UpdateOutput(x, tree);
    UpdateJ(x, tree);
    UpdateJdotTimesV(x, tree);
  }
  Eigen::VectorXd GetOutput();
  Eigen::VectorXd GetJ();
  Eigen::VectorXd GetJdotTimesV();

  // A bunch of setters here

  // Add constant trajectory
  // add system name as well, so that later we can all GetMutableSystems() from
  // DiagramBuilder and find the system by name, then connect the ports.
  void AddConstantTraj(Eigen::VectorXd v) {
    // https://github.com/RobotLocomotion/drake/blob/1644e19001e1e6f013afe5ac04819890a1a9c814/systems/primitives/trajectory_source.h
    // PiecewisePolynomial (const Eigen::MatrixBase< Derived > &constant_value)
  }

 protected:
  // PD control gains
  Eigen::MatrixXd K_p_;
  Eigen::MatrixXd K_d_;

 private:
  // Run this function in OSC constructor to make sure that users constructed
  // OscTrackingData correctly.
  bool CheckOscTrackingData();

  std::string name_;
  int n_r_;  // dimension of the traj

  // Feedback output, jacobian and dJ/dt * v (cache and update function)
  Eigen::VectorXd y_;
  Eigen::MatrixXd J_;
  Eigen::VectorXd JdotTimesV_;
  virtual void UpdateOutput(const Eigen::VectorXd& x,
    RigidBodyTree<double>* tree) = 0;
  virtual void UpdateJ(Eigen::VectorXd x) = 0;
  virtual void UpdateJdotTimesV(Eigen::VectorXd x) = 0;

  // Finite state machine (optional)
  std::vector<int> state_indices_;
  // Whether to track the traj or not in each state in finite state machine
  std::vector<bool> do_track_;

  // Cost weights
  Eigen::MatrixXd W_;

  // The source of desired traj
  bool traj_is_constant_ = false;
  // TODO(yminchen): you probably don't need the above line, multiplying with zeros is fine
  bool traj_has_exp_ = false;
    // You can use polymorphism, so you only need this in the code:
    //    drake::trajectories::Trajectory<double>* mother_traj;
    //    *mother_traj = readTrajFromInput();
    // value() and MakeDerivative() are the functions you need. (functions tested)

    // if traj is constant:
    //   You look at kp, kd gains, if they are not empty, you track those
    //   You don't track acceleration at all here.
    //   TODO: check if you can get a constant velocity traj leaf system
    // if traj is not constant:
    //   You always get zeroth, first, second time derivatives

  // A period when we don't apply control
  double period_of_no_control_ = 0;  // Starting at the time when fsm switches state.
                                     // Unit: seconds.
}

class TranslationalTaskSpaceTrackingData : public OscTrackingData {
 public:
  TranslationalTaskSpaceTrackingData();

  TranslationalTaskSpaceTrackingData() {}  // Default constructor

  // A bunch of setters here

  void UpdateOutput(const Eigen::VectorXd& x, RigidBodyTree<double>* tree) override;
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

  void UpdateOutput(const Eigen::VectorXd& x, RigidBodyTree<double>* tree) override;
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

  void UpdateOutput(const Eigen::VectorXd& x, RigidBodyTree<double>* tree) override;
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

  void UpdateOutput(const Eigen::VectorXd& x, RigidBodyTree<double>* tree) override;
  void UpdateJ(Eigen::VectorXd x) override;
  void UpdateJdotTimesV(Eigen::VectorXd x) override;

 protected:

 private:

}



}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
