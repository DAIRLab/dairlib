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
  //OscTrackingData();

  OscTrackingData() {}  // Default constructor

  // Updater and getters
  void Update(VectorXd x,
              RigidBodyTree<double>* tree,
              const drake::trajectories::Trajectory<double>* traj, double t,
              int finite_state_machine_state,
              double time_since_last_state_switch);
  Eigen::VectorXd GetDesiredOutputWithPdControl(Eigen::VectorXd q);
  Eigen::MatrixXd GetWeight() {return W_};

  // Setters
  void SetPGain(Eigen::MatrixXd K_p) {K_p_ = K_p;}
  void SetDGain(Eigen::MatrixXd K_d) {K_d_ = K_d;}
  void SetWeight(Eigen::MatrixXd W);
  // Set constant trajectory
  void SetConstantTraj(Eigen::VectorXd v);
  // No control peirod
  void SetNoControlPeriod(double duration) {period_of_no_control_ = duration;}

  // Run this function in OSC constructor to make sure that users constructed
  // OscTrackingData correctly.
  bool CheckOscTrackingData();

 protected:
  std::string name_;

  // dimension of the traj
  int n_r_;

  // PD control gains
  Eigen::MatrixXd K_p_;
  Eigen::MatrixXd K_d_;

  // Trajectory info
  bool traj_is_const_;
  bool traj_has_exp_;

  // Cost weights
  Eigen::MatrixXd W_;

  // Feedback output, jacobian and dJ/dt * v
  Eigen::VectorXd y_;
  Eigen::MatrixXd J_;
  Eigen::VectorXd JdotTimesV_;

  // The states of finite state machine where the tracking is enabled
  // If `state_to_do_tracking_` is empty, then the tracking is always on.
  std::vector<int> state_to_do_tracking_;

 private:
  // Check if we should do tracking in the current state
  bool TrackOrNot(int finite_state_machine_state,
                  double time_since_last_state_switch);

  // Getters/updaters of feedback output, jacobian and dJ/dt * v
  Eigen::VectorXd GetOutput() {return y_}
  Eigen::VectorXd GetJ() {return J_}
  Eigen::VectorXd GetJdotTimesV() {return JdotTimesV_}
  virtual void UpdateOutput(const Eigen::VectorXd& x,
                            RigidBodyTree<double>* tree) = 0;
  virtual void UpdateJ(const Eigen::VectorXd& x,
                       RigidBodyTree<double>* tree) = 0;
  virtual void UpdateJdotTimesV(const Eigen::VectorXd& x,
                                RigidBodyTree<double>* tree) = 0;

  // Desired output
  Eigen::VectorXd y_des_;
  Eigen::VectorXd dy_des_;
  Eigen::VectorXd ddy_des_;

  //
  Eigen::VectorXd fixed_position_;
// in constructor
//   // Testing
//   testing_input_port_ = this->DeclareAbstractInputPort("testing",
//             drake::Value<ExponentialPlusPiecewisePolynomial<double>> {}).get_index();
// in update function that get the input
//   // Testing
//     const drake::AbstractValue* traj_intput =
//       this->EvalAbstractInput(context, testing_input_port_);
//     DRAKE_ASSERT(traj_intput != nullptr);
//     const drake::trajectories::Trajectory<double> & testing_traj =
//         traj_intput->get_value <ExponentialPlusPiecewisePolynomial<double >> ();

  // A period when we don't apply control
  // (starting at the time when fsm switches to a new state)
  double period_of_no_control_ = 0;  // Unit: seconds

  // cache
  bool track_at_current_step_;
}


class TaskSpaceTrackingData : public OscTrackingData {
 public:
  TaskSpaceTrackingData();

  TaskSpaceTrackingData() {}  // Default constructor

  void AddPointToTrack(int body_index, VectorXd pt_on_body, int state);
  void AddPointToTrack(std::vector<int> body_index,
                       std::vector<VectorXd> pt_on_body,
                       std::vector<int> state);

 private:
  std::vector<int> body_index_;
  std::vector<Eigen::VectorXd> pt_on_body_;

  virtual void UpdateOutput(const Eigen::VectorXd& x,
                            RigidBodyTree<double>* tree);
  virtual void UpdateJ(const Eigen::VectorXd& x,
                       RigidBodyTree<double>* tree);
  virtual void UpdateJdotTimesV(const Eigen::VectorXd& x,
                                RigidBodyTree<double>* tree);
}


class TransTaskSpaceTrackingData : public TaskSpaceTrackingData {
 public:
  TransTaskSpaceTrackingData(std::string traj_name, int n_r,
                             Eigen::MatrixXd K_p,
                             Eigen::MatrixXd K_d,
                             Eigen::MatrixXd W,
                             bool traj_is_const = false,
                             bool traj_has_exp = false);

  TransTaskSpaceTrackingData() {}  // Default constructor

 private:
  void UpdateOutput(const Eigen::VectorXd& x,
                    RigidBodyTree<double>* tree) override;
  void UpdateJ(const Eigen::VectorXd& x,
               RigidBodyTree<double>* tree) override;
  void UpdateJdotTimesV(const Eigen::VectorXd& x,
                        RigidBodyTree<double>* tree) override;
}


class RotTaskSpaceTrackingData : public TaskSpaceTrackingData {
 public:
  RotTaskSpaceTrackingData(std::string traj_name, int n_r,
                           Eigen::MatrixXd K_p,
                           Eigen::MatrixXd K_d,
                           Eigen::MatrixXd W,
                           bool traj_is_const = false,
                           bool traj_has_exp = false);

  RotTaskSpaceTrackingData() {}  // Default constructor

 private:
  void UpdateOutput(const Eigen::VectorXd& x,
                    RigidBodyTree<double>* tree) override;
  void UpdateJ(const Eigen::VectorXd& x,
               RigidBodyTree<double>* tree) override;
  void UpdateJdotTimesV(const Eigen::VectorXd& x,
                        RigidBodyTree<double>* tree) override;
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
  JointSpaceTrackingData(std::string traj_name, int n_r,
                         Eigen::MatrixXd K_p,
                         Eigen::MatrixXd K_d,
                         Eigen::MatrixXd W,
                         bool traj_is_const = false,
                         bool traj_has_exp = false);

  JointSpaceTrackingData() {}  // Default constructor

  void AddJointToTrack(int joint_position_index,
                       int joint_velocity_index,
                       int state);
  void AddJointToTrack(std::vector<int> joint_position_index,
                       std::vector<VectorXd> joint_velocity_index,
                       std::vector<int> state);

 private:
  std::vector<int> joint_position_index_;
  std::vector<int> joint_velocity_index_;

  void UpdateOutput(const Eigen::VectorXd& x,
                    RigidBodyTree<double>* tree) override;
  void UpdateJ(const Eigen::VectorXd& x,
               RigidBodyTree<double>* tree) override;
  void UpdateJdotTimesV(const Eigen::VectorXd& x,
                        RigidBodyTree<double>* tree) override;
}


class AbstractTrackingData : public OscTrackingData {
 public:
  AbstractTrackingData(std::string traj_name, int n_r,
                       Eigen::MatrixXd K_p,
                       Eigen::MatrixXd K_d,
                       Eigen::MatrixXd W,
                       bool traj_is_const = false,
                       bool traj_has_exp = false);

  AbstractTrackingData() {}  // Default constructor

  // A bunch of setters here

 private:
  void UpdateOutput(const Eigen::VectorXd& x,
                    RigidBodyTree<double>* tree) override;
  void UpdateJ(const Eigen::VectorXd& x,
               RigidBodyTree<double>* tree) override;
  void UpdateJdotTimesV(const Eigen::VectorXd& x,
                        RigidBodyTree<double>* tree) override;

}



}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
