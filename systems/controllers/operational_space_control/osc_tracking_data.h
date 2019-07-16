#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"

#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {
namespace controllers {

// OscTrackingData is a virtual class
class OscTrackingData {
 public:
  OscTrackingData(std::string name, int n_r,
                  Eigen::MatrixXd K_p,
                  Eigen::MatrixXd K_d,
                  Eigen::MatrixXd W,
                  bool traj_is_const = false,
                  bool traj_has_exp = false);

  OscTrackingData() {}  // Default constructor

  // Updater and getters used by osc block
  bool Update(Eigen::VectorXd x,
              const KinematicsCache<double>& cache,
              RigidBodyTree<double>* tree,
              const drake::trajectories::Trajectory<double>& traj, double t,
              int finite_state_machine_state,
              double time_since_last_state_switch);
  Eigen::VectorXd GetOutput() {return y_;}
  Eigen::VectorXd GetJ() {return J_;}
  Eigen::VectorXd GetJdotTimesV() {return JdotV_;}
  Eigen::VectorXd GetDesiredOutputWithPdControl(Eigen::VectorXd v);
  Eigen::MatrixXd GetWeight();
  bool TrajHasExp() {return traj_has_exp_;}

  // Getters
  std::string GetName() {return name_;};
  bool TrajIsConst() {return traj_is_const_;}
  Eigen::VectorXd GetFixedPosition() {return fixed_position_;}

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
  void CheckOscTrackingData();

 protected:
  // Feedback output, jacobian and dJ/dt * v
  Eigen::VectorXd error_y_;
  Eigen::VectorXd y_;
  Eigen::VectorXd ydot_;
  Eigen::MatrixXd J_;
  Eigen::VectorXd JdotV_;

  // The states of finite state machine where the tracking is enabled
  // If `state_` is empty, then the tracking is always on.
  std::vector<int> state_;

 private:
  // Check if we should do tracking in the current state
  void TrackOrNot(int finite_state_machine_state,
                  double time_since_last_state_switch);

  // Updaters of feedback output, jacobian and dJ/dt * v
  virtual void UpdateError(const Eigen::VectorXd& x,
                            const KinematicsCache<double>& cache,
                            RigidBodyTree<double>* tree) = 0;
  virtual void UpdateJ(const Eigen::VectorXd& x,
                       const KinematicsCache<double>& cache,
                       RigidBodyTree<double>* tree) = 0;
  virtual void UpdateJdotV(const Eigen::VectorXd& x,
                           const KinematicsCache<double>& cache,
                           RigidBodyTree<double>* tree) = 0;

  std::string name_;

  // dimension of the traj
  int n_r_;

  // PD control gains
  Eigen::MatrixXd K_p_;
  Eigen::MatrixXd K_d_;

  // Cost weights
  Eigen::MatrixXd W_;

  // Trajectory info
  bool traj_is_const_;
  bool traj_has_exp_;

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
  int state_idx_;
};


class TaskSpaceTrackingData : public OscTrackingData {
 public:
  TaskSpaceTrackingData(std::string name, int n_r,
                        Eigen::MatrixXd K_p,
                        Eigen::MatrixXd K_d,
                        Eigen::MatrixXd W,
                        bool traj_is_const = false,
                        bool traj_has_exp = false);

  TaskSpaceTrackingData() {}  // Default constructor

  void AddPointToTrack(int body_index, Eigen::VectorXd pt_on_body, int state);
  void AddPointToTrack(std::vector<int> body_index,
                       std::vector<Eigen::VectorXd> pt_on_body,
                       std::vector<int> state);

 private:
  std::vector<int> body_index_;
  std::vector<Eigen::VectorXd> pt_on_body_;

  virtual void UpdateError(const Eigen::VectorXd& x,
                            const KinematicsCache<double>& cache,
                            RigidBodyTree<double>* tree);
  virtual void UpdateJ(const Eigen::VectorXd& x,
                       const KinematicsCache<double>& cache,
                       RigidBodyTree<double>* tree);
  virtual void UpdateJdotV(const Eigen::VectorXd& x,
                           const KinematicsCache<double>& cache,
                           RigidBodyTree<double>* tree);
};


class TransTaskSpaceTrackingData : public TaskSpaceTrackingData {
 public:
  TransTaskSpaceTrackingData(std::string name, int n_r,
                             Eigen::MatrixXd K_p,
                             Eigen::MatrixXd K_d,
                             Eigen::MatrixXd W,
                             bool traj_is_const = false,
                             bool traj_has_exp = false,
                             bool track_center_of_mass = false);

  TransTaskSpaceTrackingData() {}  // Default constructor

 private:
  void UpdateError(const Eigen::VectorXd& x,
                    const KinematicsCache<double>& cache,
                    RigidBodyTree<double>* tree) override;
  void UpdateJ(const Eigen::VectorXd& x,
               const KinematicsCache<double>& cache,
               RigidBodyTree<double>* tree) override;
  void UpdateJdotV(const Eigen::VectorXd& x,
                   const KinematicsCache<double>& cache,
                   RigidBodyTree<double>* tree) override;

  bool track_center_of_mass_;
};


class RotTaskSpaceTrackingData : public TaskSpaceTrackingData {
 public:
  RotTaskSpaceTrackingData(std::string name, int n_r,
                           Eigen::MatrixXd K_p,
                           Eigen::MatrixXd K_d,
                           Eigen::MatrixXd W,
                           bool traj_is_const = false,
                           bool traj_has_exp = false,
                           Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity());

  RotTaskSpaceTrackingData() {}  // Default constructor

 private:
  void UpdateError(const Eigen::VectorXd& x,
                    const KinematicsCache<double>& cache,
                    RigidBodyTree<double>* tree) override;
  void UpdateJ(const Eigen::VectorXd& x,
               const KinematicsCache<double>& cache,
               RigidBodyTree<double>* tree) override;
  void UpdateJdotV(const Eigen::VectorXd& x,
                   const KinematicsCache<double>& cache,
                   RigidBodyTree<double>* tree) override;
  // TODO: can use this to get J and JdotTimesV
  // https://drake.mit.edu/doxygen_cxx/class_rigid_body_tree.html#a3f4ec4dc3b053f6420a5fd449ff4d2c3

  // The RBT and desired position should use quaternion in case of
  // non-uniqueness RPY.
  // TODO: (need to test this) You should convert the relative quaternion to
  // roll-pitch-yaw, and do pd control.
  // In this frame work, you can have differet cost wieght and gains for RPY.
  Eigen::Isometry3d isometry_;
};


class JointSpaceTrackingData : public OscTrackingData {
 public:
  JointSpaceTrackingData(std::string name, int n_r,
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
                       std::vector<int> joint_velocity_index,
                       std::vector<int> state);

 private:
  std::vector<int> joint_position_index_;
  std::vector<int> joint_velocity_index_;

  void UpdateError(const Eigen::VectorXd& x,
                    const KinematicsCache<double>& cache,
                    RigidBodyTree<double>* tree) override;
  void UpdateJ(const Eigen::VectorXd& x,
               const KinematicsCache<double>& cache,
               RigidBodyTree<double>* tree) override;
  void UpdateJdotV(const Eigen::VectorXd& x,
                   const KinematicsCache<double>& cache,
                   RigidBodyTree<double>* tree) override;
};


class AbstractTrackingData : public OscTrackingData {
 public:
  AbstractTrackingData(std::string name, int n_r,
                       Eigen::MatrixXd K_p,
                       Eigen::MatrixXd K_d,
                       Eigen::MatrixXd W,
                       bool traj_is_const = false,
                       bool traj_has_exp = false);

  AbstractTrackingData() {}  // Default constructor

  // A bunch of setters here

 private:
  void UpdateError(const Eigen::VectorXd& x,
                    const KinematicsCache<double>& cache,
                    RigidBodyTree<double>* tree) override;
  void UpdateJ(const Eigen::VectorXd& x,
               const KinematicsCache<double>& cache,
               RigidBodyTree<double>* tree) override;
  void UpdateJdotV(const Eigen::VectorXd& x,
                   const KinematicsCache<double>& cache,
                   RigidBodyTree<double>* tree) override;
};



}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
