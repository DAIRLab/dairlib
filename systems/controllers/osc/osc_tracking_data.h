#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"

#include "systems/framework/output_vector.h"

#include "systems/controllers/osc/osc_user_defined_traj.h"

namespace dairlib {
namespace systems {
namespace controllers {

/// OscTrackingData is a virtual class

/// Input of the constructor:
/// - dimension of the output/trajectory
/// - gains of PD controller
/// - cost weight
/// - a flag indicating the trajectory is a constant
/// - a flag indicating the trajecotry has exponential term (that is, the traj
///   is of ExponentialPlusPiecewisePolynomial class)

/// Cost:
///   0.5 * (J_*dv + JdotV - y_command)^T * W_ * (J_*dv + JdotV - y_command),
///   where dv is the decision variable of QP, and
///   y_commend = ddy_des_ + K_p_ * (error_y_) + K_d_ * (dy_des_ - J_ * v)
/// We ignore the cosntant term in cost function, since it doesn't affect
/// solution.

/// Most of the time, error_y_ = y_des_ - y_. The exception happends when we are
/// tracking rotation trajectory.

/// error_y_, JdotV and J_ are implemented in the derived class.

/// Users can implement their own derived classes if the current implementation
/// here is not comprehensive enough.

/// Two methods for the users:
/// - SetConstantTraj(), to set constant trajectory value
/// - SetNoControlPeriod() to set a period of not tracking the desired traj (the
///   period starts when the finite state machine switches to a new state)

class OscTrackingData {
 public:
  OscTrackingData(std::string name, int n_r,
                  Eigen::MatrixXd K_p,
                  Eigen::MatrixXd K_d,
                  Eigen::MatrixXd W,
                  bool traj_is_const,
                  bool traj_has_exp);

  OscTrackingData() {}  // Default constructor

  // Setters
  void SetConstantTraj(Eigen::VectorXd v) {fixed_position_ = v;}
  void SetNoControlPeriod(double duration) {period_of_no_control_ = duration;}

  // Updater and getters used by osc block
  bool Update(Eigen::VectorXd x_w_spr,
              KinematicsCache<double>& cache_w_spr,
              const RigidBodyTree<double>& tree_w_spr,
              Eigen::VectorXd x_wo_spr,
              KinematicsCache<double>& cache_wo_spr,
              const RigidBodyTree<double>& tree_wo_spr,
              const drake::trajectories::Trajectory<double>& traj, double t,
              int finite_state_machine_state,
              double time_since_last_state_switch);
  Eigen::VectorXd GetOutput() {return y_;}
  Eigen::MatrixXd GetJ() {return J_;}
  Eigen::VectorXd GetJdotTimesV() {return JdotV_;}
  Eigen::VectorXd GetCommandOutput() {return ddy_command_;}
  Eigen::MatrixXd GetWeight() {return W_;}
  // void UpdatePGain(Eigen::MatrixXd K_p) {K_p_ = K_p;}
  // void UpdateDGain(Eigen::MatrixXd K_d) {K_d_ = K_d;}
  // void UpdateWeight(Eigen::MatrixXd W);

  // Getters
  std::string GetName() {return name_;};
  int GetTrajDim() {return n_r_;};
  bool TrajIsConst() {return traj_is_const_;}
  bool TrajHasExp() {return traj_has_exp_;}
  Eigen::VectorXd GetFixedPosition() {return fixed_position_;}
  bool GetTrackOrNot() {return track_at_current_step_;}

  // Print feedback and desired values
  void PrintFeedbackAndDesiredValues(Eigen::VectorXd dv);

  // Finalize and ensure that users construct OscTrackingData class correctly.
  // (called in OSC constructor)
  void CheckOscTrackingData();

 protected:
  int GetStateIdx() {return state_idx_;};

  // Feedback output, jacobian and dJ/dt * v
  Eigen::VectorXd error_y_;
  Eigen::VectorXd y_;
  Eigen::VectorXd dy_;
  Eigen::MatrixXd J_;
  Eigen::VectorXd JdotV_;

  // Desired output
  Eigen::VectorXd y_des_;
  Eigen::VectorXd dy_des_;
  Eigen::VectorXd ddy_des_;

  // `state_` is the finite state machine state when the tracking is enabled
  // If `state_` is empty, then the tracking is always on.
  std::vector<int> state_;

 private:
  // Check if we should do tracking in the current state
  void UpdateTrackingFlag(int finite_state_machine_state,
                  double time_since_last_state_switch);

  // Updaters of feedback output, jacobian and dJ/dt * v
  virtual void UpdateYAndError(const Eigen::VectorXd& x_w_spr,
                               KinematicsCache<double>& cache_w_spr,
                               const RigidBodyTree<double>& tree_w_spr) = 0;
  virtual void UpdateYdot(const Eigen::VectorXd& x_w_spr,
                          KinematicsCache<double>& cache_w_spr,
                          const RigidBodyTree<double>& tree_w_spr) = 0;
  virtual void UpdateJ(const Eigen::VectorXd& x_wo_spr,
                       KinematicsCache<double>& cache_wo_spr,
                       const RigidBodyTree<double>& tree_wo_spr) = 0;
  virtual void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                           KinematicsCache<double>& cache_wo_spr,
                           const RigidBodyTree<double>& tree_wo_spr) = 0;
  // Finalize and ensure that users construct OscTrackingData derived class
  // correctly.
  virtual void CheckDerivedOscTrackingData() = 0;

  // Trajectory name
  std::string name_;

  // Dimension of the traj
  int n_r_;

  // PD control gains
  Eigen::MatrixXd K_p_;
  Eigen::MatrixXd K_d_;

  // Cost weights
  Eigen::MatrixXd W_;

  // Trajectory info
  bool traj_is_const_;
  bool traj_has_exp_;

  // `fixed_position_` stores the fixed position and pass it to input port if
  // the traj is a const
  Eigen::VectorXd fixed_position_;

  // A period when we don't apply control
  // (starting at the time when fsm switches to a new state)
  double period_of_no_control_ = 0;  // Unit: seconds

  // cache
  bool track_at_current_step_;
  int state_idx_ = 0;
  Eigen::VectorXd ddy_command_;
};


// TaskSpaceTrackingData is still a virtual class
class TaskSpaceTrackingData : public OscTrackingData {
 public:
  TaskSpaceTrackingData(std::string name, int n_r,
                        Eigen::MatrixXd K_p,
                        Eigen::MatrixXd K_d,
                        Eigen::MatrixXd W,
                        bool traj_is_const,
                        bool traj_has_exp);

  TaskSpaceTrackingData() {}  // Default constructor

 protected:
  // `body_index_w_spr` is the index of the body
  // `body_index_wo_spr` is the index of the body
  // If `body_index_w_spr_` is empty, `body_index_wo_spr_` replaces it.
  std::vector<int> body_index_w_spr_;
  std::vector<int> body_index_wo_spr_;

 private:
};


/// TransTaskSpaceTrackingData is used when we want to track a trajectory
/// (translational position) in the task space.

/// `track_center_of_mass_` should be set to true when we track center of mass.

/// If `track_center_of_mass_` is false, AddPointToTrack() should be called
/// to specify what is the point that tracks the desired trajectory
class TransTaskSpaceTrackingData final : public TaskSpaceTrackingData {
 public:
  TransTaskSpaceTrackingData(std::string name, int n_r,
                             Eigen::MatrixXd K_p,
                             Eigen::MatrixXd K_d,
                             Eigen::MatrixXd W,
                             bool traj_is_const = false,
                             bool traj_has_exp = false,
                             bool track_center_of_mass = false);

  TransTaskSpaceTrackingData() {}  // Default constructor

  void AddPointToTrack(int body_index_wo_spr,
      Eigen::Vector3d pt_on_body = Eigen::Vector3d::Zero());
  void AddStateAndPointToTrack(int state, int body_index_wo_spr,
      Eigen::Vector3d pt_on_body = Eigen::Vector3d::Zero());
  void AddPointToTrack(int body_index_w_spr, int body_index_wo_spr,
      Eigen::Vector3d pt_on_body = Eigen::Vector3d::Zero());
  void AddStateAndPointToTrack(int state, int body_index_w_spr,
      int body_index_wo_spr,
      Eigen::Vector3d pt_on_body = Eigen::Vector3d::Zero());

 private:
  void UpdateYAndError(const Eigen::VectorXd& x_w_spr,
                       KinematicsCache<double>& cache_w_spr,
                       const RigidBodyTree<double>& tree_w_spr) final;
  void UpdateYdot(const Eigen::VectorXd& x_w_spr,
                  KinematicsCache<double>& cache_w_spr,
                  const RigidBodyTree<double>& tree_w_spr) final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               KinematicsCache<double>& cache_wo_spr,
               const RigidBodyTree<double>& tree_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   KinematicsCache<double>& cache_wo_spr,
                   const RigidBodyTree<double>& tree_wo_spr) final;

  void CheckDerivedOscTrackingData() final;

  // Whether or not we are tracking the center of mass of the robot
  bool track_center_of_mass_;

  // `pt_on_body` is the position w.r.t. the origin of the body
  std::vector<Eigen::Vector3d> pt_on_body_;
};


/// RotTaskSpaceTrackingData is used when we want to track a trajectory
/// (rotational position) in the task space. The desired position must be
/// expressed in quaternion (a 4d vector).

/// AddFrameToTrack() should be called to specify what is the frame that
/// follows the desired trajectory
class RotTaskSpaceTrackingData final : public TaskSpaceTrackingData {
 public:
  RotTaskSpaceTrackingData(std::string name, int n_r,
                           Eigen::MatrixXd K_p,
                           Eigen::MatrixXd K_d,
                           Eigen::MatrixXd W,
                           bool traj_is_const = false,
                           bool traj_has_exp = false);

  RotTaskSpaceTrackingData() {}  // Default constructor

  void AddFrameToTrack(int body_index_wo_spr,
      Eigen::Isometry3d frame_pose = Eigen::Isometry3d::Identity());
  void AddStateAndFrameToTrack(int state, int body_index_wo_spr,
      Eigen::Isometry3d frame_pose = Eigen::Isometry3d::Identity());
  void AddFrameToTrack(int body_index_w_spr, int body_index_wo_spr,
      Eigen::Isometry3d frame_pose = Eigen::Isometry3d::Identity());
  void AddStateAndFrameToTrack(int state, int body_index_w_spr,
      int body_index_wo_spr,
      Eigen::Isometry3d frame_pose = Eigen::Isometry3d::Identity());

 private:
  void UpdateYAndError(const Eigen::VectorXd& x_w_spr,
                       KinematicsCache<double>& cache_w_spr,
                       const RigidBodyTree<double>& tree_w_spr) final;
  void UpdateYdot(const Eigen::VectorXd& x_w_spr,
                  KinematicsCache<double>& cache_w_spr,
                  const RigidBodyTree<double>& tree_w_spr) final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               KinematicsCache<double>& cache_wo_spr,
               const RigidBodyTree<double>& tree_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   KinematicsCache<double>& cache_wo_spr,
                   const RigidBodyTree<double>& tree_wo_spr) final;

  void CheckDerivedOscTrackingData() final;

  // frame_pose_ represents the pose of the frame (w.r.t. the body's frame)
  // which follows the desired rotation.
  std::vector<Eigen::Isometry3d> frame_pose_;
};


/// JointSpaceTrackingData is used when we want to track a trajectory
/// in the joint space.

/// AddJointToTrack() should be called to specify which joint to track.
/// Note that one instance of `JointSpaceTrackingData` allows to track 1 joint.
class JointSpaceTrackingData final : public OscTrackingData {
 public:
  JointSpaceTrackingData(std::string name, int n_r,
                         Eigen::MatrixXd K_p,
                         Eigen::MatrixXd K_d,
                         Eigen::MatrixXd W,
                         bool traj_is_const = false,
                         bool traj_has_exp = false);

  JointSpaceTrackingData() {}  // Default constructor

  void AddJointToTrack(int joint_pos_idx_wo_spr,
                       int joint_vel_idx_wo_spr);
  void AddStateAndJointToTrack(int state,
                               int joint_pos_idx_wo_spr,
                               int joint_vel_idx_wo_spr);
  void AddJointToTrack(int joint_pos_idx_w_spr,
                       int joint_vel_idx_w_spr,
                       int joint_pos_idx_wo_spr,
                       int joint_vel_idx_wo_spr);
  void AddStateAndJointToTrack(int state,
                               int joint_pos_idx_w_spr,
                               int joint_vel_idx_w_spr,
                               int joint_pos_idx_wo_spr,
                               int joint_vel_idx_wo_spr);

 private:
  void UpdateYAndError(const Eigen::VectorXd& x_w_spr,
                       KinematicsCache<double>& cache_w_spr,
                       const RigidBodyTree<double>& tree_w_spr) final;
  void UpdateYdot(const Eigen::VectorXd& x_w_spr,
                  KinematicsCache<double>& cache_w_spr,
                  const RigidBodyTree<double>& tree_w_spr) final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               KinematicsCache<double>& cache_wo_spr,
               const RigidBodyTree<double>& tree_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   KinematicsCache<double>& cache_wo_spr,
                   const RigidBodyTree<double>& tree_wo_spr) final;

  void CheckDerivedOscTrackingData() final;

  // `joint_pos_idx_wo_spr` is the index of the joint position
  // `joint_vel_idx_wo_spr` is the index of the joint velocity
  std::vector<int> joint_pos_idx_w_spr_;
  std::vector<int> joint_vel_idx_w_spr_;
  std::vector<int> joint_pos_idx_wo_spr_;
  std::vector<int> joint_vel_idx_wo_spr_;
};


class AbstractTrackingData final : public OscTrackingData {
 public:
  AbstractTrackingData(std::string name, int n_r,
                       Eigen::MatrixXd K_p,
                       Eigen::MatrixXd K_d,
                       Eigen::MatrixXd W,
                       OscUserDefinedTraj* user_defined_traj);

  AbstractTrackingData() {}  // Default constructor

 private:
  void UpdateYAndError(const Eigen::VectorXd& x_w_spr,
                       KinematicsCache<double>& cache_w_spr,
                       const RigidBodyTree<double>& tree_w_spr) final;
  void UpdateYdot(const Eigen::VectorXd& x_w_spr,
                  KinematicsCache<double>& cache_w_spr,
                  const RigidBodyTree<double>& tree_w_spr) final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               KinematicsCache<double>& cache_wo_spr,
               const RigidBodyTree<double>& tree_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   KinematicsCache<double>& cache_wo_spr,
                   const RigidBodyTree<double>& tree_wo_spr) final;

  void CheckDerivedOscTrackingData() final;

  OscUserDefinedTraj* user_defined_traj_;
};



}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
