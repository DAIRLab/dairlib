#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/common/trajectories/trajectory.h>

#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {
namespace controllers {

/// OscTrackingData is a virtual class

/// Input of the constructor:
/// - dimension of the output/trajectory
/// - gains of PD controller
/// - cost weight
/// - the multibody plants used for the OSC controller

/// Cost:
///   0.5 * (J_*dv + JdotV - y_command)^T * W_ * (J_*dv + JdotV - y_command),
///   where dv is the decision variable of QP, and
///   y_commend = ddy_des_ + K_p_ * (error_y_) + K_d_ * (dy_des_ - J_ * v)
/// We ignore the constant term in cost function, since it doesn't affect
/// solution.

/// Most of the time, error_y_ = y_des_ - y_. The exception happens when we are
/// tracking rotation trajectory.

/// error_y_, JdotV and J_ are implemented in the derived class.

/// Users can implement their own derived classes if the current implementation
/// here is not comprehensive enough.

class OscTrackingDataMBP {
 public:
  OscTrackingDataMBP(
      const std::string& name, int n_r, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>* plant_w_spr,
      const drake::multibody::MultibodyPlant<double>* plant_wo_spr);

  // Getters for debugging
  Eigen::VectorXd GetY() { return y_; }
  Eigen::VectorXd GetYDes() { return y_des_; }
  Eigen::VectorXd GetErrorY() { return error_y_; }
  Eigen::VectorXd GetDy() { return dy_; }
  Eigen::VectorXd GetDyDes() { return dy_des_; }
  Eigen::VectorXd GetErrorDy() { return error_dy_; }
  Eigen::VectorXd GetDdyDes() { return ddy_des_; }
  Eigen::VectorXd GetDdyDesConverted() { return ddy_des_converted_; }
  Eigen::VectorXd GetDdyCommandSol() { return ddy_command_sol_; }

  // Update() updates the caches. It does the following things in order:
  //  - update track_at_current_state_
  //  - update desired output
  //  - update feedback output (Calling virtual methods)
  //  - update command output (desired output with pd control)
  // Inputs/Arguments:
  //  - `x_w_spr`, state of the robot (with spring)
  //  - `context_w_spr`, plant context of the robot (without spring)
  //  - `x_wo_spr`, state of the robot (with spring)
  //  - `context_wo_spr`, plant context of the robot (without spring)
  //  - `traj`, desired trajectory
  //  - `t`, current time
  //  - `finite_state_machine_state`, current finite state machine state
  bool Update(const Eigen::VectorXd& x_w_spr,
              drake::systems::Context<double>& context_w_spr,
              const Eigen::VectorXd& x_wo_spr,
              drake::systems::Context<double>& context_wo_spr,
              const drake::trajectories::Trajectory<double>& traj, double t,
              int finite_state_machine_state);

  // Getters used by osc block
  Eigen::VectorXd GetOutput() { return y_; }
  Eigen::MatrixXd GetJ() { return J_; }
  Eigen::VectorXd GetJdotTimesV() { return JdotV_; }
  Eigen::VectorXd GetDdyCommand() { return ddy_command_; }
  Eigen::MatrixXd GetWeight() { return W_; }

  // Getters
  std::string GetName() { return name_; };
  int GetTrajDim() { return n_r_; };
  bool IsActive() { return track_at_current_state_; }

  void SaveDdyCommandSol(const Eigen::VectorXd& dv);

  // Print feedback and desired values
  void PrintFeedbackAndDesiredValues(const Eigen::VectorXd& dv);

  // Finalize and ensure that users construct OscTrackingDataMBP class
  // correctly.
  void CheckOscTrackingData();

 protected:
  int GetStateIdx() { return state_idx_; };
  void AddState(int state);

  // Feedback output, Jacobian and dJ/dt * v
  Eigen::VectorXd error_y_;
  Eigen::VectorXd error_dy_;
  Eigen::VectorXd y_;
  Eigen::VectorXd dy_;
  Eigen::MatrixXd J_;
  Eigen::VectorXd JdotV_;

  // Desired output
  Eigen::VectorXd y_des_;
  Eigen::VectorXd dy_des_;
  Eigen::VectorXd ddy_des_;
  Eigen::VectorXd ddy_des_converted_;

  // Commanded acceleration after feedback terms
  Eigen::VectorXd ddy_command_;
  // Osc solution
  Eigen::VectorXd ddy_command_sol_;

  // `state_` is the finite state machine state when the tracking is enabled
  // If `state_` is empty, then the tracking is always on.
  std::vector<int> state_;

  /// OSC calculates feedback positions/velocities from `plant_w_spr_`,
  /// but in the optimization it uses `plant_wo_spr_`. The reason of using
  /// MultibodyPlant without springs is that the OSC cannot track desired
  /// acceleration instantaneously when springs exist. (relative degrees of 4)
  const drake::multibody::MultibodyPlant<double>* plant_w_spr_;
  const drake::multibody::MultibodyPlant<double>* plant_wo_spr_;

  // World frames
  const drake::multibody::BodyFrame<double>& world_w_spr_;
  const drake::multibody::BodyFrame<double>& world_wo_spr_;

 private:
  // Check if we should do tracking in the current state
  void UpdateTrackingFlag(int finite_state_machine_state);

  // Updaters of feedback output, jacobian and dJ/dt * v
  virtual void UpdateYAndError(
      const Eigen::VectorXd& x_w_spr,
      drake::systems::Context<double>& context_w_spr) = 0;
  virtual void UpdateYdotAndError(
      const Eigen::VectorXd& x_w_spr,
      drake::systems::Context<double>& context_w_spr) = 0;
  virtual void UpdateYddotDes() = 0;
  virtual void UpdateJ(const Eigen::VectorXd& x_wo_spr,
                       drake::systems::Context<double>& context_wo_spr) = 0;
  virtual void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                           drake::systems::Context<double>& context_wo_spr) = 0;

  // Finalize and ensure that users construct OscTrackingDataMBP derived class
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

  // Store whether or not the tracking data is active
  bool track_at_current_state_;
  int state_idx_ = 0;
};

/// ComTrackingData is used when we want to track center of mass trajectory.
class ComTrackingDataMBP final : public OscTrackingDataMBP {
 public:
  ComTrackingDataMBP(
      const std::string& name, int n_r, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>* plant_w_spr,
      const drake::multibody::MultibodyPlant<double>* plant_wo_spr);

  //  ComTrackingDataMBP() {}  // Default constructor

  // If state is not specified, it will track COM for all states
  void AddStateToTrack(int state);

 private:
  void UpdateYAndError(const Eigen::VectorXd& x_w_spr,
                       drake::systems::Context<double>& context_w_spr) final;
  void UpdateYdotAndError(const Eigen::VectorXd& x_w_spr,
                          drake::systems::Context<double>& context_w_spr) final;
  void UpdateYddotDes() final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               drake::systems::Context<double>& context_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   drake::systems::Context<double>& context_wo_spr) final;

  void CheckDerivedOscTrackingData() final;
};

// TaskSpaceTrackingDataMBP is still a virtual class
class TaskSpaceTrackingDataMBP : public OscTrackingDataMBP {
 public:
  TaskSpaceTrackingDataMBP(
      const std::string& name, int n_r, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>* plant_w_spr,
      const drake::multibody::MultibodyPlant<double>* plant_wo_spr);

 protected:
  // `body_index_w_spr` is the index of the body
  // `body_index_wo_spr` is the index of the body
  // If `body_index_w_spr_` is empty, `body_index_wo_spr_` replaces it.
  std::vector<drake::multibody::BodyIndex> body_index_w_spr_;
  std::vector<drake::multibody::BodyIndex> body_index_wo_spr_;
  std::vector<const drake::multibody::BodyFrame<double>*> body_frames_w_spr_;
  std::vector<const drake::multibody::BodyFrame<double>*> body_frames_wo_spr_;
};

/// TransTaskSpaceTrackingData is used when we want to track a trajectory
/// (translational position) in the task space.

/// AddPointToTrack() should be called to specify what is the point that
/// follows the desired trajectory.

/// If users want to track the trajectory only in some states of the finite
/// state machine, they should use AddStateAndPointToTrack().
/// Also, at most one point (of the body) can follow the desired trajectory, so
/// state_ elements can not repeat, and the length of state_ must be the same as
/// pt_on_body_'s if state_ is not empty.
/// This also means that AddPointToTrack and AddStateAndPointToTrack cannot be
/// called one after another for the same TrackingData.
class TransTaskSpaceTrackingDataMBP final : public TaskSpaceTrackingDataMBP {
 public:
  TransTaskSpaceTrackingDataMBP(
      const std::string& name, int n_r, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>* plant_w_spr,
      const drake::multibody::MultibodyPlant<double>* plant_wo_spr);

  void AddPointToTrack(
      const std::string& body_name,
      const Eigen::Vector3d& pt_on_body = Eigen::Vector3d::Zero());
  void AddStateAndPointToTrack(
      int state, const std::string& body_name,
      const Eigen::Vector3d& pt_on_body = Eigen::Vector3d::Zero());

 private:
  void UpdateYAndError(const Eigen::VectorXd& x_w_spr,
                       drake::systems::Context<double>& context_w_spr) final;
  void UpdateYdotAndError(const Eigen::VectorXd& x_w_spr,
                          drake::systems::Context<double>& context_w_spr) final;
  void UpdateYddotDes() final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               drake::systems::Context<double>& context_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   drake::systems::Context<double>& context_wo_spr) final;

  void CheckDerivedOscTrackingData() final;

  // `pt_on_body` is the position w.r.t. the origin of the body
  std::vector<Eigen::Vector3d> pts_on_body_;
};

/// RotTaskSpaceTrackingData is used when we want to track a trajectory
/// (rotational position) in the task space. The desired position must be
/// expressed in quaternion (a 4d vector).

/// AddFrameToTrack() should be called to specify what is the frame that
/// follows the desired trajectory

/// If users want to track the trajectory only in some states of the finite
/// state machine, they should use AddStateAndFrameToTrack().
/// Also, at most one point (of the body) can follow the desired trajectory, so
/// state_ elements can not repeat, and the length of state_ must be the same as
/// frame_pose_'s if state_ is not empty.
/// This also means that AddFrameToTrack and AddStateAndFrameToTrack cannot be
/// called one after another for the same TrackingData.
class RotTaskSpaceTrackingDataMBP final : public TaskSpaceTrackingDataMBP {
 public:
  RotTaskSpaceTrackingDataMBP(
      const std::string& name, int n_r, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>* plant_w_spr,
      const drake::multibody::MultibodyPlant<double>* plant_wo_spr);

  void AddFrameToTrack(
      const std::string& body_name,
      const Eigen::Isometry3d& frame_pose = Eigen::Isometry3d::Identity());
  void AddStateAndFrameToTrack(
      int state, const std::string& body_name,
      const Eigen::Isometry3d& frame_pose = Eigen::Isometry3d::Identity());

 private:
  void UpdateYAndError(const Eigen::VectorXd& x_w_spr,
                       drake::systems::Context<double>& context_w_spr) final;
  void UpdateYdotAndError(const Eigen::VectorXd& x_w_spr,
                          drake::systems::Context<double>& context_w_spr) final;
  void UpdateYddotDes() final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               drake::systems::Context<double>& context_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   drake::systems::Context<double>& context_wo_spr) final;

  void CheckDerivedOscTrackingData() final;

  // frame_pose_ represents the pose of the frame (w.r.t. the body's frame)
  // which follows the desired rotation.
  std::vector<Eigen::Isometry3d> frame_pose_;
};

/// JointSpaceTrackingData is used when we want to track a trajectory
/// in the joint space.

/// AddJointToTrack() should be called to specify which joint to track.
/// Note that one instance of `JointSpaceTrackingData` allows to track 1 joint.

/// If users want to track the trajectory only in some states of the finite
/// state machine, they should use AddStateAndJointToTrack().
/// Also, at most one point (of the body) can follow the desired trajectory, so
/// state_ elements can not repeat, and the length of state_ must be the same as
/// joint_idx's if state_ is not empty.
/// This also means that AddJointToTrack and AddStateAndJointToTrack cannot be
/// called one after another for the same TrackingData.
class JointSpaceTrackingDataMBP final : public OscTrackingDataMBP {
 public:
  JointSpaceTrackingDataMBP(
      const std::string& name, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>* plant_w_spr,
      const drake::multibody::MultibodyPlant<double>* plant_wo_spr);

  void AddJointToTrack(const std::string& joint_pos_name,
                       const std::string& joint_vel_name);
  void AddStateAndJointToTrack(int state, const std::string& joint_pos_name,
                               const std::string& joint_vel_name);

 private:
  void UpdateYAndError(const Eigen::VectorXd& x_w_spr,
                       drake::systems::Context<double>& context_w_spr) final;
  void UpdateYdotAndError(const Eigen::VectorXd& x_w_spr,
                          drake::systems::Context<double>& context_w_spr) final;
  void UpdateYddotDes() final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               drake::systems::Context<double>& context_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   drake::systems::Context<double>& context_wo_spr) final;

  void CheckDerivedOscTrackingData() final;

  // `joint_pos_idx_wo_spr` is the index of the joint position
  // `joint_vel_idx_wo_spr` is the index of the joint velocity
  std::vector<int> joint_pos_idx_w_spr_;
  std::vector<int> joint_vel_idx_w_spr_;
  std::vector<int> joint_pos_idx_wo_spr_;
  std::vector<int> joint_vel_idx_wo_spr_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
