#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <drake/common/trajectories/trajectory.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/output_vector.h"

namespace dairlib {
namespace systems {
namespace controllers {

static constexpr int kSpaceDim = 3;
static constexpr int kQuaternionDim = 4;

class OscTrackingData {
 public:
  OscTrackingData(const std::string& name, int n_y, int n_ydot,
                  const Eigen::MatrixXd& K_p, const Eigen::MatrixXd& K_d,
                  const Eigen::MatrixXd& W,
                  const drake::multibody::MultibodyPlant<double>& plant_w_spr,
                  const drake::multibody::MultibodyPlant<double>& plant_wo_spr);

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
  void Update(const Eigen::VectorXd& x_w_spr,
              const drake::systems::Context<double>& context_w_spr,
              const Eigen::VectorXd& x_wo_spr,
              const drake::systems::Context<double>& context_wo_spr,
              const drake::trajectories::Trajectory<double>& traj, double t);

  virtual void UpdateActual(
      const Eigen::VectorXd& x_wo_spr,
      const drake::systems::Context<double>& context_wo_spr);
  void UpdateDesired(const drake::trajectories::Trajectory<double>& traj,
                     double t);

  // Add this state to the list of fsm states where this tracking data is active
  void AddFiniteStateToTrack(int state);
  bool IsActive(int fsm_state) const { return state_.count(fsm_state); }

  // Getters for debugging
  const Eigen::VectorXd& GetY() const { return y_; }
  const Eigen::VectorXd& GetYDes() const { return y_des_; }
  const Eigen::VectorXd& GetErrorY() const { return error_y_; }
  const Eigen::VectorXd& GetYdot() const { return ydot_; }
  const Eigen::VectorXd& GetYdotDes() const { return ydot_des_; }
  const Eigen::VectorXd& GetErrorYdot() const { return error_ydot_; }
  const Eigen::VectorXd& GetYddotDes() const { return yddot_des_; }
  const Eigen::VectorXd& GetYddotCommandSol() const {
    return yddot_command_sol_;
  }

  // Getters used by osc block
  const Eigen::MatrixXd& GetKp() const { return K_p_; }
  const Eigen::MatrixXd& GetKd() const { return K_d_; }
  const Eigen::MatrixXd& GetJ() const { return J_; }
  const Eigen::VectorXd& GetJdotTimesV() const { return JdotV_; }
  const Eigen::VectorXd& GetYddotCommand() const { return yddot_command_; }
  const Eigen::MatrixXd& GetWeight() const { return W_; }

  // Getters
  const std::string& GetName() const { return name_; };
  const std::set<int>& GetActiveStates() { return state_; };
  int GetYDim() const { return n_y_; };
  int GetYdotDim() const { return n_ydot_; };
  const drake::multibody::MultibodyPlant<double>& plant_w_spr() const {
    return plant_w_spr_;
  };
  const drake::multibody::MultibodyPlant<double>& plant_wo_spr() const {
    return plant_wo_spr_;
  };

  void StoreYddotCommandSol(const Eigen::VectorXd& dv);

 protected:
  // Output dimension
  int n_y_;
  int n_ydot_;

  // Feedback output, Jacobian and dJ/dt * v
  Eigen::VectorXd y_;
  Eigen::VectorXd error_y_;
  Eigen::VectorXd ydot_;
  Eigen::VectorXd error_ydot_;
  Eigen::MatrixXd J_;
  Eigen::VectorXd JdotV_;

  // PD control gains
  Eigen::MatrixXd K_p_;
  Eigen::MatrixXd K_d_;

  // Desired output
  Eigen::VectorXd y_des_;
  Eigen::VectorXd ydot_des_;
  Eigen::VectorXd yddot_des_;
  Eigen::VectorXd yddot_des_converted_;

  // Commanded acceleration after feedback terms
  Eigen::VectorXd yddot_command_;
  // OSC solution
  Eigen::VectorXd yddot_command_sol_;

  // `state_` is the finite state machine state when the tracking is enabled
  // If `state_` is empty, then the tracking is always on.
  std::set<int> state_;

  /// OSC calculates feedback positions/velocities from `plant_w_spr_`,
  /// but in the optimization it uses `plant_wo_spr_`. The reason of using
  /// MultibodyPlant without springs is that the OSC cannot track desired
  /// acceleration instantaneously when springs exist. (relative degrees of 4)
  const drake::multibody::MultibodyPlant<double>& plant_w_spr_;
  const drake::multibody::MultibodyPlant<double>& plant_wo_spr_;

  // World frames
  const drake::multibody::BodyFrame<double>& world_w_spr_;
  const drake::multibody::BodyFrame<double>& world_wo_spr_;

 private:
  // Trajectory name
  std::string name_;

  // Updaters of feedback output, jacobian and dJ/dt * v
  virtual void UpdateY(
      const Eigen::VectorXd& x_w_spr,
      const drake::systems::Context<double>& context_w_spr) = 0;
  virtual void UpdateYError() = 0;
  virtual void UpdateYdot(
      const Eigen::VectorXd& x_w_spr,
      const drake::systems::Context<double>& context_w_spr) = 0;
  virtual void UpdateYdotError() = 0;
  virtual void UpdateJ(
      const Eigen::VectorXd& x_wo_spr,
      const drake::systems::Context<double>& context_wo_spr) = 0;
  virtual void UpdateJdotV(
      const Eigen::VectorXd& x_wo_spr,
      const drake::systems::Context<double>& context_wo_spr) = 0;
  virtual void UpdateYddotDes(double t) = 0;

  // Finalize and ensure that users construct OscTrackingData derived class
  // correctly.
  virtual void CheckDerivedOscTrackingData() = 0;

  // Cost weights
  Eigen::MatrixXd W_;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib