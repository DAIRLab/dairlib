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

  virtual ~OscTrackingData() = default;

  // Update() updates the caches. It does the following things in order:
  //  - updates the current fsm state
  //  - update the actual outputs
  //  - update the desired outputs
  //  - update the errors (Calling virtual methods)
  //  - update commanded accelerations (desired output with pd control)
  // Inputs/Arguments:
  //  - `x_w_spr`, state of the robot (with spring)
  //  - `context_w_spr`, plant context of the robot (without spring)
  //  - `x_wo_spr`, state of the robot (with spring)
  //  - `context_wo_spr`, plant context of the robot (without spring)
  //  - `traj`, desired trajectory
  //  - `t`, current time
  //  - `t`, time since the last state switch
  //  - `v_proj`, impact invariant velocity projection
  virtual void Update(const Eigen::VectorXd& x_w_spr,
                      const drake::systems::Context<double>& context_w_spr,
                      const Eigen::VectorXd& x_wo_spr,
                      const drake::systems::Context<double>& context_wo_spr,
                      const drake::trajectories::Trajectory<double>& traj,
                      double t, double t_since_state_switch, int fsm_state,
                      const Eigen::VectorXd& v_proj);

  // Add this state to the list of fsm states where this tracking data is active
  void AddFiniteStateToTrack(int state);
  bool IsActive(int fsm_state) const {
    return active_fsm_states_.count(fsm_state) || active_fsm_states_.count(-1);
  }
  void CheckOscTrackingData();

  // Set whether to use springs in the calculation of the actual outputs
  void SetSpringsInKinematicCalculation(bool use_springs_in_eval) {
    use_springs_in_eval_ = use_springs_in_eval;
  }

  // Set whether to use the impact invariant projection
  void SetImpactInvariantProjection(bool use_impact_invariant_projection) {
    impact_invariant_projection_ = use_impact_invariant_projection;
  }

  // Set whether to use the impact invariant projection
  void SetNoDerivativeFeedback(bool no_derivative_feedback) {
    no_derivative_feedback_ = no_derivative_feedback;
  }

  // Get whether to use the impact invariant projection
  bool GetImpactInvariantProjection() { return impact_invariant_projection_; }
  // Get whether to use no derivative feedback near impacts
  bool GetNoDerivativeFeedback() { return no_derivative_feedback_; }

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
  virtual const Eigen::MatrixXd& GetWeight() const { return W_; }

  // Getters
  const std::string& GetName() const { return name_; };
  const std::set<int>& GetActiveStates() { return active_fsm_states_; };
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
  virtual void UpdateActual(
      const Eigen::VectorXd& x_w_spr,
      const drake::systems::Context<double>& context_w_spr,
      const Eigen::VectorXd& x_wo_spr,
      const drake::systems::Context<double>& context_wo_spr, double t);

  // Output dimension
  int n_y_;
  int n_ydot_;

  // Current fsm state
  int fsm_state_;

  // Flags
  bool use_springs_in_eval_ = true;
  bool impact_invariant_projection_ = false;
  bool no_derivative_feedback_ = false;

  double time_through_trajectory_ = 0;

  // Actual outputs, Jacobian and dJ/dt * v
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
  std::set<int> active_fsm_states_;

  // Cost weights
  Eigen::MatrixXd W_;

  /// OSC calculates feedback positions/velocities from `plant_w_spr_`,
  /// but in the optimization it uses `plant_wo_spr_`. The reason of using
  /// MultibodyPlant without springs is that the OSC cannot track desired
  /// acceleration instantaneously when springs exist. (relative degrees of 4)
  const drake::multibody::MultibodyPlant<double>& plant_w_spr_;
  const drake::multibody::MultibodyPlant<double>& plant_wo_spr_;

  // World frames
  const drake::multibody::BodyFrame<double>& world_w_spr_;
  const drake::multibody::BodyFrame<double>& world_wo_spr_;

  // Trajectory name
  std::string name_;
 private:
  void UpdateDesired(const drake::trajectories::Trajectory<double>& traj,
                     double t, double t_since_state_switch);
  // Update actual output methods
  virtual void UpdateY(
      const Eigen::VectorXd& x_w_spr,
      const drake::systems::Context<double>& context_w_spr) = 0;
  virtual void UpdateYdot(
      const Eigen::VectorXd& x_w_spr,
      const drake::systems::Context<double>& context_w_spr) = 0;
  virtual void UpdateJ(
      const Eigen::VectorXd& x_wo_spr,
      const drake::systems::Context<double>& context_wo_spr) = 0;
  virtual void UpdateJdotV(
      const Eigen::VectorXd& x_wo_spr,
      const drake::systems::Context<double>& context_wo_spr) = 0;
  // Update error methods
  virtual void UpdateYError() = 0;
  virtual void UpdateYdotError(const Eigen::VectorXd& v_proj) = 0;
  virtual void UpdateYddotDes(double t, double t_since_state_switch) = 0;
  virtual void UpdateYddotCmd(double t, double t_since_state_switch);

  // Finalize and ensure that users construct OscTrackingData derived class
  // correctly.
  virtual void CheckDerivedOscTrackingData() = 0;

};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib