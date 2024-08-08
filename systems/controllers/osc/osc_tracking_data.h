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

struct OscTrackingDataState {
  std::string name_;
  Eigen::Matrix3d view_frame_rot_T_;

  int fsm_state_;
  double time_through_trajectory_ = 0;

  // Actual outputs, Jacobian and dJ/dt * v
  Eigen::VectorXd y_;
  Eigen::VectorXd error_y_;
  Eigen::VectorXd ydot_;
  Eigen::VectorXd error_ydot_;
  Eigen::MatrixXd J_;
  Eigen::VectorXd JdotV_;

  // Desired output
  Eigen::VectorXd y_des_;
  Eigen::VectorXd ydot_des_;
  Eigen::VectorXd yddot_des_;
  Eigen::VectorXd yddot_des_converted_;

  // Commanded acceleration after feedback terms
  Eigen::VectorXd yddot_command_;

  // Members of low-pass filter
  Eigen::VectorXd filtered_y_;
  Eigen::VectorXd filtered_ydot_;
  Eigen::MatrixXd time_varying_weight_;
  double last_timestamp_ = -1;

  Eigen::VectorXd CalcYddotCommandSol(const Eigen::VectorXd& dv) const {
      return J_ * dv + JdotV_;
  }

  /// TODO (@Brian-Acosta)
  /// Fix this hack for doing domain randomization in footstep learning project
  /// by doing it correctly
  Eigen::MatrixXd K_p_;
  Eigen::MatrixXd K_d_;

  friend std::ostream& operator<<(std::ostream& o, const OscTrackingDataState& s) {
    o << "Osc Tracking Data State" << s.name_ << "\n"
      << "\ny: " << s.y_.transpose()
      << "\nydot: " << s.ydot_.transpose()
      << "\nerror_y: " << s.error_y_.transpose()
      << "\nerror_ydot: " << s.error_ydot_.transpose()
      << "\ny_des: " << s.y_des_.transpose()
      << "\nydot_des: " << s.ydot_des_.transpose()
      << "\nyddot_des: " << s.yddot_des_converted_.transpose()
      << "\nyddot_cmd: " << s.yddot_command_.transpose() << std::endl;
    return o;
  }
};

class OscTrackingData {
 public:
  OscTrackingData(const std::string& name, int n_y, int n_ydot,
                  const Eigen::MatrixXd& K_p, const Eigen::MatrixXd& K_d,
                  const Eigen::MatrixXd& W,
                  const drake::multibody::MultibodyPlant<double>& plant);

  OscTrackingDataState AllocateState() const;

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
  virtual void Update(const Eigen::VectorXd& x,
                      const drake::systems::Context<double>& context,
                      const drake::trajectories::Trajectory<double>& traj,
                      double t, double t_since_state_switch, int fsm_state,
                      const Eigen::VectorXd& v_proj,
                      OscTrackingDataState& tracking_data_state) const;

  void MakeImpactInvariantCorrection(
      double t, double t_since_state_switch, int fsm_state,
      const Eigen::VectorXd& v_proj,
      OscTrackingDataState& tracking_data_state) const {
    UpdateYdotError(v_proj, tracking_data_state);
    UpdateYddotCmd(t, t_since_state_switch, tracking_data_state);
  }

  // Add this state to the list of fsm states where this tracking data is active
  void AddFiniteStateToTrack(int state);
  bool IsActive(int fsm_state) const {
    return active_fsm_states_.count(fsm_state) || active_fsm_states_.count(-1);
  }
  void CheckOscTrackingData();

  // Set whether to use the impact invariant projection
  void SetImpactInvariantProjection(bool use_impact_invariant_projection) {
    impact_invariant_projection_ = use_impact_invariant_projection;
  }

  // Set whether to use the impact invariant projection
  void SetNoDerivativeFeedback(bool no_derivative_feedback) {
    no_derivative_feedback_ = no_derivative_feedback;
  }

  // Get whether to use the impact invariant projection
  bool GetImpactInvariantProjection() const { return impact_invariant_projection_; }
  // Get whether to use no derivative feedback near impacts
  bool GetNoDerivativeFeedback() const { return no_derivative_feedback_; }

  // Getters used by osc block
  const Eigen::MatrixXd& GetKp() const { return K_p_; }
  const Eigen::MatrixXd& GetKd() const { return K_d_; }
  virtual const Eigen::MatrixXd& GetWeight() const { return W_; }

  // Getters
  const std::string& GetName() const { return name_; };
  const std::set<int>& GetActiveStates() { return active_fsm_states_; };
  int GetYDim() const { return n_y_; };
  int GetYdotDim() const { return n_ydot_; };

  const drake::multibody::MultibodyPlant<double>& plant() const {
    return plant_;
  };

  struct PDGains {
    Eigen::MatrixXd K_p_;
    Eigen::MatrixXd K_d_;
    int rows() const {return K_p_.rows();}
    int cols() const {return K_d_.rows();}
  };

  PDGains GetDefaultPDGains() const {
    return {K_p_, K_d_};
  }

 protected:
  virtual void UpdateActual(
      const Eigen::VectorXd& x,
      const drake::systems::Context<double>& context, double t,
      OscTrackingDataState& tracking_data_state) const;

  // Output dimension
  const int n_y_;
  const int n_ydot_;

  // PD control gains
  Eigen::MatrixXd K_p_;
  Eigen::MatrixXd K_d_;

  // Flags
  bool impact_invariant_projection_ = false;
  bool no_derivative_feedback_ = false;

  // `state_` is the finite state machine state when the tracking is enabled
  // If `state_` is empty, then the tracking is always on.
  std::set<int> active_fsm_states_;

  // Cost weights
  Eigen::MatrixXd W_;

  /// OSC calculates feedback positions/velocities from `plant_w_spr_`,
  /// but in the optimization it uses `plant_wo_spr_`. The reason of using
  /// MultibodyPlant without springs is that the OSC cannot track desired
  /// acceleration instantaneously when springs exist. (relative degrees of 4)
  const drake::multibody::MultibodyPlant<double>& plant_;

  // World frames
  const drake::multibody::RigidBodyFrame<double>& world_;

  // Trajectory name
  std::string name_;
 private:
  void UpdateDesired(const drake::trajectories::Trajectory<double>& traj,
                     double t, double t_since_state_switch,
                     OscTrackingDataState& tracking_data_state) const;
  // Update actual output methods
  virtual void UpdateY(
      const Eigen::VectorXd& x_w_spr,
      const drake::systems::Context<double>& context_w_spr,
      OscTrackingDataState& tracking_data_state) const = 0;
  virtual void UpdateYdot(
      const Eigen::VectorXd& x_w_spr,
      const drake::systems::Context<double>& context_w_spr,
      OscTrackingDataState& tracking_data_state) const = 0;
  virtual void UpdateJ(
      const Eigen::VectorXd& x_wo_spr,
      const drake::systems::Context<double>& context_wo_spr,
      OscTrackingDataState& tracking_data_state) const = 0;
  virtual void UpdateJdotV(
      const Eigen::VectorXd& x_wo_spr,
      const drake::systems::Context<double>& context_wo_spr,
      OscTrackingDataState& tracking_data_state) const = 0;

  // Update error methods
  virtual void UpdateYError(OscTrackingDataState& tracking_data_state) const = 0;
  virtual void UpdateYdotError(
      const Eigen::VectorXd& v_proj,
      OscTrackingDataState& tracking_data_state) const = 0;
  virtual void UpdateYddotDes(
      double t, double t_since_state_switch,
      OscTrackingDataState& tracking_data_state) const = 0;
  virtual void UpdateYddotCmd(
      double t, double t_since_state_switch,
      OscTrackingDataState& tracking_data_state) const;

  // Finalize and ensure that users construct OscTrackingData derived class
  // correctly.
  virtual void CheckDerivedOscTrackingData() = 0;

};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib