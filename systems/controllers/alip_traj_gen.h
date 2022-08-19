#pragma once

#include "multibody/multibody_utils.h"
#include "systems/controllers/control_utils.h"
#include "systems/controllers/footstep_planning/alip_utils.h"
#include "systems/framework/output_vector.h"
#include "systems/filters/s2s_kalman_filter.h"

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"


namespace dairlib {
namespace systems {

/// This class creates predicted center of mass (COM) trajectory of a bipedal
/// robot.
/// The trajectories in horizontal directions (x and y axes) are predicted, and
/// the traj in the vertical direction (z axis) starts/ends at the
/// current/desired height.
/// There is also a port which, along with predicted x and y CoM positions,
/// outputs Lx and Ly predictions using the ALIP model. The state order for
/// this trajectory is [x_com, y_com, Lx, Ly]^T


/// Constructor inputs:
///  @param plant, the MultibodyPlant
///  @param desired_com_height, desired COM height
///  @param unordered_fsm_states, vector of fsm states
///  @param unordered_state_durations, duration of each state in
///         unordered_fsm_states
///  @param contact_points_in_each_state, <position of the points on the bodies,
///         body frame> pairs of plant for calculating the stance foot
///         position (of each state in unordered_fsm_states). If there are two
///         or more pairs, we get the average of the positions.
/// The last three parameters must have the same size.

class ALIPTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  ALIPTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context, double desired_com_height,
      const std::vector<int>& unordered_fsm_states,
      const std::vector<double>& unordered_state_durations,
      const std::vector<std::vector<std::pair<
          const Eigen::Vector3d, const drake::multibody::Frame<double>&>>>&
      contact_points_in_each_state, const Eigen::MatrixXd& Q,
      const Eigen::MatrixXd& R, bool filter_alip_state = true,
      bool target_com_z = false);

  // Input port getters
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_touchdown_time()
  const {
    return this->get_input_port(touchdown_time_port_);
  }
  // Input port for the desired CoM height at the following touchdown relative
  // to the current stance foot
  const drake::systems::InputPort<double>& get_input_port_target_com_z() const {
    DRAKE_ASSERT(target_com_z_);
    return this->get_input_port(com_z_input_port_);
  }

  // Output port getters
  const drake::systems::OutputPort<double>& get_output_port_alip_state()
  const {
    return this->get_output_port(output_port_alip_state_);
  }
  const drake::systems::OutputPort<double>& get_output_port_com() const {
    return this->get_output_port(output_port_com_);
  }

 private:

  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

  int GetModeIdx(int fsm_state) const;

  void CalcAlipState(const Eigen::VectorXd& x, int mode_index,
                     const drake::EigenPtr<Eigen::Vector3d>& CoM,
                     const drake::EigenPtr<Eigen::Vector3d>& L,
                     const drake::EigenPtr<Eigen::Vector3d>& stance_pos) const;

  drake::trajectories::ExponentialPlusPiecewisePolynomial<double>
  ConstructAlipComTraj(const Eigen::Vector3d& CoM,
                       const Eigen::Vector3d& stance_foot_pos,
                       const Eigen::Vector4d& x_alip,
                       double com_z_rel_to_stance_at_next_td,
                       double start_time,
                       double end_time_of_this_fsm_state) const;

  drake::trajectories::ExponentialPlusPiecewisePolynomial<double>
  ConstructAlipStateTraj(const Eigen::Vector4d& x_alip, double com_z,
                         double start_time,
                         double end_time_of_this_fsm_state) const;

  void CalcComTrajFromCurrent(const drake::systems::Context<double>& context,
                           drake::trajectories::Trajectory<double>* traj) const;

  void CalcAlipTrajFromCurrent(const drake::systems::Context<double>& context,
                               drake::trajectories::Trajectory<double>* traj) const;

  Eigen::Matrix4d CalcA(double com_z) const {
    return controllers::alip_utils::CalcA(com_z, m_);
  }

  // Port indices
  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex fsm_port_;
  drake::systems::InputPortIndex touchdown_time_port_;
  drake::systems::InputPortIndex com_z_input_port_;

  drake::systems::OutputPortIndex output_port_alip_state_;
  drake::systems::OutputPortIndex output_port_com_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  double desired_com_height_;
  std::vector<int> unordered_fsm_states_;
  std::vector<double> unordered_state_durations_;

  // A list of pairs of contact body frame and contact point in each FSM state
  const std::vector<std::vector<std::pair<
      const Eigen::Vector3d, const drake::multibody::Frame<double>&>>>&
      contact_points_in_each_state_;
  const drake::multibody::BodyFrame<double>& world_;

  bool filter_alip_state_;
  bool target_com_z_;
  double m_;

  drake::systems::AbstractStateIndex alip_filter_idx_;
  drake::systems::DiscreteStateIndex com_z_idx_;
  drake::systems::DiscreteStateIndex prev_fsm_idx_;
  drake::systems::DiscreteStateIndex prev_foot_idx_;
};

}  // namespace systems
}  // namespace dairlib
