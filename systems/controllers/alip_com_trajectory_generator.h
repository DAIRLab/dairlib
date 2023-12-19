#pragma once

#include "systems/controllers/footstep_planning/alip_utils.h"

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/trajectory.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"


namespace dairlib {
namespace systems {
namespace controllers {

struct AlipComTrajGeneratorParams {
  double desired_com_height;
  std::vector<int> fsm_states;
  std::vector<alip_utils::PointOnFramed> contact_point_in_each_state;
};

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
///  @param contact_points_in_each_state, <position of the points on the bodies,
///         body frame> pairs of plant for calculating the stance foot
///         position (of each state in unordered_fsm_states). If there are two
///         or more pairs, we get the average of the positions.
/// The last three parameters must have the same size.

class AlipComTrajectoryGenerator : public drake::systems::LeafSystem<double> {
 public:
  AlipComTrajectoryGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      double desired_com_height,
      const std::vector<int>& fsm_states,
      const std::vector<alip_utils::PointOnFramed> contact_point_in_each_state);

  AlipComTrajectoryGenerator(const drake::multibody::MultibodyPlant<double>& plant,
                         drake::systems::Context<double>* context,
                         AlipComTrajGeneratorParams params) :
      AlipComTrajectoryGenerator(plant, context,
                             params.desired_com_height,
                             params.fsm_states,
                             params.contact_point_in_each_state) {}

  // Input port getters
  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm_switch_time()
  const{
    return this->get_input_port(prev_liftoff_time_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_next_fsm_switch_time()
  const {
    return this->get_input_port(next_touchdown_time_port_);
  }
  // Input port for the desired CoM height at the following touchdown relative
  // to the current stance foot
  const drake::systems::InputPort<double>& get_input_port_slope_params() const {
    return this->get_input_port(slope_params_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_com() const {
    return this->get_output_port(output_port_com_);
  }

 private:

  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;
  void CalcAlipState(const Eigen::VectorXd& x, int mode_index,
                     const drake::EigenPtr<Eigen::Vector3d>& CoM,
                     const drake::EigenPtr<Eigen::Vector3d>& L,
                     const drake::EigenPtr<Eigen::Vector3d>& stance_pos) const;

  drake::trajectories::ExponentialPlusPiecewisePolynomial<double>
  ConstructAlipComTraj(const Eigen::Vector3d& stance_foot_pos,
                       const Eigen::Vector4d& x_alip,
                       const Eigen::Vector2d& kx_ky,
                       double start_time, double end_time) const;

  void CalcComTrajFromCurrent(const drake::systems::Context<double>& context,
                              drake::trajectories::Trajectory<double>* traj) const;

  Eigen::Matrix4d CalcA(double com_z) const {
    return controllers::alip_utils::CalcA(com_z, m_);
  }

  int GetModeIdx(int fsm_state) const {
    auto it = find(fsm_states_.begin(), fsm_states_.end(), fsm_state);
    int mode_index = std::distance(fsm_states_.begin(), it);
    DRAKE_DEMAND(it != fsm_states_.end());
    return mode_index;
  }

  // Port indices
  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex fsm_port_;
  drake::systems::InputPortIndex prev_liftoff_time_port_;
  drake::systems::InputPortIndex next_touchdown_time_port_;
  drake::systems::InputPortIndex slope_params_port_;
  drake::systems::OutputPortIndex output_port_com_;

  // States
  drake::systems::DiscreteStateIndex prev_slope_idx_;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  double desired_com_height_;

  // A list of pairs of contact body frame and contact point in each FSM state
  const std::vector<int> fsm_states_;
  const std::vector<alip_utils::PointOnFramed> contact_point_in_each_state_;
  double m_;

};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
