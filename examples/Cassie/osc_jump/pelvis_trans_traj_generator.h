#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "examples/Cassie/osc_jump/jumping_event_based_fsm.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::examples::osc_jump {

class PelvisTransTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  PelvisTransTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      drake::trajectories::PiecewisePolynomial<double>& crouch_traj,
      const std::vector<std::pair<const Eigen::Vector3d,
                                  const drake::multibody::Frame<double>&>>&
      feet_contact_points,
      double time_offset = 0.0, JUMPING_FSM_STATE init_fsm_state = BALANCE);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }

  void SetLandingOffset(double landing_x_offset){
    landing_x_offset_ = landing_x_offset;
  }

 private:
  drake::trajectories::PiecewisePolynomial<double> GenerateBalanceTraj(
      const drake::systems::Context<double>& context, const Eigen::VectorXd& x,
      double time) const;
  drake::trajectories::PiecewisePolynomial<double> GenerateCrouchTraj(
      const Eigen::VectorXd& x, double time) const;
  drake::trajectories::PiecewisePolynomial<double> GenerateLandingTraj(
      const drake::systems::Context<double>& context, const Eigen::VectorXd& x,
      double time) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;

  drake::systems::DiscreteStateIndex prev_fsm_idx_;
  drake::systems::DiscreteStateIndex pelvis_x_offset_idx_;
  drake::systems::DiscreteStateIndex initial_pelvis_pos_idx_;
  drake::systems::DiscreteStateIndex switch_time_idx_;

  // Center of mass trajectory
  drake::trajectories::PiecewisePolynomial<double> crouch_traj_;

  // A list of pairs of contact body frame and contact point
  const std::vector<
      std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>&
      feet_contact_points_;
  double time_offset_;
  double landing_x_offset_ = 0.00;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex fsm_port_;

  // The trajectory optimization solution sets the final CoM very close to
  // rear toe contacts - this is an offset to move it closer to the center of
  // the support polygon
//  static constexpr double kLandingOffset = 0.04;  // 0.04 m (4cm)
//  static constexpr double kLandingOffset = 0.04;  // 0.04 m (4cm)
};

}  // namespace dairlib::examples::osc_jump
