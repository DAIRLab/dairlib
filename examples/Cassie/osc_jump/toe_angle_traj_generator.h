#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "examples/Cassie/osc_jump/jumping_event_based_fsm.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::cassie::osc_jump {

class FlightToeAngleTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  FlightToeAngleTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      drake::trajectories::PiecewisePolynomial<double>& toe_traj, int swing_toe_idx,
      const std::vector<std::pair<const Eigen::Vector3d,
                                  const drake::multibody::Frame<double>&>>&
      feet_contact_points,
      const std::string& traj_name);

  // Input/output ports
  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }

  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }

 private:
  drake::trajectories::PiecewisePolynomial<double> CalcToeAngle(
      Eigen::VectorXd q) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;

  drake::trajectories::PiecewisePolynomial<double> toe_traj_;
  bool use_traj_;

  int swing_toe_idx_;
  // A list of pairs of contact body frame and contact point
  const std::vector<
      std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>&
      feet_contact_points_;

  int state_port_;
  int fsm_port_;
};
}  // namespace dairlib::cassie::osc_jump
