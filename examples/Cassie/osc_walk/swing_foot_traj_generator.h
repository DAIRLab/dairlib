#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "examples/Cassie/osc_walk/walking_event_based_fsm.h"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::examples::osc_walk {

class SwingFootTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  SwingFootTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      const std::string& stance_foot_name, bool isLeftFoot,
      const drake::trajectories::PiecewisePolynomial<double>& foot_traj,
      double time_offset = 0.0);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::trajectories::PiecewisePolynomial<double> generateFootTraj(
      const drake::systems::Context<double>& context, const Eigen::VectorXd& x,
      double t) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::Frame<double>& world_;
  const drake::multibody::Frame<double>& stance_foot_frame_;
  drake::systems::Context<double>* context_;


  drake::trajectories::PiecewisePolynomial<double> foot_traj_;
  double time_offset_;
  // fsm state during which this trajectory describes the stance foot
  FSM_STATE active_state_;

  int state_port_;
  int fsm_port_;

  int fsm_idx_;
  int time_shift_idx_;
  int x_offset_idx_;
};

}  // namespace dairlib::examples::osc_walk
