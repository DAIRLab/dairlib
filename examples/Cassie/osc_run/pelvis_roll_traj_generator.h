#pragma once

#include <drake/multibody/plant/multibody_plant.h>

#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::examples::osc {

class PelvisRollTrajGenerator : public drake::systems::LeafSystem<double> {
 public:
  PelvisRollTrajGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      drake::trajectories::PiecewisePolynomial<double>& hip_roll_traj, int axis,
      const std::string& system_name);

  PelvisRollTrajGenerator(const drake::multibody::MultibodyPlant<double>& plant,
                          drake::systems::Context<double>* context,
                          double offset, int axis,
                          const std::string& system_name);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_clock_input_port() const {
    return this->get_input_port(clock_port_);
  }

 private:
  drake::trajectories::PiecewisePolynomial<double> GeneratePelvisTraj(
      const systems::OutputVector<double>* robot_output, double t,
      int fsm_state) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;

  // pelvis trajectory
  drake::trajectories::PiecewisePolynomial<double> hip_roll_traj_;
  double neutral_offset_;

  // A list of pairs of contact body frame and contact point
  int axis_;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex fsm_port_;
  drake::systems::InputPortIndex clock_port_;
};

}  // namespace dairlib::examples::osc
