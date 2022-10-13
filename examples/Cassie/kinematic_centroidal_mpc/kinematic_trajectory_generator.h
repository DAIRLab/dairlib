#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {

class KinematicTrajectoryGenerator : public drake::systems::LeafSystem<double> {
 public:
  KinematicTrajectoryGenerator(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      const std::string& body_name,
      Eigen::Vector3d point_on_body);

  const drake::systems::InputPort<double>& get_state_input_port() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_fsm_input_port() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_state_trajectory_input_port() const {
    return this->get_input_port(state_trajectory_port_);
  }
  const drake::systems::OutputPort<double>& get_trajectory_output_port() const {
    return this->get_output_port(target_trajectory_port_);
  }

 private:

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  void CalcTraj(const drake::systems::Context<double>& context,
                drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;


  std::string body_name_;
  Eigen::Vector3d point_on_body_;

  drake::systems::InputPortIndex state_port_;
  drake::systems::InputPortIndex fsm_port_;
  drake::systems::InputPortIndex state_trajectory_port_;
  drake::systems::OutputPortIndex target_trajectory_port_;


};

}  // namespace dairlib::examples::osc_jump
