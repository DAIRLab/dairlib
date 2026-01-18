#pragma once

#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <drake/multibody/plant/multibody_plant.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "drake/systems/framework/leaf_system.h"

using dairlib::lcmt_timestamped_saved_traj;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Vector3d;

namespace dairlib {

class TimedGate : public drake::systems::LeafSystem<double> {
 public:
  explicit TimedGate(double time_to_switch);

  const drake::systems::InputPort<double>& get_input_port_c3_actor() const {
    return this->get_input_port(c3_actor_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_c3_object() const {
    return this->get_input_port(c3_object_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_ic3_actor() const {
    return this->get_input_port(ic3_actor_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_ic3_object() const {
    return this->get_input_port(ic3_object_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_actor() const {
    return this->get_output_port(actor_output_port);
  }
  const drake::systems::OutputPort<double>& get_output_port_object() const {
    return this->get_output_port(object_output_port);
  }

 private:
   void OutputActorTrajectory(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output_traj) const;

  void OutputObjectTrajectory(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* output_traj) const;

  drake::systems::EventStatus SetFirstCallTime(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::InputPortIndex c3_actor_port_;
  drake::systems::InputPortIndex c3_object_port_;
  drake::systems::InputPortIndex ic3_actor_port_;
  drake::systems::InputPortIndex ic3_object_port_;

  drake::systems::OutputPortIndex actor_output_port;
  drake::systems::OutputPortIndex object_output_port;

  double time_to_switch_;
  double t0_idx_;
  mutable bool called_;

};

} // namespace dairlib