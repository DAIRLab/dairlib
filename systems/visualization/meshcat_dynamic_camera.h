#pragma once

#include <drake/geometry/meshcat.h>
#include <drake/systems/framework/context.h>

#include "drake/multibody/plant/multibody_plant.h"

namespace dairlib {

/// Receives the output of an MPC planner as a lcmt_timestamped_saved_traj,
/// and draws it through meshcat.
class MeshcatDynamicCamera : public drake::systems::LeafSystem<double> {
 public:
  explicit MeshcatDynamicCamera(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* plant_context,
      const std::shared_ptr<drake::geometry::Meshcat>&,
      const drake::multibody::RigidBodyFrame<double>& body_frame_to_track);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }

 private:
  drake::systems::EventStatus UpdateMeshcat(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  drake::systems::InputPortIndex state_port_;
  // Kinematic calculations
  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* plant_context_;
  const drake::multibody::RigidBodyFrame<double>& body_frame_to_track_;

  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
};

}  // namespace dairlib