#pragma once

#include <string>
#include <vector>
#include <drake/multibody/plant/multibody_plant.h>
#include "examples/franka/systems/franka_kinematics_vector.h"

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/timestamped_vector.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/state_vector.h"

namespace dairlib {
namespace systems {

/// Outputs a lcmt_timestamped_saved_traj
class FrankaKinematics : public drake::systems::LeafSystem<double> {
 public:
  explicit FrankaKinematics(const drake::multibody::MultibodyPlant<double>& franka_plant,
                            drake::systems::Context<double>* franka_context,
                            const drake::multibody::MultibodyPlant<double>& object_plant,
                            drake::systems::Context<double>* object_context,
                            const std::string& end_effector_name,
                            const std::string& object_name,
                            bool include_end_effector_orientation);

  const drake::systems::InputPort<double>& get_input_port_object_state() const {
    return this->get_input_port(object_state_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_franka_state() const {
    return this->get_input_port(franka_state_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_lcs_state() const {
    return this->get_output_port(lcs_state_port_);
  }

 private:
  void ComputeLCSState(
      const drake::systems::Context<double>& context,
      FrankaKinematicsVector<double>* output_traj) const;

  drake::systems::InputPortIndex franka_state_port_;
  drake::systems::InputPortIndex object_state_port_;
  drake::systems::OutputPortIndex lcs_state_port_;

  int num_end_effector_positions_;
  int num_object_positions_;
  int num_end_effector_velocities_;
  int num_object_velocities_;

  const drake::multibody::MultibodyPlant<double>& franka_plant_;
  drake::systems::Context<double>* franka_context_;
  const drake::multibody::MultibodyPlant<double>& object_plant_;
  drake::systems::Context<double>* object_context_;
  const drake::multibody::Frame<double>& world_;
  std::string end_effector_name_;
  std::string object_name_;
  const bool include_end_effector_orientation_;
};

}  // namespace systems
}  // namespace dairlib
