#pragma once
#include <string>

#include <drake/multibody/plant/multibody_plant.h>

#include "examples/trifinger/systems/trifinger_kinematics_vector.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/state_vector.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// Outputs a lcmt_timestamped_saved_traj
class TrifingerKinematics : public drake::systems::LeafSystem<double> {
 public:
  explicit TrifingerKinematics(
      const drake::multibody::MultibodyPlant<double>& trifinger_plant,
      drake::systems::Context<double>* trifinger_context,
      const drake::multibody::MultibodyPlant<double>& object_plant,
      drake::systems::Context<double>* object_context,
      const std::string& fingertip_0_name,
      const std::string& fingertip_120_name,
      const std::string& fingertip_240_name, const std::string& object_name);

  const drake::systems::InputPort<double>& get_input_port_object_state() const {
    return this->get_input_port(object_state_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_trifinger_state()
      const {
    return this->get_input_port(trifinger_state_port_);
  }

  const drake::systems::OutputPort<double>& get_output_port_lcs_state() const {
    return this->get_output_port(lcs_state_port_);
  }

 private:
  void ComputeLCSState(const drake::systems::Context<double>& context,
                       TrifingerKinematicsVector<double>* output_traj) const;

  drake::systems::InputPortIndex trifinger_state_port_;
  drake::systems::InputPortIndex object_state_port_;
  drake::systems::OutputPortIndex lcs_state_port_;

  int num_fingertip_positions_;
  int num_fingertip_velocities_;
  int num_object_positions_;
  int num_object_velocities_;

  const drake::multibody::MultibodyPlant<double>& trifinger_plant_;
  drake::systems::Context<double>* trifinger_context_;
  const drake::multibody::MultibodyPlant<double>& object_plant_;
  drake::systems::Context<double>* object_context_;
  const drake::multibody::Frame<double>& world_;
  std::string fingertip_0_name_;
  std::string fingertip_120_name_;
  std::string fingertip_240_name_;
  std::string object_name_;
};

}  // namespace systems
}  // namespace dairlib
