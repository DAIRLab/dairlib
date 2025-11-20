#pragma once

#include <string>
#include <vector>

#include <drake/multibody/plant/multibody_plant.h>

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/state_vector.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/systems/framework/leaf_system.h"

using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::InputPort;
using drake::systems::InputPortIndex;
using drake::systems::OutputPort;
using drake::systems::OutputPortIndex;
using Eigen::VectorXd;

namespace dairlib {

using systems::OutputVector;
using systems::StateVector;
using systems::TimestampedVector;

namespace systems {

/// A system that takes in robot and object states and creates an LCS state from
/// them in order [q_robot, q_object, v_robot, v_object], with no representation
/// change from the input states.  A more complicated version of this is
/// FrankaKinematics, which converts Franka joint angles to end effector
/// position or pose, and which can handle multiple objects.
/// Outputs a lcmt_timestamped_saved_traj
class SimpleRobotObjectKinematics : public drake::systems::LeafSystem<double> {
 public:
  explicit SimpleRobotObjectKinematics(
      const MultibodyPlant<double>& plant, const ModelInstanceIndex& robot_idx,
      const ModelInstanceIndex& object_idx,
      const bool& enforce_msg_from_all_systems = true);

  const InputPort<double>& get_input_port_robot_state() const {
    return this->get_input_port(robot_state_port_);
  }

  const InputPort<double>& get_input_port_object_state() const {
    return this->get_input_port(object_state_port_);
  }

  const OutputPort<double>& get_output_port_lcs_state() const {
    return this->get_output_port(lcs_state_port_);
  }

 private:
  void ComputeLCSState(const drake::systems::Context<double>& context,
                       TimestampedVector<double>* output_traj) const;

  InputPortIndex robot_state_port_;
  InputPortIndex object_state_port_;
  OutputPortIndex lcs_state_port_;

  const int num_robot_positions_;
  const int num_object_positions_;
  const int num_robot_velocities_;
  const int num_object_velocities_;
  const bool enforce_msg_from_all_systems_;
};

}  // namespace systems
}  // namespace dairlib
