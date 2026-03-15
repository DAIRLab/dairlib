#pragma once

#include <string>
#include <vector>
#include <drake/multibody/plant/multibody_plant.h>

#include "drake/systems/framework/leaf_system.h"
#include "systems/framework/timestamped_vector.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/state_vector.h"
#include "multibody/multibody_utils.h"

using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using Eigen::VectorXd;
using drake::multibody::ModelInstanceIndex;
using drake::systems::InputPort;
using drake::systems::OutputPort; 
using drake::systems::InputPortIndex;
using drake::systems::OutputPortIndex; 
using std::vector;

namespace dairlib {

using systems::OutputVector;
using systems::StateVector;
using systems::TimestampedVector;

/// Outputs a lcmt_timestamped_saved_traj
class TrifingerKinematics : public drake::systems::LeafSystem<double> {
 public:
  explicit TrifingerKinematics(const MultibodyPlant<double>& trifinger_plant,
                                Context<double>* trifinger_context,
                                const MultibodyPlant<double>& object_plant,
                                Context<double>* object_context,
                                const vector<std::string> end_effector_names,
                                const std::string& object_name);
 
  std::vector<const drake::systems::InputPort<double>*> get_input_ports_object_state() const {
      std::vector<const InputPort<double>*> output;
      for (InputPortIndex port : object_state_ports_) {
        output.push_back(
          &this->get_input_port(port)
        );
      } 
      return output;
  }
  const InputPort<double>& get_input_port_trifinger_state() const {
    return this->get_input_port(trifinger_state_port_);
  }

  const InputPort<double>& get_input_port_object_state() const {
    return this->get_input_port(object_state_port_);
  }

  const OutputPort<double>& get_output_port_lcs_state() const {
    return this->get_output_port(lcs_state_port_);
  }

 private:
  void ComputeLCSState(
      const drake::systems::Context<double>& context,
      TimestampedVector<double>* output_traj) const;

  InputPortIndex trifinger_state_port_;
  InputPortIndex object_state_port_;
  std::vector<InputPortIndex> object_state_ports_;
  OutputPortIndex lcs_state_port_;

  int num_end_effector_positions_;
  int num_object_positions_;
  int num_end_effector_velocities_;
  int num_object_velocities_;
  std::vector<std::string> object_names_;
  int num_objects_;

  const MultibodyPlant<double>& trifinger_plant_;
  Context<double>* trifinger_context_;
  const MultibodyPlant<double>& object_plant_;
  Context<double>* object_context_;
  const drake::multibody::Frame<double>& world_;
  vector<std::string> end_effector_names_;
  std::string object_name_;
};

}  // namespace dairlib
