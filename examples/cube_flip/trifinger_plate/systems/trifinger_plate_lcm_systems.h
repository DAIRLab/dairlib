#pragma once

#include <map>
#include <string>
#include <vector>

#include <dairlib/lcmt_object_state.hpp>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "systems/framework/output_vector.h"
#include "systems/framework/state_vector.h"
#include "systems/framework/timestamped_vector.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"

using drake::systems::InputPort;
using drake::systems::OutputPort;
using drake::systems::InputPortIndex;
using drake::systems::OutputPortIndex;
using drake::multibody::ModelInstanceIndex;
using drake::systems::Context;

namespace dairlib {

using systems::SubvectorPassThrough;

class TrifingerPlateStateMerger : public drake::systems::LeafSystem<double> {
 public:
  explicit TrifingerPlateStateMerger(
      const drake::multibody::MultibodyPlant<double>& plant, 
      ModelInstanceIndex trifinger_index, ModelInstanceIndex plate_index, 
      const bool publish_efforts = false);

  const drake::systems::InputPort<double>& get_input_port_trifinger() const {
    return this->get_input_port(trifinger_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_plate() const {
    return this->get_input_port(plate_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_effort() const {
    return this->get_input_port(effort_port_);
  }

 private:
  void Merge(const Context<double>& context,
             dairlib::lcmt_robot_output* state_msg) const;

  ModelInstanceIndex trifinger_index_;
  ModelInstanceIndex plate_index_;

  InputPortIndex trifinger_port_;
  InputPortIndex plate_port_;
  InputPortIndex effort_port_;
  OutputPortIndex output_port_;

  int n_q_trifinger_;
  int n_v_trifinger_;
  int n_q_plate_;
  int n_v_plate_;
  int n_u_;

  int trifinger_positions_start_idx_;
  int trifinger_velocities_start_idx_;
  int plate_positions_start_idx_;
  int plate_velocities_start_idx_;

  std::vector<std::string> ordered_trifinger_position_names_;
  std::vector<std::string> ordered_trifinger_velocity_names_;
  std::map<std::string, int> trifinger_position_index_map_;
  std::map<std::string, int> trifinger_velocity_index_map_;

  std::vector<std::string> ordered_plate_position_names_;
  std::vector<std::string> ordered_plate_velocity_names_;
  std::map<std::string, int> plate_position_index_map_;
  std::map<std::string, int> plate_velocity_index_map_;

  std::vector<std::string> ordered_effort_names_;
  std::map<std::string, int> effort_index_map_;

  bool publish_efforts_;

};

SubvectorPassThrough<double>* AddActuationRecieverAndStateSenderTrifingerPlate(
    drake::systems::DiagramBuilder<double>* builder,
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::lcm::LcmInterfaceSystem* lcm, std::string actuator_channel,
    std::string state_channel, double publish_rate,
    drake::multibody::ModelInstanceIndex trifinger_index,
    drake::multibody::ModelInstanceIndex plate_index, 
    bool publish_efforts = true, double actuator_delay = 0);


} // namespace dairlib