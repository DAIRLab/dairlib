#include "trifinger_plate_lcm_systems.h"
#include <iostream>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "multibody/multibody_utils.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/discrete_time_delay.h"

namespace dairlib {

using drake::multibody::JointActuatorIndex;
using drake::multibody::JointIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::VectorXd;
using std::string;
using systems::OutputVector;
using systems::RobotInputReceiver;

TrifingerPlateStateMerger::TrifingerPlateStateMerger(
   const drake::multibody::MultibodyPlant<double>& plant, 
  ModelInstanceIndex trifinger_index, ModelInstanceIndex plate_index,
  const bool publish_efforts)
    : trifinger_index_(trifinger_index),
      plate_index_(plate_index),
      publish_efforts_(publish_efforts) {

  n_q_trifinger_ = plant.num_positions(trifinger_index_);
  n_v_trifinger_ = plant.num_velocities(trifinger_index_);
  n_q_plate_ = plant.num_positions(plate_index_);
  n_v_plate_ = plant.num_velocities(plate_index_);
  n_u_ = plant.num_actuators();

  trifinger_position_index_map_ =
      multibody::MakeNameToPositionsMap(plant, trifinger_index);
  trifinger_velocity_index_map_ =
      multibody::MakeNameToVelocitiesMap(plant, trifinger_index);
  plate_position_index_map_ =
      multibody::MakeNameToPositionsMap(plant, plate_index);
  plate_velocity_index_map_ =
      multibody::MakeNameToVelocitiesMap(plant, plate_index);
  effort_index_map_ = multibody::MakeNameToActuatorsMap(plant);

  trifinger_positions_start_idx_ =
      plant.get_joint(plant.GetJointIndices(trifinger_index).front())
          .position_start();
  trifinger_velocities_start_idx_ =
      plant.get_joint(plant.GetJointIndices(trifinger_index).front())
          .velocity_start();
  plate_positions_start_idx_ =
      plant.get_joint(plant.GetJointIndices(plate_index).front())
          .position_start();
  plate_velocities_start_idx_ =
      plant.get_joint(plant.GetJointIndices(plate_index).front())
          .velocity_start();

  ordered_trifinger_position_names_ = multibody::ExtractOrderedNamesFromMap(
      trifinger_position_index_map_, trifinger_positions_start_idx_);
  ordered_trifinger_velocity_names_ = multibody::ExtractOrderedNamesFromMap(
      trifinger_velocity_index_map_, trifinger_velocities_start_idx_);
  ordered_plate_position_names_ = multibody::ExtractOrderedNamesFromMap(
      plate_position_index_map_, plate_positions_start_idx_);
  ordered_plate_velocity_names_ = multibody::ExtractOrderedNamesFromMap(
      plate_velocity_index_map_, plate_velocities_start_idx_);
  ordered_effort_names_ =
      multibody::ExtractOrderedNamesFromMap(effort_index_map_);


  trifinger_port_ =
      this->DeclareVectorInputPort(
              "trifinger_x", BasicVector<double>(n_q_trifinger_ + n_v_trifinger_))
          .get_index();
  
  plate_port_ =
      this->DeclareVectorInputPort(
              "plate_x", BasicVector<double>(n_q_plate_ + n_v_plate_))
          .get_index();

  if (publish_efforts_) {
    effort_port_ =
        this->DeclareVectorInputPort("u", BasicVector<double>(n_u_))
            .get_index();
  }

  output_port_ = 
      this->DeclareAbstractOutputPort("trifinger_plate_merged_state",
                                  &TrifingerPlateStateMerger::Merge)
          .get_index();

        
}

void TrifingerPlateStateMerger::Merge(const Context<double>& context,
                               dairlib::lcmt_robot_output* state_msg) const {

  const auto state_trifinger = this->EvalVectorInput(context, trifinger_port_);
  const auto state_plate = this->EvalVectorInput(context, plate_port_);



  // using the time from the context
  state_msg->utime = context.get_time() * 1e6;

  state_msg->num_positions = n_q_trifinger_ + n_q_plate_;
  state_msg->num_velocities = n_v_trifinger_ + n_v_plate_;
  state_msg->position_names.resize(n_q_trifinger_ + n_q_plate_);
  state_msg->velocity_names.resize(n_v_trifinger_ + n_v_plate_);
  state_msg->position.resize(n_q_trifinger_ + n_q_plate_);
  state_msg->velocity.resize(n_v_trifinger_ + n_v_plate_);

  // Set positions
  int i = 0;
  while (i < n_q_trifinger_) {
    state_msg->position_names[i] = ordered_trifinger_position_names_[i];
    if (std::isnan(state_trifinger->GetAtIndex(i))) {
      state_msg->position[i] = 0;
    } else {
      state_msg->position[i] = state_trifinger->GetAtIndex(i);
    }
    i++;
  }
  int j = 0;
  while (j < n_q_plate_) {
    state_msg->position_names[i] = ordered_plate_position_names_[j];
    if (std::isnan(state_plate->GetAtIndex(j))) {
      state_msg->position[i] = 0;
    } else {
      state_msg->position[i] = state_plate->GetAtIndex(j);
    }
    i++;
    j++;
  }

  // Set velocities
  i = 0;
  while (i < n_v_trifinger_) {
    state_msg->velocity[i] = state_trifinger->GetAtIndex(n_q_trifinger_ + i);
    state_msg->velocity_names[i] = ordered_trifinger_velocity_names_[i];
    i++;
  }
  j = 0;
  while (j < n_v_plate_) {
    state_msg->velocity[i] = state_plate->GetAtIndex(n_q_plate_ + j);
    state_msg->velocity_names[i] = ordered_plate_velocity_names_[j];
    i++;
    j++;
  }

  if (publish_efforts_) {
    const auto efforts = this->EvalVectorInput(context, effort_port_);

    state_msg->num_efforts = n_u_;
    state_msg->effort_names.resize(n_u_);
    state_msg->effort.resize(n_u_);

    for (int i = 0; i < n_u_; i++) {
      state_msg->effort[i] = efforts->GetAtIndex(i);
      state_msg->effort_names[i] = ordered_effort_names_[i];
    }
  }

}

// Hardcoded for trifinger-plate example
SubvectorPassThrough<double>* AddActuationRecieverAndStateSenderTrifingerPlate(
    drake::systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& plant,
    drake::systems::lcm::LcmInterfaceSystem* lcm, std::string actuator_channel,
    std::string state_channel, double publish_rate,
    drake::multibody::ModelInstanceIndex trifinger_index,
    drake::multibody::ModelInstanceIndex plate_index,  
    bool publish_efforts, double actuator_delay) {
  // Create LCM input for actuators
  auto input_sub =
      builder->AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          actuator_channel, lcm));
  auto input_receiver = builder->AddSystem<RobotInputReceiver>(plant);
  auto passthrough = builder->AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(), 0,
      plant.get_actuation_input_port().size());
  builder->Connect(*input_sub, *input_receiver);
  builder->Connect(*input_receiver, *passthrough);

  // Create LCM output for state and efforts
  auto state_pub =
      builder->AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          state_channel, lcm, 1.0 / publish_rate));
  auto state_sender = builder->AddSystem<TrifingerPlateStateMerger>(
      plant, trifinger_index, plate_index, publish_efforts);

  
  builder->Connect(plant.get_state_output_port(trifinger_index),
                    state_sender->get_input_port_trifinger());
  builder->Connect(plant.get_state_output_port(plate_index),
                    state_sender->get_input_port_plate());
  

  // Add delay, if used, and associated connections
  if (actuator_delay > 0) {
    auto discrete_time_delay =
        builder->AddSystem<drake::systems::DiscreteTimeDelay>(
            1.0 / publish_rate, actuator_delay * publish_rate,
            plant.num_actuators());
    builder->Connect(*passthrough, *discrete_time_delay);
    builder->Connect(discrete_time_delay->get_output_port(),
                     plant.get_actuation_input_port());

    if (publish_efforts) {
      builder->Connect(discrete_time_delay->get_output_port(),
                       state_sender->get_input_port_effort());
    }
  } else {
    builder->Connect(passthrough->get_output_port(),
                     plant.get_actuation_input_port());
    if (publish_efforts) {
      builder->Connect(passthrough->get_output_port(),
                       state_sender->get_input_port_effort());
    }
  }

  builder->Connect(*state_sender, *state_pub);

  return passthrough;
}

} // namespace dairlib