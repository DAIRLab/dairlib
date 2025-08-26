#include "object_state_systems.h"

#include "multibody/multibody_utils.h"

#include "drake/multibody/plant/multibody_plant.h"

using Eigen::VectorXd;

using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;

namespace dairlib {
namespace systems {
namespace lcmt_systems {

ObjectStateConsumer::ObjectStateConsumer(
    const drake::multibody::MultibodyPlant<double>& plant) {
  num_positions_ = plant.num_positions();
  num_velocities_ = plant.num_velocities();
  position_index_map_ = multibody::MakeNameToPositionsMap(plant);
  velocity_index_map_ = multibody::MakeNameToVelocitiesMap(plant);
  model_instance_ = drake::multibody::ModelInstanceIndex(-1);

  positions_start_idx_ = 0;
  velocities_start_idx_ = 0;
  this->DeclareAbstractInputPort("lcmt_object_state",
                                 drake::Value<dairlib::lcmt_object_state>{});
  this->DeclareVectorOutputPort(
      "x, t",
      StateVector<double>(plant.num_positions(), plant.num_velocities()),
      &ObjectStateConsumer::CopyOutput);
}

// Consumes lcmt_object_state and outputs StateVector for a model instance.
ObjectStateConsumer::ObjectStateConsumer(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::multibody::ModelInstanceIndex model_instance) {
  model_instance_ = model_instance;
  num_positions_ = plant.num_positions(model_instance);
  num_velocities_ = plant.num_velocities(model_instance);
  position_index_map_ =
      multibody::MakeNameToPositionsMap(plant, model_instance);
  velocity_index_map_ =
      multibody::MakeNameToVelocitiesMap(plant, model_instance);

  // Get starting indices for positions and velocities in the plant state vector
  positions_start_idx_ =
      plant.get_joint(plant.GetJointIndices(model_instance).front())
          .position_start();
  velocities_start_idx_ =
      plant.get_joint(plant.GetJointIndices(model_instance).front())
          .velocity_start();

  // Declare input and output ports
  this->DeclareAbstractInputPort("lcmt_object_state",
                                 drake::Value<dairlib::lcmt_object_state>{});
  this->DeclareVectorOutputPort(
      "x, t",
      StateVector<double>(plant.num_positions(model_instance),
                          plant.num_velocities(model_instance)),
      &ObjectStateConsumer::CopyOutput);
}

// Converts lcmt_object_state input to StateVector output.
void ObjectStateConsumer::CopyOutput(const Context<double>& context,
                                     StateVector<double>* output) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& state_msg = input->get_value<dairlib::lcmt_object_state>();

  VectorXd positions = VectorXd::Zero(num_positions_);
  // Map received positions to output vector
  for (int i = 0; i < state_msg.num_positions; i++) {
    int j = position_index_map_.at(state_msg.position_names[i]);
    positions(j - positions_start_idx_) = state_msg.position[i];
  }
  VectorXd velocities = VectorXd::Zero(num_velocities_);
  // Map received velocities to output vector
  for (int i = 0; i < state_msg.num_velocities; i++) {
    int j = velocity_index_map_.at(state_msg.velocity_names[i]);
    velocities(j - velocities_start_idx_) = state_msg.velocity[i];
  }

  output->SetPositions(positions);
  output->SetVelocities(velocities);
  output->set_timestamp(state_msg.utime * 1.0e-6);
}

// Initializes lcmt_object_state in the context with zero state and names.
void ObjectStateConsumer::InitializeSubscriberPositions(
    const MultibodyPlant<double>& plant,
    drake::systems::Context<double>& context) const {
  auto& state_msg = context.get_mutable_abstract_state<lcmt_object_state>(0);

  // Set timestamp from context time
  state_msg.utime = context.get_time() * 1e6;

  // Get ordered names for positions and velocities
  std::vector<std::string> ordered_position_names =
      multibody::ExtractOrderedNamesFromMap(position_index_map_,
                                            positions_start_idx_);
  std::vector<std::string> ordered_velocity_names =
      multibody::ExtractOrderedNamesFromMap(velocity_index_map_,
                                            velocities_start_idx_);

  state_msg.num_positions = num_positions_;
  state_msg.num_velocities = num_velocities_;
  state_msg.position_names.resize(num_positions_);
  state_msg.velocity_names.resize(num_velocities_);
  state_msg.position.resize(num_positions_);
  state_msg.velocity.resize(num_positions_);

  // Initialize positions to zero
  for (int i = 0; i < num_positions_; i++) {
    state_msg.position_names[i] = ordered_position_names[i];
    state_msg.position[i] = 0;
  }

  // Set quaternion w = 1 for floating base, if present
  if (model_instance_ != drake::multibody::ModelInstanceIndex(-1)) {
    if (plant.HasUniqueFreeBaseBody(model_instance_)) {
      state_msg.position.at(0) = 1;
    }
  } else {
    for (const auto& body_idx : plant.GetFloatingBaseBodies()) {
      const auto& body = plant.get_body(body_idx);
      if (body.has_quaternion_dofs()) {
        state_msg.position.at(body.floating_positions_start()) = 1;
      }
    }
  }

  // Initialize velocities to zero
  for (int i = 0; i < num_velocities_; i++) {
    state_msg.velocity[i] = 0;
    state_msg.velocity_names[i] = ordered_velocity_names[i];
  }
}

/*--------------------------------------------------------------------------*/
// Implementation for ObjectStateGenerator, which generates lcmt_object_state
// from state vector.

// Constructor for all model instances.
ObjectStateGenerator::ObjectStateGenerator(
    const drake::multibody::MultibodyPlant<double>& plant) {
  num_positions_ = plant.num_positions();
  num_velocities_ = plant.num_velocities();

  position_index_map_ = multibody::MakeNameToPositionsMap(plant);
  velocity_index_map_ = multibody::MakeNameToVelocitiesMap(plant);

  model_instance_ = drake::multibody::ModelInstanceIndex(-1);
  positions_start_idx_ = 0;
  velocities_start_idx_ = 0;

  ordered_position_names_ =
      multibody::ExtractOrderedNamesFromMap(position_index_map_);
  ordered_velocity_names_ =
      multibody::ExtractOrderedNamesFromMap(velocity_index_map_);

  // Declare input port for state vector
  state_input_port_ =
      this->DeclareVectorInputPort(
              "x", BasicVector<double>(num_positions_ + num_velocities_))
          .get_index();

  // Declare output port for lcmt_object_state
  this->DeclareAbstractOutputPort("lcmt_object_state",
                                  &ObjectStateGenerator::Output);
}

// Constructor for a specific model instance, with option to publish velocities.
ObjectStateGenerator::ObjectStateGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    bool publish_velocities,
    drake::multibody::ModelInstanceIndex model_instance)
    : publish_velocities_(publish_velocities), model_instance_(model_instance) {
  num_positions_ = plant.num_positions(model_instance);
  num_velocities_ = plant.num_velocities(model_instance);

  position_index_map_ =
      multibody::MakeNameToPositionsMap(plant, model_instance);
  velocity_index_map_ =
      multibody::MakeNameToVelocitiesMap(plant, model_instance);

  // Get starting indices for positions and velocities
  positions_start_idx_ =
      plant.get_joint(plant.GetJointIndices(model_instance).front())
          .position_start();
  velocities_start_idx_ =
      plant.get_joint(plant.GetJointIndices(model_instance).front())
          .velocity_start();

  ordered_position_names_ = multibody::ExtractOrderedNamesFromMap(
      position_index_map_, positions_start_idx_);
  ordered_velocity_names_ = multibody::ExtractOrderedNamesFromMap(
      velocity_index_map_, velocities_start_idx_);

  // Declare input port for state vector
  state_input_port_ =
      this->DeclareVectorInputPort(
              "x", BasicVector<double>(num_positions_ + num_velocities_))
          .get_index();

  // Declare output port for lcmt_object_state
  this->DeclareAbstractOutputPort("lcmt_object_state",
                                  &ObjectStateGenerator::Output);
}

/// Populate a state message with all states from the input vector.
void ObjectStateGenerator::Output(const Context<double>& context,
                                  dairlib::lcmt_object_state* state_msg) const {
  const auto state = this->EvalVectorInput(context, state_input_port_);

  // Set timestamp from context time
  state_msg->utime = context.get_time() * 1e6;

  state_msg->num_positions = num_positions_;
  state_msg->num_velocities = num_velocities_;
  state_msg->position_names.resize(num_positions_);
  state_msg->velocity_names.resize(num_velocities_);
  state_msg->position.resize(num_positions_);
  state_msg->velocity.resize(num_velocities_);

  // Fill positions, replacing NaN with zero
  for (int i = 0; i < num_positions_; i++) {
    state_msg->position_names[i] = ordered_position_names_[i];
    if (std::isnan(state->GetAtIndex(i))) {
      state_msg->position[i] = 0;
    } else {
      state_msg->position[i] = state->GetAtIndex(i);
    }
  }
  // Fill velocities if enabled
  for (int i = 0; i < num_velocities_; i++) {
    state_msg->velocity[i] = 0;
    if (publish_velocities_) {
      state_msg->velocity[i] = state->GetAtIndex(num_positions_ + i);
    }
    state_msg->velocity_names[i] = ordered_velocity_names_[i];
  }
}

}  // namespace lcmt_systems
}  // namespace systems
}  // namespace dairlib