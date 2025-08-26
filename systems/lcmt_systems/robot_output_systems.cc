#include "robot_output_systems.h"

#include "multibody/multibody_utils.h"

using Eigen::VectorXd;

using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;

namespace dairlib {
namespace systems {
namespace lcmt_systems {

// Constructor for full plant (all model instances)
RobotOutputConsumer::RobotOutputConsumer(
    const drake::multibody::MultibodyPlant<double>& plant) {
  num_positions_ = plant.num_positions();
  num_velocities_ = plant.num_velocities();
  num_efforts_ = plant.num_actuators();
  position_index_map_ = multibody::MakeNameToPositionsMap(plant);
  velocity_index_map_ = multibody::MakeNameToVelocitiesMap(plant);
  model_instance_ = drake::multibody::ModelInstanceIndex(-1);

  positions_start_idx_ = 0;
  velocities_start_idx_ = 0;
  effort_index_map_ = multibody::MakeNameToActuatorsMap(plant);

  // Declare input and output ports
  this->DeclareAbstractInputPort("lcmt_robot_output",
                                 drake::Value<dairlib::lcmt_robot_output>{});
  this->DeclareVectorOutputPort(
      "x, u, t",
      OutputVector<double>(plant.num_positions(), plant.num_velocities(),
                           plant.num_actuators()),
      &RobotOutputConsumer::CopyOutput);
}

// Constructor for a specific model instance
RobotOutputConsumer::RobotOutputConsumer(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::multibody::ModelInstanceIndex model_instance) {
  model_instance_ = model_instance;
  num_positions_ = plant.num_positions(model_instance);
  num_velocities_ = plant.num_velocities(model_instance);
  num_efforts_ = plant.num_actuators();
  position_index_map_ =
      multibody::MakeNameToPositionsMap(plant, model_instance);
  velocity_index_map_ =
      multibody::MakeNameToVelocitiesMap(plant, model_instance);

  // Get start indices for positions and velocities
  positions_start_idx_ =
      plant.get_joint(plant.GetJointIndices(model_instance).front())
          .position_start();
  velocities_start_idx_ =
      plant.get_joint(plant.GetJointIndices(model_instance).front())
          .velocity_start();
  effort_index_map_ = multibody::MakeNameToActuatorsMap(plant);

  // Declare input and output ports
  this->DeclareAbstractInputPort("lcmt_robot_output",
                                 drake::Value<dairlib::lcmt_robot_output>{});
  this->DeclareVectorOutputPort(
      "x, u, t",
      OutputVector<double>(plant.num_positions(model_instance),
                           plant.num_velocities(model_instance),
                           plant.num_actuators()),
      &RobotOutputConsumer::CopyOutput);
}

// Copies data from lcmt_robot_output message to OutputVector
void RobotOutputConsumer::CopyOutput(const Context<double>& context,
                                     OutputVector<double>* output) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& state_msg = input->get_value<dairlib::lcmt_robot_output>();

  // Fill positions
  VectorXd positions = VectorXd::Zero(num_positions_);
  for (int i = 0; i < state_msg.num_positions; i++) {
    int j = position_index_map_.at(state_msg.position_names[i]);
    positions(j - positions_start_idx_) = state_msg.position[i];
  }
  // Fill velocities
  VectorXd velocities = VectorXd::Zero(num_velocities_);
  for (int i = 0; i < state_msg.num_velocities; i++) {
    int j = velocity_index_map_.at(state_msg.velocity_names[i]);
    velocities(j - velocities_start_idx_) = state_msg.velocity[i];
  }
  // Fill efforts
  VectorXd efforts = VectorXd::Zero(num_efforts_);
  for (int i = 0; i < state_msg.num_efforts; i++) {
    int j = effort_index_map_.at(state_msg.effort_names[i]);
    efforts(j) = state_msg.effort[j];
  }

  // Fill IMU acceleration if needed
  VectorXd imu = VectorXd::Zero(3);
  if (num_positions_ != num_velocities_) {
    for (int i = 0; i < 3; ++i) {
      imu[i] = state_msg.imu_accel[i];
    }
  }

  // Set output values
  output->SetPositions(positions);
  output->SetVelocities(velocities);
  output->SetEfforts(efforts);
  output->SetIMUAccelerations(imu);
  output->set_timestamp(state_msg.utime * 1.0e-6);
}

// Initializes lcmt_robot_output message in context with zeros and names
void RobotOutputConsumer::InitializeSubscriberPositions(
    const MultibodyPlant<double>& plant,
    drake::systems::Context<double>& context) const {
  auto& state_msg = context.get_mutable_abstract_state<lcmt_robot_output>(0);

  // Set time
  state_msg.utime = context.get_time() * 1e6;

  // Get ordered names for positions, velocities, efforts
  std::vector<std::string> ordered_position_names =
      multibody::ExtractOrderedNamesFromMap(position_index_map_,
                                            positions_start_idx_);
  std::vector<std::string> ordered_velocity_names =
      multibody::ExtractOrderedNamesFromMap(velocity_index_map_,
                                            velocities_start_idx_);
  std::vector<std::string> ordered_effort_names =
      multibody::ExtractOrderedNamesFromMap(effort_index_map_);

  state_msg.num_positions = num_positions_;
  state_msg.num_velocities = num_velocities_;
  state_msg.position_names.resize(num_positions_);
  state_msg.velocity_names.resize(num_velocities_);
  state_msg.position.resize(num_positions_);
  state_msg.velocity.resize(num_positions_);

  // Set position names and initialize positions to zero
  for (int i = 0; i < num_positions_; i++) {
    state_msg.position_names[i] = ordered_position_names[i];
    state_msg.position[i] = 0;
  }

  // Set quaternion w = 1 for floating base if present
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

  // Set velocity names and initialize velocities to zero
  for (int i = 0; i < num_velocities_; i++) {
    state_msg.velocity[i] = 0;
    state_msg.velocity_names[i] = ordered_velocity_names[i];
  }
}

// RobotOutputGenerator constructor for full plant
RobotOutputGenerator::RobotOutputGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    const bool publish_efforts, const bool publish_imu)
    : publish_efforts_(publish_efforts), publish_imu_(publish_imu) {
  num_positions_ = plant.num_positions();
  num_velocities_ = plant.num_velocities();
  num_efforts_ = plant.num_actuators();

  position_index_map_ = multibody::MakeNameToPositionsMap(plant);
  velocity_index_map_ = multibody::MakeNameToVelocitiesMap(plant);
  effort_index_map_ = multibody::MakeNameToActuatorsMap(plant);

  positions_start_idx_ = 0;
  velocities_start_idx_ = 0;

  // Get ordered names for output message
  ordered_position_names_ =
      multibody::ExtractOrderedNamesFromMap(position_index_map_);
  ordered_velocity_names_ =
      multibody::ExtractOrderedNamesFromMap(velocity_index_map_);
  ordered_effort_names_ =
      multibody::ExtractOrderedNamesFromMap(effort_index_map_);

  // Declare input ports for state, effort, and IMU
  state_input_port_ =
      this->DeclareVectorInputPort(
              "x", BasicVector<double>(num_positions_ + num_velocities_))
          .get_index();
  if (publish_efforts_) {
    effort_input_port_ =
        this->DeclareVectorInputPort("u", BasicVector<double>(num_efforts_))
            .get_index();
  }
  if (publish_imu_) {
    imu_input_port_ =
        this->DeclareVectorInputPort("imu_acceleration", BasicVector<double>(3))
            .get_index();
  }

  // Declare output port for lcmt_robot_output message
  this->DeclareAbstractOutputPort("lcmt_robot_output",
                                  &RobotOutputGenerator::Output);
}

// RobotOutputGenerator constructor for a specific model instance
RobotOutputGenerator::RobotOutputGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::multibody::ModelInstanceIndex model_instance,
    const bool publish_efforts, const bool publish_imu)
    : publish_efforts_(publish_efforts), publish_imu_(publish_imu) {
  num_positions_ = plant.num_positions(model_instance);
  num_velocities_ = plant.num_velocities(model_instance);
  num_efforts_ = plant.num_actuators();

  position_index_map_ =
      multibody::MakeNameToPositionsMap(plant, model_instance);
  velocity_index_map_ =
      multibody::MakeNameToVelocitiesMap(plant, model_instance);
  effort_index_map_ = multibody::MakeNameToActuatorsMap(plant);

  // Get start indices for positions and velocities
  positions_start_idx_ =
      plant.get_joint(plant.GetJointIndices(model_instance).front())
          .position_start();
  velocities_start_idx_ =
      plant.get_joint(plant.GetJointIndices(model_instance).front())
          .velocity_start();

  // Get ordered names for output message
  ordered_position_names_ = multibody::ExtractOrderedNamesFromMap(
      position_index_map_, positions_start_idx_);
  ordered_velocity_names_ = multibody::ExtractOrderedNamesFromMap(
      velocity_index_map_, velocities_start_idx_);
  ordered_effort_names_ =
      multibody::ExtractOrderedNamesFromMap(effort_index_map_);

  // Declare input ports for state, effort, and IMU
  state_input_port_ =
      this->DeclareVectorInputPort(
              "x", BasicVector<double>(num_positions_ + num_velocities_))
          .get_index();
  if (publish_efforts_) {
    effort_input_port_ =
        this->DeclareVectorInputPort("u", BasicVector<double>(num_efforts_))
            .get_index();
  }
  if (publish_imu_) {
    imu_input_port_ =
        this->DeclareVectorInputPort("imu_acceleration", BasicVector<double>(3))
            .get_index();
  }

  // Declare output port for lcmt_robot_output message
  this->DeclareAbstractOutputPort("lcmt_robot_output",
                                  &RobotOutputGenerator::Output);
}

/// Populate a state message with all states
void RobotOutputGenerator::Output(const Context<double>& context,
                                  dairlib::lcmt_robot_output* state_msg) const {
  const auto state = this->EvalVectorInput(context, state_input_port_);

  // Set time
  state_msg->utime = context.get_time() * 1e6;

  // Set position and velocity names and values
  state_msg->num_positions = num_positions_;
  state_msg->num_velocities = num_velocities_;
  state_msg->position_names.resize(num_positions_);
  state_msg->velocity_names.resize(num_velocities_);
  state_msg->position.resize(num_positions_);
  state_msg->velocity.resize(num_velocities_);

  for (int i = 0; i < num_positions_; i++) {
    state_msg->position_names[i] = ordered_position_names_[i];
    // Replace NaN with zero
    if (std::isnan(state->GetAtIndex(i))) {
      state_msg->position[i] = 0;
    } else {
      state_msg->position[i] = state->GetAtIndex(i);
    }
  }
  for (int i = 0; i < num_velocities_; i++) {
    state_msg->velocity[i] = state->GetAtIndex(num_positions_ + i);
    state_msg->velocity_names[i] = ordered_velocity_names_[i];
  }

  // Set effort values if enabled
  if (publish_efforts_) {
    const auto efforts = this->EvalVectorInput(context, effort_input_port_);

    state_msg->num_efforts = num_efforts_;
    state_msg->effort_names.resize(num_efforts_);
    state_msg->effort.resize(num_efforts_);

    for (int i = 0; i < num_efforts_; i++) {
      state_msg->effort[i] = efforts->GetAtIndex(i);
      state_msg->effort_names[i] = ordered_effort_names_[i];
    }
  }

  // Set IMU acceleration if enabled
  if (publish_imu_) {
    const auto imu = this->EvalVectorInput(context, imu_input_port_);
    for (int i = 0; i < 3; ++i) {
      state_msg->imu_accel[i] = imu->get_value()[i];
    }
  }
}
}  // namespace lcmt_systems
}  // namespace systems
}  // namespace dairlib