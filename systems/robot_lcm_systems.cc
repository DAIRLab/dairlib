#include "robot_lcm_systems.h"

#include <iostream>

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "multibody/multibody_utils.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/trajectory.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/discrete_time_delay.h"

namespace dairlib {
namespace systems {

using drake::multibody::JointActuatorIndex;
using drake::multibody::JointIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;
using Eigen::VectorXd;
using std::string;
using systems::OutputVector;

/*--------------------------------------------------------------------------*/
// methods implementation for RobotOutputReceiver.

RobotOutputReceiver::RobotOutputReceiver(
    const drake::multibody::MultibodyPlant<double>& plant) {
  num_positions_ = plant.num_positions();
  num_velocities_ = plant.num_velocities();
  num_efforts_ = plant.num_actuators();
  position_index_map_ = multibody::MakeNameToPositionsMap(plant);
  velocity_index_map_ = multibody::MakeNameToVelocitiesMap(plant);
  model_instance_ =
      drake::multibody::ModelInstanceIndex(DEFAULT_MODEL_INSTANCE_INDEX);

  positions_start_idx_ = 0;
  velocities_start_idx_ = 0;
  effort_index_map_ = multibody::MakeNameToActuatorsMap(plant);
  this->DeclareAbstractInputPort("lcmt_robot_output",
                                 drake::Value<dairlib::lcmt_robot_output>{});
  this->DeclareVectorOutputPort(
      "x, u, t",
      OutputVector<double>(plant.num_positions(), plant.num_velocities(),
                           plant.num_actuators()),
      &RobotOutputReceiver::CopyOutput);
}

RobotOutputReceiver::RobotOutputReceiver(
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
  positions_start_idx_ =
      plant.get_joint(plant.GetJointIndices(model_instance).front())
          .position_start();
  velocities_start_idx_ =
      plant.get_joint(plant.GetJointIndices(model_instance).front())
          .velocity_start();
  effort_index_map_ = multibody::MakeNameToActuatorsMap(plant);
  this->DeclareAbstractInputPort("lcmt_robot_output",
                                 drake::Value<dairlib::lcmt_robot_output>{});
  this->DeclareVectorOutputPort(
      "x, u, t",
      OutputVector<double>(plant.num_positions(model_instance),
                           plant.num_velocities(model_instance),
                           plant.num_actuators()),
      &RobotOutputReceiver::CopyOutput);
}

void RobotOutputReceiver::CopyOutput(const Context<double>& context,
                                     OutputVector<double>* output) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& state_msg = input->get_value<dairlib::lcmt_robot_output>();

  VectorXd positions = VectorXd::Zero(num_positions_);
  for (int i = 0; i < state_msg.num_positions; i++) {
    auto it = position_index_map_.find(state_msg.position_names[i]);
    if (it == position_index_map_.end()) {
      throw std::runtime_error("missing position key: " +
                               state_msg.position_names[i]);
    }
    int j = it->second;
    positions(j - positions_start_idx_) = state_msg.position[i];
  }
  VectorXd velocities = VectorXd::Zero(num_velocities_);
  for (int i = 0; i < state_msg.num_velocities; i++) {
    auto it = velocity_index_map_.find(state_msg.velocity_names[i]);
    if (it == velocity_index_map_.end()) {
      throw std::runtime_error("missing velocity key: " +
                               state_msg.velocity_names[i]);
    }
    int j = it->second;
    velocities(j - velocities_start_idx_) = state_msg.velocity[i];
  }
  VectorXd efforts = VectorXd::Zero(num_efforts_);
  for (int i = 0; i < state_msg.num_efforts; i++) {
    auto it = effort_index_map_.find(state_msg.effort_names[i]);
    if (it == effort_index_map_.end()) {
      throw std::runtime_error("missing effort key: " +
                               state_msg.effort_names[i]);
    }
    int j = it->second;
    efforts(j) = state_msg.effort[j];
  }

  VectorXd imu = VectorXd::Zero(3);
  if (num_positions_ != num_velocities_) {
    for (int i = 0; i < 3; ++i) {
      imu[i] = state_msg.imu_accel[i];
    }
  }

  output->SetPositions(positions);
  output->SetVelocities(velocities);
  output->SetEfforts(efforts);
  output->SetIMUAccelerations(imu);
  output->set_timestamp(state_msg.utime * 1.0e-6);
}

void RobotOutputReceiver::InitializeSubscriberPositions(
    const MultibodyPlant<double>& plant,
    drake::systems::Context<double>& context) const {
  auto& state_msg = context.get_mutable_abstract_state<lcmt_robot_output>(0);

  // using the time from the context
  state_msg.utime = context.get_time() * 1e6;

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
  state_msg.velocity.resize(num_velocities_);

  for (int i = 0; i < num_positions_; i++) {
    state_msg.position_names[i] = ordered_position_names[i];
    state_msg.position[i] = 0;
  }

  // Set quaternion w = 1, assumes drake quaternion ordering of wxyz
  if (model_instance_ !=
      drake::multibody::ModelInstanceIndex(DEFAULT_MODEL_INSTANCE_INDEX)) {
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

  for (int i = 0; i < num_velocities_; i++) {
    state_msg.velocity[i] = 0;
    state_msg.velocity_names[i] = ordered_velocity_names[i];
  }
}

/*--------------------------------------------------------------------------*/
// methods implementation for RobotOutputSender.
RobotOutputSender::RobotOutputSender(
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

  ordered_position_names_ =
      multibody::ExtractOrderedNamesFromMap(position_index_map_);
  ordered_velocity_names_ =
      multibody::ExtractOrderedNamesFromMap(velocity_index_map_);
  ordered_effort_names_ =
      multibody::ExtractOrderedNamesFromMap(effort_index_map_);

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

  this->DeclareAbstractOutputPort("lcmt_robot_output",
                                  &RobotOutputSender::Output);
}

RobotOutputSender::RobotOutputSender(
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
  ordered_effort_names_ =
      multibody::ExtractOrderedNamesFromMap(effort_index_map_);

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

  this->DeclareAbstractOutputPort("lcmt_robot_output",
                                  &RobotOutputSender::Output);
}

/// Populate a state message with all states
void RobotOutputSender::Output(const Context<double>& context,
                               dairlib::lcmt_robot_output* state_msg) const {
  const auto state = this->EvalVectorInput(context, state_input_port_);

  // using the time from the context
  state_msg->utime = context.get_time() * 1e6;

  state_msg->num_positions = num_positions_;
  state_msg->num_velocities = num_velocities_;
  state_msg->position_names.resize(num_positions_);
  state_msg->velocity_names.resize(num_velocities_);
  state_msg->position.resize(num_positions_);
  state_msg->velocity.resize(num_velocities_);

  for (int i = 0; i < num_positions_; i++) {
    state_msg->position_names[i] = ordered_position_names_[i];
    double q = state->GetAtIndex(i);
    state_msg->position[i] = std::isnan(q) ? 0 : q;
  }

  for (int i = 0; i < num_velocities_; i++) {
    state_msg->velocity[i] = state->GetAtIndex(num_positions_ + i);
    state_msg->velocity_names[i] = ordered_velocity_names_[i];
  }

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

  if (publish_imu_) {
    const auto imu = this->EvalVectorInput(context, imu_input_port_);
    for (int i = 0; i < 3; ++i) {
      state_msg->imu_accel[i] = imu->get_value()[i];
    }
  }
}

ObjectStateReceiver::ObjectStateReceiver(
    const drake::multibody::MultibodyPlant<double>& plant) {
  num_positions_ = plant.num_positions();
  num_velocities_ = plant.num_velocities();
  position_index_map_ = multibody::MakeNameToPositionsMap(plant);
  velocity_index_map_ = multibody::MakeNameToVelocitiesMap(plant);
  model_instance_ =
      drake::multibody::ModelInstanceIndex(DEFAULT_MODEL_INSTANCE_INDEX);

  positions_start_idx_ = 0;
  velocities_start_idx_ = 0;
  this->DeclareAbstractInputPort("lcmt_object_state",
                                 drake::Value<dairlib::lcmt_object_state>{});
  this->DeclareVectorOutputPort(
      "x, t",
      StateVector<double>(plant.num_positions(), plant.num_velocities()),
      &ObjectStateReceiver::CopyOutput);
}

ObjectStateReceiver::ObjectStateReceiver(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::multibody::ModelInstanceIndex model_instance) {
  model_instance_ = model_instance;
  num_positions_ = plant.num_positions(model_instance);
  num_velocities_ = plant.num_velocities(model_instance);
  position_index_map_ =
      multibody::MakeNameToPositionsMap(plant, model_instance);
  velocity_index_map_ =
      multibody::MakeNameToVelocitiesMap(plant, model_instance);

  for (const auto& entry : plant.GetJointIndices(model_instance)) {
    // If joint.num_positions() == 0, then it is a fixed joint.
    // Skip it and fix positions_start_idx_ to be the non fixed joint.
    if (plant.get_joint(entry).num_positions() != 0) {
      positions_start_idx_ = plant.get_joint(entry).position_start();
      velocities_start_idx_ = plant.get_joint(entry).velocity_start();
      break;
    }
  }
  this->DeclareAbstractInputPort("lcmt_object_state",
                                 drake::Value<dairlib::lcmt_object_state>{});
  this->DeclareVectorOutputPort(
      "x, t",
      StateVector<double>(plant.num_positions(model_instance),
                          plant.num_velocities(model_instance)),
      &ObjectStateReceiver::CopyOutput);
}

void ObjectStateReceiver::CopyOutput(const Context<double>& context,
                                     StateVector<double>* output) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& state_msg = input->get_value<dairlib::lcmt_object_state>();

  VectorXd positions = VectorXd::Zero(num_positions_);
  for (int i = 0; i < state_msg.num_positions; i++) {
    int j = position_index_map_.at(state_msg.position_names[i]);
    positions(j - positions_start_idx_) = state_msg.position[i];
  }
  VectorXd velocities = VectorXd::Zero(num_velocities_);
  for (int i = 0; i < state_msg.num_velocities; i++) {
    int j = velocity_index_map_.at(state_msg.velocity_names[i]);
    velocities(j - velocities_start_idx_) = state_msg.velocity[i];
  }

  output->SetPositions(positions);
  output->SetVelocities(velocities);
  output->set_timestamp(state_msg.utime * 1.0e-6);
}

void ObjectStateReceiver::InitializeSubscriberPositions(
    const MultibodyPlant<double>& plant,
    drake::systems::Context<double>& context) const {
  auto& state_msg = context.get_mutable_abstract_state<lcmt_object_state>(0);

  // using the time from the context
  state_msg.utime = context.get_time() * 1e6;

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

  for (int i = 0; i < num_positions_; i++) {
    state_msg.position_names[i] = ordered_position_names[i];
    state_msg.position[i] = 0;
  }

  // Set quaternion w = 1, assumes drake quaternion ordering of wxyz
  if (model_instance_ !=
      drake::multibody::ModelInstanceIndex(DEFAULT_MODEL_INSTANCE_INDEX)) {
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

  for (int i = 0; i < num_velocities_; i++) {
    state_msg.velocity[i] = 0;
    state_msg.velocity_names[i] = ordered_velocity_names[i];
  }
}

/*--------------------------------------------------------------------------*/
// methods implementation for RobotOutputSender.
ObjectStateSender::ObjectStateSender(
    const drake::multibody::MultibodyPlant<double>& plant) {
  num_positions_ = plant.num_positions();
  num_velocities_ = plant.num_velocities();

  position_index_map_ = multibody::MakeNameToPositionsMap(plant);
  velocity_index_map_ = multibody::MakeNameToVelocitiesMap(plant);

  model_instance_ =
      drake::multibody::ModelInstanceIndex(DEFAULT_MODEL_INSTANCE_INDEX);
  positions_start_idx_ = 0;
  velocities_start_idx_ = 0;

  ordered_position_names_ =
      multibody::ExtractOrderedNamesFromMap(position_index_map_);
  ordered_velocity_names_ =
      multibody::ExtractOrderedNamesFromMap(velocity_index_map_);

  state_input_port_ =
      this->DeclareVectorInputPort(
              "x", BasicVector<double>(num_positions_ + num_velocities_))
          .get_index();

  this->DeclareAbstractOutputPort("lcmt_object_state",
                                  &ObjectStateSender::Output);
}

ObjectStateSender::ObjectStateSender(
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

  for (const auto& entry : plant.GetJointIndices(model_instance)) {
    // If joint.num_positions() == 0, then it is a fixed joint.
    // Skip it and fix positions_start_idx_ to be the non fixed joint.
    if (plant.get_joint(entry).num_positions() != 0) {
      positions_start_idx_ = plant.get_joint(entry).position_start();
      velocities_start_idx_ = plant.get_joint(entry).velocity_start();
      break;
    }
  }

  ordered_position_names_ = multibody::ExtractOrderedNamesFromMap(
      position_index_map_, positions_start_idx_);
  ordered_velocity_names_ = multibody::ExtractOrderedNamesFromMap(
      velocity_index_map_, velocities_start_idx_);

  state_input_port_ =
      this->DeclareVectorInputPort(
              "x", BasicVector<double>(num_positions_ + num_velocities_))
          .get_index();
  this->DeclareAbstractOutputPort("lcmt_object_state",
                                  &ObjectStateSender::Output);
}

/// Populate a state message with all states
void ObjectStateSender::Output(const Context<double>& context,
                               dairlib::lcmt_object_state* state_msg) const {
  const auto state = this->EvalVectorInput(context, state_input_port_);

  // using the time from the context
  state_msg->utime = context.get_time() * 1e6;

  state_msg->num_positions = num_positions_;
  state_msg->num_velocities = num_velocities_;
  state_msg->position_names.resize(num_positions_);
  state_msg->velocity_names.resize(num_velocities_);
  state_msg->position.resize(num_positions_);
  state_msg->velocity.resize(num_velocities_);

  for (int i = 0; i < num_positions_; i++) {
    state_msg->position_names[i] = ordered_position_names_[i];
    if (std::isnan(state->GetAtIndex(i))) {
      state_msg->position[i] = 0;
    } else {
      state_msg->position[i] = state->GetAtIndex(i);
    }
  }
  for (int i = 0; i < num_velocities_; i++) {
    state_msg->velocity[i] = 0;
    if (publish_velocities_) {
      state_msg->velocity[i] = state->GetAtIndex(num_positions_ + i);
    }
    state_msg->velocity_names[i] = ordered_velocity_names_[i];
  }
}

/*--------------------------------------------------------------------------*/
// methods implementation for RobotInputReceiver.

RobotInputReceiver::RobotInputReceiver(
    const drake::multibody::MultibodyPlant<double>& plant) {
  num_actuators_ = plant.num_actuators();
  actuator_index_map_ = multibody::MakeNameToActuatorsMap(plant);
  this->DeclareAbstractInputPort("lcmt_robot_input",
                                 drake::Value<dairlib::lcmt_robot_input>{});
  this->DeclareVectorOutputPort(
      "u, t", TimestampedVector<double>(num_actuators_),
      &RobotInputReceiver::CopyInputOut, {all_sources_ticket()});
}

void RobotInputReceiver::CopyInputOut(const Context<double>& context,
                                      TimestampedVector<double>* output) const {
  const drake::AbstractValue* input = this->EvalAbstractInput(context, 0);
  DRAKE_ASSERT(input != nullptr);
  const auto& input_msg = input->get_value<dairlib::lcmt_robot_input>();

  VectorXd input_vector = VectorXd::Zero(num_actuators_);

  for (int i = 0; i < input_msg.num_efforts; i++) {
    int j = actuator_index_map_.at(input_msg.effort_names[i]);
    input_vector(j) = input_msg.efforts[i];
  }
  output->SetDataVector(input_vector);
  output->set_timestamp(input_msg.utime * 1.0e-6);
}

ThreeDPrinterInputReceiver::ThreeDPrinterInputReceiver(
    const drake::multibody::MultibodyPlant<double>& plant)
    : num_positions_(plant.num_positions()),
      num_velocities_(plant.num_velocities()) {
  this->DeclareAbstractInputPort("lcmt_robot_output",
                                 drake::Value<dairlib::lcmt_robot_output>{});

  this->DeclareVectorOutputPort("x", BasicVector<double>(6),
                                &ThreeDPrinterInputReceiver::CopyInputOut,
                                {all_sources_ticket()});

  // Populate the name → index maps from the plant here.
}

void ThreeDPrinterInputReceiver::CopyInputOut(
    const Context<double>& context, BasicVector<double>* output) const {
  const auto& msg = this->EvalAbstractInput(context, 0)
                        ->get_value<dairlib::lcmt_robot_output>();

  VectorXd x = VectorXd::Zero(6);
  for (int i = 0; i < 3 && i < msg.num_positions; ++i) {
    x[i] = msg.position[i];
  }

  for (int i = 0; i < 3 && i < msg.num_velocities; ++i) {
    x[3 + i] = msg.velocity[i];
  }

  for (int i = 0; i < 6; ++i) {
    output->SetAtIndex(i, x[i]);
  }
}

/*--------------------------------------------------------------------------*/
// methods implementation for RobotCommandSender.

RobotCommandSender::RobotCommandSender(
    const drake::multibody::MultibodyPlant<double>& plant) {
  num_actuators_ = plant.num_actuators();
  actuator_index_map_ = multibody::MakeNameToActuatorsMap(plant);

  for (JointActuatorIndex i(0); i < plant.num_actuators(); ++i) {
    ordered_actuator_names_.push_back(plant.get_joint_actuator(i).name());
  }

  this->DeclareVectorInputPort("u, t",
                               TimestampedVector<double>(num_actuators_));
  this->DeclareAbstractOutputPort("lcmt_robot_input",
                                  &RobotCommandSender::OutputCommand);
}

void RobotCommandSender::OutputCommand(
    const Context<double>& context,
    dairlib::lcmt_robot_input* input_msg) const {
  const TimestampedVector<double>* command =
      (TimestampedVector<double>*)this->EvalVectorInput(context, 0);

  input_msg->utime = command->get_timestamp() * 1e6;
  input_msg->num_efforts = num_actuators_;
  input_msg->effort_names.resize(num_actuators_);
  input_msg->efforts.resize(num_actuators_);
  for (int i = 0; i < num_actuators_; i++) {
    input_msg->effort_names[i] = ordered_actuator_names_[i];
    if (std::isnan(command->GetAtIndex(i))) {
      input_msg->efforts[i] = 0;
    } else {
      input_msg->efforts[i] = command->GetAtIndex(i);
    }
  }
}

ThreeDPrinterCommandSender::ThreeDPrinterCommandSender(
    const drake::multibody::MultibodyPlant<double>& plant,
    const Eigen::Vector3d& target_offset)
    : target_offset_(target_offset) {
  // Declare an abstract input port that matches the trajectory output type.
  PiecewisePolynomial<double> empty_pp(Eigen::VectorXd::Zero(3));
  Trajectory<double>& traj_inst = empty_pp;

  this->DeclareAbstractInputPort("trajectory",
                                 drake::Value<Trajectory<double>>(traj_inst));

  this->DeclareAbstractOutputPort("lcmt_robot_output",
                                  &ThreeDPrinterCommandSender::OutputCommand);
}

void ThreeDPrinterCommandSender::OutputCommand(
    const Context<double>& context,
    dairlib::lcmt_robot_output* output_msg) const {
  // Get the trajectory from the abstract input port
  const auto& base = get_input_port(0).Eval<Trajectory<double>>(context);

  const auto& trajectory =
      dynamic_cast<const PiecewisePolynomial<double>&>(base);

  // Get current time from context
  double current_time = context.get_time();
  VectorXd default_position_ = VectorXd::Zero(3);
  default_position_ << 0.0, 0.0,
      0.25;  // Default position if trajectory is empty

  VectorXd desired_position;

  if (trajectory.start_time() < 1e-3) {
    desired_position = default_position_;
  } else {
    desired_position = trajectory.value(current_time);
  }

  desired_position += target_offset_;

  // Evaluate trajectory derivative at current time to get velocities
  VectorXd desired_velocity = trajectory.EvalDerivative(current_time, 1);

  // Initialize the output message
  output_msg->utime = current_time * 1e6;

  // Set up position names and values for x, y, z
  output_msg->num_positions = desired_position.size();
  output_msg->position_names.resize(desired_position.size());
  output_msg->position.resize(desired_position.size());

  // Use standard coordinate names
  std::vector<std::string> coord_names = {"x", "y", "z"};

  for (int i = 0; i < static_cast<int>(desired_position.size()); i++) {
    if (i < static_cast<int>(coord_names.size())) {
      output_msg->position_names[i] = coord_names[i];
    } else {
      output_msg->position_names[i] = "coord_" + std::to_string(i);
    }
    output_msg->position[i] = desired_position(i);
  }

  // Set up velocity names and values from trajectory derivative
  output_msg->num_velocities = desired_velocity.size();
  output_msg->velocity_names.resize(desired_velocity.size());
  output_msg->velocity.resize(desired_velocity.size());

  std::vector<std::string> velocity_names = {"x_vel", "y_vel", "z_vel"};

  for (int i = 0; i < static_cast<int>(desired_velocity.size()); i++) {
    if (i < static_cast<int>(velocity_names.size())) {
      output_msg->velocity_names[i] = velocity_names[i];
    } else {
      output_msg->velocity_names[i] = "vel_" + std::to_string(i);
    }
    output_msg->velocity[i] = desired_velocity(i);
  }

  output_msg->num_efforts = 0;
  output_msg->effort_names.resize(0);
  output_msg->effort.resize(0);

  // Zero out IMU accelerations
  for (int i = 0; i < 3; ++i) {
    output_msg->imu_accel[i] = 0.0;
  }
}

SubvectorPassThrough<double>* AddActuationRecieverAndStateSenderLcm(
    drake::systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& plant,
    drake::systems::lcm::LcmInterfaceSystem* lcm, std::string actuator_channel,
    std::string state_channel, double publish_rate,
    drake::multibody::ModelInstanceIndex model_instance_index,
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
  auto state_sender = builder->AddSystem<RobotOutputSender>(
      plant, model_instance_index, publish_efforts);

  builder->Connect(plant.get_state_output_port(model_instance_index),
                   state_sender->get_input_port_state());

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

drake::systems::LeafSystem<double>* Add3dPrinterStateReceiverAndStateSenderLcm(
    drake::systems::DiagramBuilder<double>* builder,
    const MultibodyPlant<double>& plant,
    drake::systems::lcm::LcmInterfaceSystem* lcm,
    std::string state_input_channel, std::string state_output_channel,
    double publish_rate,
    drake::multibody::ModelInstanceIndex model_instance_index,
    bool publish_efforts) {
  // Subscribe to the printer state.
  auto input_sub =
      builder->AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
          state_input_channel, lcm));

  auto state_receiver = builder->AddSystem<ThreeDPrinterInputReceiver>(plant);

  builder->Connect(*input_sub, *state_receiver);

  // Choose a model instance for the end-effector whose state size is 6
  // (3 positions + 3 velocities). Prefer the named instance if available,
  // otherwise search for one with the expected size.
  drake::multibody::ModelInstanceIndex ee_model_instance =
      drake::multibody::ModelInstanceIndex(DEFAULT_MODEL_INSTANCE_INDEX);
  bool found = false;
  if (plant.HasModelInstanceNamed("end_effector_simple")) {
    ee_model_instance = plant.GetModelInstanceByName("end_effector_simple");
    int np = plant.num_positions(ee_model_instance);
    int nv = plant.num_velocities(ee_model_instance);
    if (np + nv == 6) {
      found = true;
    }
  }
  if (!found) {
    // Search for any model instance with total state size 6.
    for (drake::multibody::ModelInstanceIndex mi(0);
         mi < plant.num_model_instances(); ++mi) {
      int np = plant.num_positions(mi);
      int nv = plant.num_velocities(mi);
      if (np + nv == 6) {
        ee_model_instance = mi;
        found = true;
        break;
      }
    }
  }

  if (!found) {
    std::cerr << "[robot_lcm_systems] Warning: could not find a model "
              << "instance with total state size 6; using default instance."
              << std::endl;
  }

  builder->Connect(state_receiver->get_output_port(),
                   plant.get_desired_state_input_port(ee_model_instance));

  // Publish the simulated state.
  auto state_pub =
      builder->AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          state_output_channel, lcm, 1.0 / publish_rate));

  // Publish only the end-effector model instance state (EE-only), not the
  // full printer model instance, so the published `lcmt_robot_output` has
  // the EE positions/velocities (expected total size 6).
  auto state_sender = builder->AddSystem<RobotOutputSender>(
      plant, ee_model_instance, publish_efforts);

  builder->Connect(plant.get_state_output_port(ee_model_instance),
                   state_sender->get_input_port_state());

  if (publish_efforts) {
    builder->Connect(plant.get_net_actuation_output_port(ee_model_instance),
                     state_sender->get_input_port_effort());
  }

  builder->Connect(*state_sender, *state_pub);

  return state_receiver;
}

}  // namespace systems
}  // namespace dairlib
