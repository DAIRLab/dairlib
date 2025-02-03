#include "robot_lcm_systems.h"

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "multibody/multibody_utils.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/discrete_time_delay.h"

#include <drake/multibody/plant/point_pair_contact_info.h>

#include "drake/common/text_logging.h"

namespace dairlib {
namespace systems {

using drake::multibody::JointActuatorIndex;
using drake::multibody::JointIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
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
    drake::multibody::ModelInstanceIndex model_instance){
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
    int j = position_index_map_.at(state_msg.position_names[i]);
    positions(j - positions_start_idx_) = state_msg.position[i];
  }
  VectorXd velocities = VectorXd::Zero(num_velocities_);
  for (int i = 0; i < state_msg.num_velocities; i++) {
    int j = velocity_index_map_.at(state_msg.velocity_names[i]);
    velocities(j - velocities_start_idx_) = state_msg.velocity[i];
  }
  VectorXd efforts = VectorXd::Zero(num_efforts_);
  for (int i = 0; i < state_msg.num_efforts; i++) {
    int j = effort_index_map_.at(state_msg.effort_names[i]);
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
  std::vector<std::string> ordered_effort_names =
      multibody::ExtractOrderedNamesFromMap(effort_index_map_);

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
  if (model_instance_ != drake::multibody::ModelInstanceIndex(1000)) {
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
ContactDataSender::ContactDataSender(
  const drake::multibody::MultibodyPlant<double>& plant): 
  plant_(plant){
  num_positions_ = plant.num_positions();
  num_velocities_ = plant.num_velocities();

  this->DeclareAbstractInputPort("x", drake::Value<drake::multibody::ContactResults<double>>{}).get_index();
  this->DeclareAbstractOutputPort("lcmt_densetact_measurement_data", &ContactDataSender::Output);
  }

void ContactDataSender::Output(
  const drake::systems::Context<double>& context,
  dairlib::lcmt_densetact_measurement_data* contact_output) const
  {
  
  // const auto& contact_results = this->Eval<drake::multibody::ContactResults<double>>(context);
  const drake::AbstractValue* contact_results = this->EvalAbstractInput(context, 0);//.num_point_pair_contacts();

  const auto& constact_result = contact_results->get_value<drake::multibody::ContactResults<double>>();

  const std::string sensor1_name = "densetact_0";
  const std::string sensor2_name = "densetact_120";
  //const std::string object_name = "examples/trifinger/robot_properties_fingers/cube/cube_v2.urdf";
  const std::string object_name = "cube";

  const drake::multibody::Body<double>& sensor1_body = plant_.GetBodyByName(sensor1_name);
  const drake::multibody::Body<double>& sensor2_body = plant_.GetBodyByName(sensor2_name);
  const drake::multibody::Body<double>& object_body = plant_.GetBodyByName(object_name);
  drake::multibody::BodyIndex sensor1_index = sensor1_body.index();
  drake::multibody::BodyIndex sensor2_index = sensor2_body.index();
  drake::multibody::BodyIndex object_index = object_body.index();
  
  const int num_of_contacts = constact_result.num_point_pair_contacts();

  bool contact_bool = false;

  Eigen::Vector3<double> contact_force = Eigen::Vector3<double>::Zero();

  for (int i = 0; i < num_of_contacts; i++){
    const auto& contact_instance = constact_result.point_pair_contact_info(i);

    if (((contact_instance.bodyA_index() == sensor1_index) && (contact_instance.bodyB_index() == object_index)) 
        || ((contact_instance.bodyA_index() == object_index) && (contact_instance.bodyB_index() == sensor1_index))){
          
          // Returns the contact force f_Bc_W on B at contact point C expressed in the world frame W. 
          contact_force =  contact_instance.contact_force();

          // Returns the position p_WC of the contact point C in the world frame W. 
          //contact_pose_W = contact_instance.point_pair();

          contact_bool = true;
        }
    }


  contact_output->numSensors = 2;
  contact_output->sensorData.resize(contact_output->numSensors);

  for (int i = 0; i < contact_output->numSensors; i++){
    dairlib::lcmt_densetact_measurement contact;
    contact.utime = context.get_time() * 1e6; 
    contact.inContact = contact_bool;
    //contact.contactPose = contact_pose_W;

    contact.scaledNormal = contact_force[0];
    contact.scaledFriction[0] = contact_force[1];
    contact.scaledFriction[1] = contact_force[2];
    contact_output->sensorData[i] = contact;
  }
  
  // plant_.GetPositions(*plant_context_);
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

  positions_start_idx_ =
      plant.get_joint(plant.GetJointIndices(model_instance).front())
          .position_start();
  velocities_start_idx_ =
      plant.get_joint(plant.GetJointIndices(model_instance).front())
          .velocity_start();
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
  if (model_instance_ != drake::multibody::ModelInstanceIndex(1000)) {
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
    drake::multibody::ModelInstanceIndex model_instance)
    : model_instance_(model_instance) {
  num_positions_ = plant.num_positions(model_instance);
  num_velocities_ = plant.num_velocities(model_instance);

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
  state_msg->object_name = "cube";
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
    //    state_msg->velocity[i] = state->GetAtIndex(num_positions_ + i);
    state_msg->velocity[i] = 0;
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

}  // namespace systems
}  // namespace dairlib
