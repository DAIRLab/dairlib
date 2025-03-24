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
#include "drake/common/schema/transform.h"
#include <drake/multibody/parsing/model_directives.h>


#include <iostream>
#include <cmath>

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

ContactDataSender::ContactDataSender(const drake::multibody::MultibodyPlant<double>& plant): 
  plant_(plant) {
  //plant_context_ = &plant_.GetMyContextFromRoot(diagram_context);
  plant_context_ = plant.CreateDefaultContext().release();
  num_positions_ = plant.num_positions();
  num_velocities_ = plant.num_velocities();

  //plant_context_ = plant.CreateDefaultContext().release();

  this->DeclareAbstractInputPort("x_contact_force", drake::Value<drake::multibody::ContactResults<double>>{}).get_index();
  //this->DeclareVectorInputPort("x_state", drake::BasicVector<drake::VectorX<double>>{}).get_index();
  this->DeclareVectorInputPort("x_state", drake::systems::BasicVector<double>(num_positions_ + num_velocities_)).get_index();

  this->DeclareAbstractOutputPort("lcmt_densetact_measurement_data", &ContactDataSender::Output);
  }


bool ContactDataSender::IsContactBetween(const drake::multibody::PointPairContactInfo<double>& contact,
  drake::multibody::BodyIndex body1_index,
  drake::multibody::BodyIndex body2_index) const{
  return ((contact.bodyA_index() == body1_index && contact.bodyB_index() == body2_index) ||
          (contact.bodyA_index() == body2_index && contact.bodyB_index() == body1_index));
  }

bool ContactDataSender::IsContact(int num_of_contacts) const{
  return (num_of_contacts > 0);
  }

Eigen::Vector3d ContactDataSender::GetAverage(const Eigen::Vector3d& points, int num_of_contacts) const{
  if (num_of_contacts > 0) {
    return points / num_of_contacts; 
  }
  return Eigen::Vector3d::Zero();
} 

Eigen::Matrix3d ContactDataSender::GetContactFrame(const bool contact_bool, const Eigen::Vector3d& normal_W) const{
  // normal_W is already normalized

  if (contact_bool){ 
    const double nx =  normal_W.coeff(0);
    const double ny =  normal_W.coeff(1);
    const double nz =  normal_W.coeff(2);

    Eigen::Vector3d t1; 
    t1 << -ny/(std::sqrt(ny*ny + nx*nx)), nx/std::sqrt(ny*ny + nx*nx), 0;

    Eigen::Vector3d t2 = normal_W.cross(t1);

    Eigen::Matrix3d contact_frame;
    contact_frame << normal_W, t1, -t2;
    return contact_frame;
  }

  return Eigen::Matrix3d::Identity();

  }

void ContactDataSender::Output(
  const drake::systems::Context<double>& context, 
  dairlib::lcmt_densetact_measurement_data* contact_output) const
  {

  auto q_init = this->EvalVectorInput(context, 1);
  const drake::AbstractValue* contact_results = this->EvalAbstractInput(context, 0);//.num_point_pair_contacts();

  plant_.SetPositionsAndVelocities(plant_context_, q_init->value());

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

  // Eigen::Matrix4d sensor1_BW = drake::multibody::RigidBody<double>& get_rotation_matrix_in_world(plant_.GetRigidBodyByName(sensor1_name));
  // Eigen::Matrix4d sensor2_BW = drake::multibody::RigidBody<double>& get_rotation_matrix_in_world(plant_.GetRigidBodyByName(sensor2_name));

  const drake::multibody::RigidBody<double>& sensor1_pose_W = plant_.GetRigidBodyByName(sensor1_name);
  const drake::multibody::RigidBody<double>& sensor2_pose_W = plant_.GetRigidBodyByName(sensor2_name);

  Eigen::Matrix4d sensor1_pose_mat_W = sensor1_pose_W.EvalPoseInWorld(*plant_context_).GetAsMatrix4();
  Eigen::Matrix4d sensor2_pose_mat_W = sensor2_pose_W.EvalPoseInWorld(*plant_context_).GetAsMatrix4();

  Eigen::Matrix3d sensor1_R_BW = sensor1_pose_mat_W.block(0,0,3,3);
  Eigen::Matrix3d sensor2_R_BW = sensor2_pose_mat_W.block(0,0,3,3);

  const int num_of_totals_contacts = constact_result.num_point_pair_contacts();
  int num_of_sensor1_contacts = 0;
  int num_of_sensor2_contacts = 0;

  // initalize contact forces in W frame
  Eigen::Vector3d contact_force_sensor1_W = Eigen::Vector3d::Zero();  
  Eigen::Vector3d contact_force_sensor2_W = Eigen::Vector3d::Zero();
  // Eigen::Vector4d contact_force_sensor1_B;
  // Eigen::Vector4d contact_force_sensor2_B;

  Eigen::Vector3d contact_normal_sensor1_W = Eigen::Vector3d::Zero();  
  Eigen::Vector3d contact_normal_sensor2_W = Eigen::Vector3d::Zero();


  // initalize contact frames which are spherical (i.e rho, theta, phi) in W frame expressed in W frame
  //Eigen::Matrix3d contact_frame_sensor1_CB = Eigen::Matrix4d::Identity();
  //Eigen::Matrix3d contact_frame_sensor2_CB = Eigen::Matrix4d::Identity();
  
  //for every contact instance
  for (int i = 0; i < num_of_totals_contacts; i++){
    const auto& contact_instance = constact_result.point_pair_contact_info(i);
    if (IsContactBetween(contact_instance, object_index, sensor1_index)){
          num_of_sensor1_contacts++;

          // Returns the contact force f_Bc_W on B at contact point C expressed in the world frame W
          contact_force_sensor1_W += contact_instance.contact_force();
          contact_normal_sensor1_W += contact_instance.point_pair().nhat_BA_W;
        }
    
    if (IsContactBetween(contact_instance, object_index, sensor2_index)){
          num_of_sensor2_contacts++;

          // Returns the contact force f_Bc_W on B at contact point C expressed in the world frame W in homogenous coords 
          contact_force_sensor2_W += contact_instance.contact_force();
          contact_normal_sensor2_W += contact_instance.point_pair().nhat_BA_W;
    }
  }
  // std::cout << "Number of contacts: " << num_of_totals_contacts << std::endl;
  // std::cout << "Number of contacts snesor1: " << num_of_sensor1_contacts << std::endl;
  // std::cout << "Number of contacts snesor2: " << num_of_sensor2_contacts << std::endl;

  contact_force_sensor1_W = GetAverage(contact_force_sensor1_W, num_of_sensor1_contacts);
  contact_force_sensor2_W = GetAverage(contact_force_sensor2_W, num_of_sensor2_contacts);

  Eigen::Vector3d normal_W_sensor_1_W = GetAverage(contact_normal_sensor1_W, num_of_sensor1_contacts);
  Eigen::Vector3d normal_W_sensor_2_W = GetAverage(contact_normal_sensor2_W, num_of_sensor2_contacts);

  bool contact_bool_1 = IsContact(num_of_sensor1_contacts);
  bool contact_bool_2 = IsContact(num_of_sensor2_contacts);

  if (!contact_bool_1){
    sensor1_R_BW = Eigen::Matrix3d::Identity();
  }
  if (!contact_bool_2){
    sensor2_R_BW = Eigen::Matrix3d::Identity();;
  }


  Eigen::Matrix3d contact_frame_sensor1_CW = GetContactFrame(contact_bool_1, normal_W_sensor_1_W);
  Eigen::Matrix3d contact_frame_sensor2_CW = GetContactFrame(contact_bool_2, normal_W_sensor_2_W);

  Eigen::Matrix3d contact_frame_sensor1_CB = sensor1_R_BW.inverse() * contact_frame_sensor1_CW;
  Eigen::Matrix3d contact_frame_sensor2_CB = sensor2_R_BW.inverse() * contact_frame_sensor2_CW;

  // std::cout << "sensor1_R_BW" << sensor1_R_BW << std::endl;
  // std::cout << "contact_frame_sensor1_CW" << contact_frame_sensor1_CW << std::endl;

  Eigen::Vector3d contact_force_sensor1_B = sensor1_R_BW.inverse() * contact_force_sensor1_W;
  Eigen::Vector3d contact_force_sensor2_B = sensor2_R_BW.inverse() * contact_force_sensor2_W;


  contact_output->numSensors = 2;
  contact_output->sensorData.resize(contact_output->numSensors);

  // DenseTact 1
  dairlib::lcmt_densetact_measurement contact_1;
  contact_1.utime = context.get_time() * 1e6; 
  contact_1.inContact = contact_bool_1;

  for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
          contact_1.contactPose[i][j] = contact_frame_sensor1_CB(i, j);
      }
  } 

  //HACK: output force in world frame
  //Eigen::Vector4d contact_force_sensor1_S = contact_force_sensor1_W;
  contact_1.scaledNormal = contact_force_sensor1_B(0);
  contact_1.scaledFriction[0] = contact_force_sensor1_B(1);
  contact_1.scaledFriction[1] = contact_force_sensor1_B(2);
  contact_output->sensorData[0] = contact_1;

  // DenseTact 2
  dairlib::lcmt_densetact_measurement contact_2;
  contact_2.utime = context.get_time() * 1e6; 
  contact_2.inContact = contact_bool_2;

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
        contact_2.contactPose[i][j] = contact_frame_sensor2_CB(i, j);
      }
  } 

  //Eigen::Vector4d contact_force_sensor2_S = contact_force_sensor2_W;

  contact_2.scaledNormal = contact_force_sensor2_B(0);
  contact_2.scaledFriction[0] = contact_force_sensor2_B(1);
  contact_2.scaledFriction[1] = contact_force_sensor2_B(2);
  contact_output->sensorData[1] = contact_2;
  
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
