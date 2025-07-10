#include "examples/trifinger/systems/trifinger_kinematics.h"

#include "common/find_resource.h"
#include "multibody/multibody_utils.h"

namespace dairlib {

using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using Eigen::VectorXd;
using systems::OutputVector;
using systems::StateVector;

namespace systems {

TrifingerKinematics::TrifingerKinematics(
    const MultibodyPlant<double>& trifinger_plant,
    Context<double>* trifinger_context,
    const MultibodyPlant<double>& object_plant, Context<double>* object_context,
    const std::string& fingertip_0_name, const std::string& fingertip_120_name,
    const std::string& fingertip_240_name, const std::string& object_name)
    : trifinger_plant_(trifinger_plant),
      trifinger_context_(trifinger_context),
      object_plant_(object_plant),
      object_context_(object_context),
      world_(trifinger_plant_.world_frame()),
      fingertip_0_name_(fingertip_0_name),
      fingertip_120_name_(fingertip_120_name),
      fingertip_240_name_(fingertip_240_name),
      object_name_(object_name) {
  this->set_name("trifinger_kinematics");
  trifinger_state_port_ =
      this->DeclareVectorInputPort(
              "x_trifinger",
              OutputVector<double>(trifinger_plant.num_positions(),
                                   trifinger_plant.num_velocities(),
                                   trifinger_plant.num_actuators()))
          .get_index();

  object_state_port_ =
      this->DeclareVectorInputPort(
              "x_object", StateVector<double>(object_plant.num_positions(),
                                              object_plant.num_velocities()))
          .get_index();
  num_fingertip_positions_ = 9;
  num_object_positions_ = 7;
  num_fingertip_velocities_ = 9;
  num_object_velocities_ = 6;
  lcs_state_port_ =
      this->DeclareVectorOutputPort(
              "x_lcs",
              TrifingerKinematicsVector<double>(
                  num_fingertip_positions_, num_object_positions_,
                  num_fingertip_velocities_, num_object_velocities_),
              &TrifingerKinematics::ComputeLCSState)
          .get_index();
}

void TrifingerKinematics::ComputeLCSState(
    const drake::systems::Context<double>& context,
    TrifingerKinematicsVector<double>* lcs_state) const {
  const OutputVector<double>* trifinger_output =
      (OutputVector<double>*)this->EvalVectorInput(context,
                                                   trifinger_state_port_);
  const StateVector<double>* object_output =
      (StateVector<double>*)this->EvalVectorInput(context, object_state_port_);

  VectorXd q_trifinger = trifinger_output->GetPositions();
  VectorXd v_trifinger = trifinger_output->GetVelocities();
  VectorXd q_object = object_output->GetPositions();
  VectorXd v_object = object_output->GetVelocities();
  multibody::SetPositionsIfNew<double>(trifinger_plant_, q_trifinger,
                                       trifinger_context_);
  multibody::SetVelocitiesIfNew<double>(trifinger_plant_, v_trifinger,
                                        trifinger_context_);
  multibody::SetPositionsIfNew<double>(object_plant_, q_object,
                                       object_context_);
  multibody::SetVelocitiesIfNew<double>(object_plant_, v_object,
                                        object_context_);

  auto fingertip_0_pos =
      trifinger_plant_
          .EvalBodyPoseInWorld(
              *trifinger_context_,
              trifinger_plant_.GetBodyByName(fingertip_0_name_))
          .translation();
  auto fingertip_0_vel =
      trifinger_plant_
          .EvalBodySpatialVelocityInWorld(
              *trifinger_context_,
              trifinger_plant_.GetBodyByName(fingertip_0_name_))
          .translational();
  auto fingertip_120_pos =
      trifinger_plant_
          .EvalBodyPoseInWorld(
              *trifinger_context_,
              trifinger_plant_.GetBodyByName(fingertip_120_name_))
          .translation();
  auto fingertip_120_vel =
      trifinger_plant_
          .EvalBodySpatialVelocityInWorld(
              *trifinger_context_,
              trifinger_plant_.GetBodyByName(fingertip_120_name_))
          .translational();
  auto fingertip_240_pos =
      trifinger_plant_
          .EvalBodyPoseInWorld(
              *trifinger_context_,
              trifinger_plant_.GetBodyByName(fingertip_240_name_))
          .translation();
  auto fingertip_240_vel =
      trifinger_plant_
          .EvalBodySpatialVelocityInWorld(
              *trifinger_context_,
              trifinger_plant_.GetBodyByName(fingertip_240_name_))
          .translational();

  auto object_pose = object_plant_.EvalBodyPoseInWorld(
      *object_context_, object_plant_.GetBodyByName(object_name_));

  VectorXd object_position = q_object;
  object_position << q_object.head(4), object_pose.translation();

  lcs_state->SetFingertip0Positions(fingertip_0_pos);
  lcs_state->SetFingertip120Positions(fingertip_120_pos);
  lcs_state->SetFingertip240Positions(fingertip_240_pos);
  lcs_state->SetObjectPositions(object_position);
  lcs_state->SetFingertip0Velocities(fingertip_0_vel);
  lcs_state->SetFingertip120Velocities(fingertip_120_vel);
  lcs_state->SetFingertip240Velocities(fingertip_240_vel);
  lcs_state->SetObjectVelocities(v_object);
  lcs_state->set_timestamp(trifinger_output->get_timestamp());
}

}  // namespace systems
}  // namespace dairlib
