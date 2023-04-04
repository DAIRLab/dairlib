#include "examples/franka/franka_kinematics.h"

#include <iostream>

#include "common/find_resource.h"
#include "multibody/multibody_utils.h"

namespace dairlib {

using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using Eigen::VectorXd;
using systems::OutputVector;
using systems::TimestampedVector;

namespace systems {

FrankaKinematics::FrankaKinematics(const MultibodyPlant<double>& franka_plant,
                                   Context<double>* franka_context,
                                   const MultibodyPlant<double>& object_plant,
                                   Context<double>* object_context,
                                   const std::string& end_effector_name,
                                   const std::string& object_name)
    : franka_plant_(franka_plant),
      franka_context_(franka_context),
      object_plant_(object_plant),
      object_context_(object_context),
      world_(franka_plant_.world_frame()),
      end_effector_name_(end_effector_name),
      object_name_(object_name){
  this->set_name("franka_kinematics");
  franka_state_port_ =
      this->DeclareVectorInputPort(
              "x_franka", OutputVector<double>(franka_plant.num_positions(),
                                               franka_plant.num_velocities(),
                                               franka_plant.num_actuators()))
          .get_index();

  object_state_port_ =
      this->DeclareVectorInputPort(
              "x_object", OutputVector<double>(object_plant.num_positions(),
                                               object_plant.num_velocities(),
                                               object_plant.num_actuators()))
          .get_index();

  lcs_state_port_ =
      this->DeclareVectorOutputPort("lcs_state",
                                    TimestampedVector<double>(3 + 7 + 3 + 6),
                                    &FrankaKinematics::ComputeLCSState)
          .get_index();
}

void FrankaKinematics::ComputeLCSState(
    const drake::systems::Context<double>& context,
    TimestampedVector<double>* lcs_state) const {
  const OutputVector<double>* franka_output =
      (OutputVector<double>*)this->EvalVectorInput(context, franka_state_port_);
  const OutputVector<double>* object_output =
      (OutputVector<double>*)this->EvalVectorInput(context, object_state_port_);

  VectorXd q_franka = franka_output->GetPositions();
  VectorXd v_franka = franka_output->GetVelocities();
  VectorXd q_object = object_output->GetPositions();
  VectorXd v_object = object_output->GetVelocities();
  multibody::SetPositionsIfNew<double>(franka_plant_, q_franka,
                                       franka_context_);
  multibody::SetVelocitiesIfNew<double>(franka_plant_, v_franka,
                                        franka_context_);
  multibody::SetPositionsIfNew<double>(object_plant_, q_object,
                                       object_context_);
  multibody::SetVelocitiesIfNew<double>(object_plant_, v_object,
                                        object_context_);

  auto end_effector_pose = franka_plant_.EvalBodyPoseInWorld(
      *franka_context_, franka_plant_.GetBodyByName(end_effector_name_));
//  auto franka_base_pose = franka_plant_.EvalBodyPoseInWorld(
//      *franka_context_, franka_plant_.GetBodyByName("panda_link0"));
  auto object_pose = object_plant_.EvalBodyPoseInWorld(
      *object_context_, object_plant_.GetBodyByName(object_name_));
  auto relative_translation = object_pose.translation();
  relative_translation(2) -= 0.7645;
  auto end_effector_spatial_velocity =
      franka_plant_.EvalBodySpatialVelocityInWorld(
          *franka_context_, franka_plant_.GetBodyByName(end_effector_name_));
  VectorXd lcs_output = VectorXd::Zero(lcs_state->size() - 1);
  lcs_output << end_effector_pose.translation(), q_object.head(4), relative_translation,
      end_effector_spatial_velocity.translational(), v_object;

  lcs_state->set_timestamp(franka_output->get_timestamp());
  lcs_state->SetDataVector(lcs_output);
}

}  // namespace systems
}  // namespace dairlib
