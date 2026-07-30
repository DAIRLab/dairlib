#include "systems/franka_kinematics.h"

#include <iostream>

#include "common/find_resource.h"

namespace dairlib {
namespace systems {

FrankaKinematics::FrankaKinematics(const MultibodyPlant<double>& franka_plant,
                                   Context<double>* franka_context,
                                   const MultibodyPlant<double>& object_plant,
                                   Context<double>* object_context,
                                   const std::string& end_effector_name,
                                   const std::string& object_name,
                                   bool include_end_effector_orientation)
    : FrankaKinematics(franka_plant, franka_context, object_plant,
                       object_context, end_effector_name,
                       std::vector<std::string>{object_name},
                       include_end_effector_orientation) {}

FrankaKinematics::FrankaKinematics(const MultibodyPlant<double>& franka_plant,
                                   Context<double>* franka_context,
                                   const MultibodyPlant<double>& object_plant,
                                   Context<double>* object_context,
                                   const std::string& end_effector_name,
                                   std::vector<std::string> object_names,
                                   bool include_end_effector_orientation)
    : franka_plant_(franka_plant),
      franka_context_(franka_context),
      object_plant_(object_plant),
      object_context_(object_context),
      world_(franka_plant_.world_frame()),
      end_effector_name_(end_effector_name),
      include_end_effector_orientation_(include_end_effector_orientation),
      object_names_(object_names) {
  num_objects_ = object_names_.size();
  this->set_name("franka_kinematics");
  franka_state_port_ =
      this->DeclareVectorInputPort(
              "x_franka", OutputVector<double>(franka_plant.num_positions(),
                                               franka_plant.num_velocities(),
                                               franka_plant.num_actuators()))
          .get_index();

  for (int i = 0; i < num_objects_; i++) {
    std::string port_name = "x_object_" + std::to_string(i);
    object_state_ports_.push_back(
        this->DeclareVectorInputPort(port_name, StateVector<double>(7, 6))
            .get_index());
  }

  num_end_effector_positions_ = 3 + include_end_effector_orientation_ * 3;
  num_object_positions_ = object_plant.num_positions();
  num_end_effector_velocities_ = 3 + include_end_effector_orientation_ * 3;
  num_object_velocities_ = object_plant.num_velocities();
  lcs_state_port_ =
      this->DeclareVectorOutputPort(
              "x_lcs",
              FrankaKinematicsVector<double>(
                  num_end_effector_positions_, num_object_positions_,
                  num_end_effector_velocities_, num_object_velocities_),
              &FrankaKinematics::ComputeLCSState)
          .get_index();
}

void FrankaKinematics::ComputeLCSState(
    const drake::systems::Context<double>& context,
    FrankaKinematicsVector<double>* lcs_state) const {
  const OutputVector<double>* franka_output =
      (OutputVector<double>*)this->EvalVectorInput(context, franka_state_port_);

  std::vector<const StateVector<double>*> object_outputs;
  for (int i = 0; i < num_objects_; i++) {
    object_outputs.push_back((StateVector<double>*)this->EvalVectorInput(
        context, object_state_ports_.at(i)));
  }

  // First, evaluate the robot states.
  VectorXd q_franka = franka_output->GetPositions();
  VectorXd v_franka = franka_output->GetVelocities();
  multibody::SetPositionsIfNew<double>(franka_plant_, q_franka,
                                       franka_context_);
  multibody::SetVelocitiesIfNew<double>(franka_plant_, v_franka,
                                        franka_context_);
  auto end_effector_pose = franka_plant_.EvalBodyPoseInWorld(
      *franka_context_, franka_plant_.GetBodyByName(end_effector_name_));
  auto end_effector_spatial_velocity =
      franka_plant_.EvalBodySpatialVelocityInWorld(
          *franka_context_, franka_plant_.GetBodyByName(end_effector_name_));
  auto end_effector_rotation_rpy =
      end_effector_pose.rotation().ToRollPitchYaw().vector();
  VectorXd end_effector_positions = VectorXd::Zero(num_end_effector_positions_);
  VectorXd end_effector_velocities =
      VectorXd::Zero(num_end_effector_velocities_);

  if (num_end_effector_positions_ > 3) {
    end_effector_positions << end_effector_pose.translation(),
        end_effector_rotation_rpy;
  } else {
    end_effector_positions << end_effector_pose.translation();
  }
  if (num_end_effector_velocities_ > 3) {
    end_effector_velocities << end_effector_spatial_velocity.rotational(),
        end_effector_spatial_velocity.translational();
  } else {
    end_effector_velocities << end_effector_spatial_velocity.translational();
  }

  // Second, evaluate the object(s) states.
  int nq = object_outputs[0]->GetPositions().size();
  int nv = object_outputs[0]->GetVelocities().size();
  VectorXd q_objects(num_objects_ * nq);
  VectorXd v_objects(num_objects_ * nv);

  for (int i = 0; i < num_objects_; i++) {
    q_objects.segment(i * nq, nq) = object_outputs.at(i)->GetPositions();
    v_objects.segment(i * nv, nv) = object_outputs.at(i)->GetVelocities();
  }

  lcs_state->SetEndEffectorPositions(end_effector_positions);
  lcs_state->SetObjectPositions(q_objects);
  lcs_state->SetEndEffectorVelocities(end_effector_velocities);
  lcs_state->SetObjectVelocities(v_objects);
  lcs_state->set_timestamp(franka_output->get_timestamp());
}

}  // namespace systems
}  // namespace dairlib
