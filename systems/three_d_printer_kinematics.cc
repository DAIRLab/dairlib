#include "systems/three_d_printer_kinematics.h"

#include <iostream>

#include "common/find_resource.h"

#include "drake/math/rigid_transform.h"
#include "drake/math/roll_pitch_yaw.h"
#include "drake/math/rotation_matrix.h"

using drake::math::RigidTransform;
using drake::math::RollPitchYaw;
using drake::math::RotationMatrix;

namespace dairlib {
namespace systems {

ThreeDPrinterKinematics::ThreeDPrinterKinematics(
    const MultibodyPlant<double>& printer_plant,
    Context<double>* printer_context,
    const MultibodyPlant<double>& object_plant, Context<double>* object_context,
    const std::string& end_effector_name, const std::string& object_name)
    : ThreeDPrinterKinematics(printer_plant, printer_context, object_plant,
                              object_context, end_effector_name,
                              std::vector<std::string>{object_name}) {}

ThreeDPrinterKinematics::ThreeDPrinterKinematics(
    const MultibodyPlant<double>& printer_plant,
    Context<double>* printer_context,
    const MultibodyPlant<double>& object_plant, Context<double>* object_context,
    const std::string& end_effector_name, std::vector<std::string> object_names)
    : printer_plant_(printer_plant),
      printer_context_(printer_context),
      object_plant_(object_plant),
      object_context_(object_context),
      end_effector_name_(end_effector_name),
      object_names_(object_names),
      num_objects_(object_names.size()) {
  this->set_name("three_d_printer_kinematics");
  printer_state_port_ =
      this->DeclareVectorInputPort(
              "x_printer", OutputVector<double>(printer_plant.num_positions(),
                                                printer_plant.num_velocities(),
                                                printer_plant.num_actuators()))
          .get_index();

  for (int i = 0; i < num_objects_; i++) {
    std::string port_name = "x_object_" + std::to_string(i);
    object_state_ports_.push_back(
        this->DeclareVectorInputPort(port_name, StateVector<double>(7, 6))
            .get_index());
  }

  num_object_positions_ = object_plant.num_positions();
  num_object_velocities_ = object_plant.num_velocities();
  lcs_state_port_ =
      this->DeclareVectorOutputPort(
              "x_lcs",
              FrankaKinematicsVector<double>(  // TODO(bibit):  don't use franka
                  num_end_effector_positions_, num_object_positions_,
                  num_end_effector_velocities_, num_object_velocities_),
              &ThreeDPrinterKinematics::ComputeLCSState)
          .get_index();
}

void ThreeDPrinterKinematics::ComputeLCSState(
    const drake::systems::Context<double>& context,
    FrankaKinematicsVector<double>* lcs_state) const {
  const OutputVector<double>* printer_output =
      (OutputVector<double>*)this->EvalVectorInput(context,
                                                   printer_state_port_);

  std::vector<const StateVector<double>*> object_outputs;
  for (int i = 0; i < num_objects_; i++) {
    object_outputs.push_back((StateVector<double>*)this->EvalVectorInput(
        context, object_state_ports_.at(i)));
  }

  VectorXd q_printer = printer_output->GetPositions();
  VectorXd v_printer = printer_output->GetVelocities();

  int nq = object_outputs[0]->GetPositions().size();
  int nv = object_outputs[0]->GetVelocities().size();

  // Preallocate total vectors
  VectorXd q_objects(num_objects_ * nq);
  VectorXd v_objects(num_objects_ * nv);

  for (int i = 0; i < num_objects_; i++) {
    q_objects.segment(i * nq, nq) = object_outputs.at(i)->GetPositions();
    v_objects.segment(i * nv, nv) = object_outputs.at(i)->GetVelocities();
  }

  multibody::SetPositionsIfNew<double>(printer_plant_, q_printer,
                                       printer_context_);
  multibody::SetVelocitiesIfNew<double>(printer_plant_, v_printer,
                                        printer_context_);

  multibody::SetPositionsIfNew<double>(object_plant_, q_objects,
                                       object_context_);
  multibody::SetVelocitiesIfNew<double>(object_plant_, v_objects,
                                        object_context_);

  auto end_effector_pose = printer_plant_.EvalBodyPoseInWorld(
      *printer_context_, printer_plant_.GetBodyByName(end_effector_name_));

  std::vector<drake::math::RigidTransform<double>> object_poses;
  for (std::string name : object_names_) {
    object_poses.push_back(object_plant_.EvalBodyPoseInWorld(
        *object_context_, object_plant_.GetBodyByName(name)));
  }
  auto end_effector_spatial_velocity =
      printer_plant_.EvalBodySpatialVelocityInWorld(
          *printer_context_, printer_plant_.GetBodyByName(end_effector_name_));
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

  VectorXd object_positions(num_objects_ * 7);
  for (int i = 0; i < object_poses.size(); i++) {
    object_positions.segment(i * 7, 4) =
        q_objects.segment(i * 7, 4);  // ith orientation
    object_positions.segment(i * 7 + 4, 3) =
        object_poses[i].translation();  // ith position
  }

  lcs_state->SetEndEffectorPositions(end_effector_positions);
  lcs_state->SetObjectPositions(object_positions);
  lcs_state->SetEndEffectorVelocities(end_effector_velocities);
  lcs_state->SetObjectVelocities(v_objects);
  lcs_state->set_timestamp(printer_output->get_timestamp());
}

}  // namespace systems
}  // namespace dairlib
