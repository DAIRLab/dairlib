#include "systems/franka_kinematics.h"

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
    : franka_plant_(franka_plant),
      franka_context_(franka_context),
      object_plant_(object_plant),
      object_context_(object_context),
      world_(franka_plant_.world_frame()),
      end_effector_name_(end_effector_name),
      object_name_(object_name),
      include_end_effector_orientation_(include_end_effector_orientation) {
  this->set_name("franka_kinematics");
  franka_state_port_ =
      this->DeclareVectorInputPort(
              "x_franka", OutputVector<double>(franka_plant.num_positions(),
                                               franka_plant.num_velocities(),
                                               franka_plant.num_actuators()))
          .get_index();

  object_state_port_ =
      this->DeclareVectorInputPort(
              "x_object", StateVector<double>(object_plant.num_positions(),
                                               object_plant.num_velocities()))
          .get_index();
  num_end_effector_positions_ = 3 + include_end_effector_orientation_ * 3;
  num_object_positions_ = 7 * 2;
  num_end_effector_velocities_ = 3 + include_end_effector_orientation_ * 3;
  num_object_velocities_ = 6 * 2;
  lcs_state_port_ =
      this->DeclareVectorOutputPort(
              "x_lcs",
              FrankaKinematicsVector<double>(
                  num_end_effector_positions_, num_object_positions_,
                  num_end_effector_velocities_, num_object_velocities_),
              &FrankaKinematics::ComputeLCSState)
          .get_index();
}

FrankaKinematics::FrankaKinematics(const MultibodyPlant<double>& franka_plant,
                                   Context<double>* franka_context,
                                   const MultibodyPlant<double>& object_plant,
                                   Context<double>* object_context,
                                   const std::string& end_effector_name,
                                   const std::string& object_name,
                                   bool include_end_effector_orientation,
                                   std::vector<ModelInstanceIndex> object_indices)
    : franka_plant_(franka_plant),
      franka_context_(franka_context),
      object_plant_(object_plant),
      object_context_(object_context),
      world_(franka_plant_.world_frame()),
      end_effector_name_(end_effector_name),
      object_name_(object_name),
      include_end_effector_orientation_(include_end_effector_orientation),
      object_indices_(object_indices) {

  num_objects_ = object_indices_.size();
  this->set_name("franka_kinematics");
  franka_state_port_ =
      this->DeclareVectorInputPort(
              "x_franka", OutputVector<double>(franka_plant.num_positions(),
                                               franka_plant.num_velocities(),
                                               franka_plant.num_actuators()))
          .get_index();

  for (int i = 0; i < num_objects_; i++) {
    std::ostringstream oss;
    oss << "x_object_" << i;
    std::string port_name = oss.str();
    object_state_ports_.push_back(
        this->DeclareVectorInputPort(
              port_name, StateVector<double>(object_plant.num_positions(),
                                               object_plant.num_velocities()))
          .get_index()
    );
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
    object_outputs.push_back(
        (StateVector<double>*)this->EvalVectorInput(context, object_state_ports_[i])
    );
  } 

  VectorXd q_franka = franka_output->GetPositions();
  VectorXd v_franka = franka_output->GetVelocities();

  VectorXd q_objects;
  for (int i = 0; i < num_objects_; i++) {
    q_objects << object_outputs[i]->GetPositions();
  }
  VectorXd v_objects;
  for (int i = 0; i < num_objects_; i++) {
    v_objects << object_outputs[i]->GetVelocities();
  }

  multibody::SetPositionsIfNew<double>(franka_plant_, q_franka,
                                       franka_context_);
  multibody::SetVelocitiesIfNew<double>(franka_plant_, v_franka,
                                        franka_context_);

  multibody::SetPositionsIfNew<double>(object_plant_, q_objects,
                                       object_context_);
  multibody::SetVelocitiesIfNew<double>(object_plant_, v_objects,
                                        object_context_);

  auto end_effector_pose = franka_plant_.EvalBodyPoseInWorld(
      *franka_context_, franka_plant_.GetBodyByName(end_effector_name_));

  std::vector<drake::math::RigidTransform<double>> object_poses;
  for (ModelInstanceIndex idx : object_indices_) {
    object_poses.push_back(
        object_plant_.EvalBodyPoseInWorld(
            *object_context_, object_plant_.GetBodyByName(object_name_, idx))
    );
  } 
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

  VectorXd object_positions(num_objects_ * 7);
  for (int i = 0; i < object_poses.size(); i++) {
    object_positions.segment(i * 7, 4) = q_objects.segment(i * 7, 4); // ith orientation
    object_positions.segment(i * 7 + 4, 3) = object_poses[i].translation(); // ith position
  } 

  lcs_state->SetEndEffectorPositions(end_effector_positions);
  lcs_state->SetObjectPositions(object_positions);
  lcs_state->SetEndEffectorVelocities(end_effector_velocities);
  lcs_state->SetObjectVelocities(v_objects);
  lcs_state->set_timestamp(franka_output->get_timestamp());
}

}  // namespace systems
}  // namespace dairlib
