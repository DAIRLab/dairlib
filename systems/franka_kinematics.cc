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
                                   std::vector<std::string> object_names)
    : franka_plant_(franka_plant),
      franka_context_(franka_context),
      object_plant_(object_plant),
      object_context_(object_context),
      world_(franka_plant_.world_frame()),
      end_effector_name_(end_effector_name),
      object_name_(object_name),
      include_end_effector_orientation_(include_end_effector_orientation),
      object_names_(object_names) {

  std::cout << "[FrankaKinematics] Constructor called." << std::endl;
  std::cout << "  end_effector_name = " << end_effector_name_ << std::endl;
  std::cout << "  object_name = " << object_name_ << std::endl;
  std::cout << "  include_end_effector_orientation = "
            << include_end_effector_orientation_ << std::endl;
  std::cout << "  object_names.size() = " << object_names_.size() << std::endl;

  num_objects_ = object_names_.size();
  std::cout << "  num_objects_ = " << num_objects_ << std::endl;

  this->set_name("franka_kinematics");
  std::cout << "  System name set to franka_kinematics." << std::endl;

  franka_state_port_ =
      this->DeclareVectorInputPort(
              "x_franka", OutputVector<double>(franka_plant.num_positions(),
                                               franka_plant.num_velocities(),
                                               franka_plant.num_actuators()))
          .get_index();

  std::cout << "  Declared franka_state_port_ = "
            << franka_state_port_ << std::endl;
  std::cout << "    franka num_positions = "
            << franka_plant.num_positions() << std::endl;
  std::cout << "    franka num_velocities = "
            << franka_plant.num_velocities() << std::endl;
  std::cout << "    franka num_actuators = "
            << franka_plant.num_actuators() << std::endl;

  for (int i = 0; i < num_objects_; i++) {
    std::string port_name = "x_object_" + std::to_string(i);

    std::cout << "  Declaring object input port " << i
              << " with name \"" << port_name << "\"" << std::endl;

    object_state_ports_.push_back(
        this->DeclareVectorInputPort(
              port_name, StateVector<double>(7, 6))
          .get_index()
    );

    std::cout << "    -> port index = "
              << object_state_ports_.back() << std::endl;
  }

  num_end_effector_positions_ = 3 + include_end_effector_orientation_ * 3;
  num_object_positions_ = object_plant.num_positions();
  num_end_effector_velocities_ = 3 + include_end_effector_orientation_ * 3;
  num_object_velocities_ = object_plant.num_velocities();

  std::cout << "  num_end_effector_positions_ = "
            << num_end_effector_positions_ << std::endl;
  std::cout << "  num_object_positions_ = "
            << num_object_positions_ << std::endl;
  std::cout << "  num_end_effector_velocities_ = "
            << num_end_effector_velocities_ << std::endl;
  std::cout << "  num_object_velocities_ = "
            << num_object_velocities_ << std::endl;

  lcs_state_port_ =
      this->DeclareVectorOutputPort(
              "x_lcs",
              FrankaKinematicsVector<double>(
                  num_end_effector_positions_, num_object_positions_,
                  num_end_effector_velocities_, num_object_velocities_),
              &FrankaKinematics::ComputeLCSState)
          .get_index();

  std::cout << "  Declared lcs_state_port_ = "
            << lcs_state_port_ << std::endl;
  std::cout << "[FrankaKinematics] Constructor complete." << std::endl;
}

void FrankaKinematics::ComputeLCSState(
    const drake::systems::Context<double>& context,
    FrankaKinematicsVector<double>* lcs_state) const {
  const OutputVector<double>* franka_output =
      (OutputVector<double>*)this->EvalVectorInput(context, franka_state_port_);

  std::vector<const StateVector<double>*> object_outputs;
  for (int i = 0; i < num_objects_; i++) {
    object_outputs.push_back(
        (StateVector<double>*)this->EvalVectorInput(context, object_state_ports_.at(i))
    );
  } 

  VectorXd q_franka = franka_output->GetPositions();
  VectorXd v_franka = franka_output->GetVelocities();

  int nq = object_outputs[0]->GetPositions().size();
  int nv = object_outputs[0]->GetVelocities().size();

  // Preallocate total vectors
  VectorXd q_objects(num_objects_ * nq);
  VectorXd v_objects(num_objects_ * nv);

  for (int i = 0; i < num_objects_; i++) {
    q_objects.segment(i * nq, nq) = object_outputs.at(i)->GetPositions();
    v_objects.segment(i * nv, nv) = object_outputs.at(i)->GetVelocities();
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

  const Eigen::VectorXd& q = object_plant_.GetPositions(*object_context_);

  std::vector<drake::math::RigidTransform<double>> object_poses;
  for (std::string name : object_names_) {
    object_poses.push_back(
          object_plant_.EvalBodyPoseInWorld(
              *object_context_, object_plant_.GetBodyByName(name))
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
