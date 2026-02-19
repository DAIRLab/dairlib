#include "systems/franka_kinematics.h"

#include <iostream>

#include "common/find_resource.h"
#include "dairlib/lcmt_elastoplastic_network.hpp"

namespace dairlib {
namespace systems {

FrankaKinematics::FrankaKinematics(const MultibodyPlant<double>& franka_plant,
                                   Context<double>* franka_context,
                                   const std::string& end_effector_name,
                                   const bool& include_end_effector_orientation)
    : FrankaKinematics(franka_plant, franka_context, nullptr, nullptr, nullptr,
                       nullptr, end_effector_name, std::vector<std::string>{},
                       0, include_end_effector_orientation) {}

FrankaKinematics::FrankaKinematics(const MultibodyPlant<double>& franka_plant,
                                   Context<double>* franka_context,
                                   MultibodyPlant<double>* object_plant,
                                   Context<double>* object_context,
                                   const std::string& end_effector_name,
                                   const std::string& object_name,
                                   const bool& include_end_effector_orientation)
    : FrankaKinematics(franka_plant, franka_context, object_plant,
                       object_context, nullptr, nullptr, end_effector_name,
                       std::vector<std::string>{object_name}, 0,
                       include_end_effector_orientation) {}

FrankaKinematics::FrankaKinematics(const MultibodyPlant<double>& franka_plant,
                                   Context<double>* franka_context,
                                   MultibodyPlant<double>* object_plant,
                                   Context<double>* object_context,
                                   const std::string& end_effector_name,
                                   const std::vector<std::string>& object_names,
                                   const bool& include_end_effector_orientation)
    : FrankaKinematics(franka_plant, franka_context, object_plant,
                       object_context, nullptr, nullptr, end_effector_name,
                       object_names, 0, include_end_effector_orientation) {}

FrankaKinematics::FrankaKinematics(const MultibodyPlant<double>& franka_plant,
                                   Context<double>* franka_context,
                                   MultibodyPlant<double>* elastoplastic_plant,
                                   Context<double>* elastoplastic_context,
                                   const std::string& end_effector_name,
                                   const int& num_elastoplastic_nodes,
                                   const bool& include_end_effector_orientation)
    : FrankaKinematics(
          franka_plant, franka_context, nullptr, nullptr, elastoplastic_plant,
          elastoplastic_context, end_effector_name, std::vector<std::string>{},
          num_elastoplastic_nodes, include_end_effector_orientation) {}

FrankaKinematics::FrankaKinematics(const MultibodyPlant<double>& franka_plant,
                                   Context<double>* franka_context,
                                   MultibodyPlant<double>* object_plant,
                                   Context<double>* object_context,
                                   MultibodyPlant<double>* elastoplastic_plant,
                                   Context<double>* elastoplastic_context,
                                   const std::string& end_effector_name,
                                   const std::vector<std::string>& object_names,
                                   const int& num_elastoplastic_nodes,
                                   const bool& include_end_effector_orientation)
    : franka_plant_(franka_plant),
      franka_context_(franka_context),
      object_plant_(object_plant),
      object_context_(object_context),
      elastoplastic_plant_(elastoplastic_plant),
      elastoplastic_context_(elastoplastic_context),
      world_(franka_plant_.world_frame()),
      end_effector_name_(end_effector_name),
      object_names_(object_names),
      num_objects_(object_names.size()),
      num_elastoplastic_nodes_(num_elastoplastic_nodes),
      include_end_effector_orientation_(include_end_effector_orientation) {
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
  if (num_elastoplastic_nodes_ > 0) {
    elastoplastic_network_port_ =
        this->DeclareAbstractInputPort(
                "lcmt_elastoplastic_network",
                drake::Value<dairlib::lcmt_elastoplastic_network>{})
            .get_index();
  }

  num_end_effector_positions_ = 3 + include_end_effector_orientation_ * 3;
  num_end_effector_velocities_ = 3 + include_end_effector_orientation_ * 3;
  num_object_positions_ = num_objects_ == 0 ? 0 : object_plant->num_positions();
  num_object_velocities_ =
      num_objects_ == 0 ? 0 : object_plant->num_velocities();
  num_node_positions_ = num_elastoplastic_nodes_ * 3;
  num_node_velocities_ = num_elastoplastic_nodes_ * 3;
  lcs_state_port_ =
      this->DeclareVectorOutputPort(
              "x_lcs",
              FrankaKinematicsVector<double>(
                  num_end_effector_positions_,
                  std::max({num_object_positions_, num_node_positions_}),
                  num_end_effector_velocities_,
                  std::max({num_object_velocities_, num_node_velocities_})),
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

  // Second, prepare to evaluate the object(s) states.
  int nq = num_elastoplastic_nodes_ != 0 ? 3 * num_elastoplastic_nodes_
           : num_objects_ == 0
               ? 0
               : num_objects_ * object_outputs[0]->GetPositions().size();
  int nv = num_elastoplastic_nodes_ != 0 ? 3 * num_elastoplastic_nodes_
           : num_objects_ == 0
               ? 0
               : num_objects_ * object_outputs[0]->GetVelocities().size();
  VectorXd q_objects = VectorXd::Zero(nq);
  VectorXd v_objects = VectorXd::Zero(nv);

  // Handle the one or several rigid body case.
  if (num_objects_ > 0) {
    for (int i = 0; i < num_objects_; i++) {
      q_objects.segment(i * 7, 7) = object_outputs.at(i)->GetPositions();
      v_objects.segment(i * 6, 6) = object_outputs.at(i)->GetVelocities();
    }
  }
  // Handle the elastoplastic network case.
  else {
    const auto& elastoplastic_network_lcmt =
        this->EvalInputValue<dairlib::lcmt_elastoplastic_network>(
            context, elastoplastic_network_port_);
    for (int point_i = 0; point_i < num_elastoplastic_nodes_; point_i++) {
      std::vector<float> point = elastoplastic_network_lcmt->points[point_i];
      for (int dim_i = 0; dim_i < 3; dim_i++) {
        q_objects(3 * point_i + dim_i) = point[dim_i];
      }
    }
    // NOTE:  This leaves node velocities at zero.
  }

  lcs_state->SetEndEffectorPositions(end_effector_positions);
  lcs_state->SetObjectPositions(q_objects);
  lcs_state->SetEndEffectorVelocities(end_effector_velocities);
  lcs_state->SetObjectVelocities(v_objects);
  lcs_state->set_timestamp(franka_output->get_timestamp());
}

}  // namespace systems
}  // namespace dairlib
