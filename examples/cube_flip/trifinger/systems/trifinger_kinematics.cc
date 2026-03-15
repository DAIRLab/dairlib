#include "examples/cube_flip/trifinger/systems/trifinger_kinematics.h"
#include <iostream>

#include "common/find_resource.h"

using drake::math::RigidTransform;
using drake::multibody::SpatialVelocity;

namespace dairlib {

TrifingerKinematics::TrifingerKinematics(const MultibodyPlant<double>& trifinger_plant,
                                   Context<double>* trifinger_context,
                                   const MultibodyPlant<double>& object_plant,
                                   Context<double>* object_context,
                                   const vector<std::string> end_effector_names,
                                   const std::string& object_name)
    : trifinger_plant_(trifinger_plant),
      trifinger_context_(trifinger_context),
      object_plant_(object_plant),
      object_context_(object_context),
      world_(trifinger_plant_.world_frame()),
      end_effector_names_(end_effector_names),
      object_name_(object_name) {

  this->set_name("trifinger_kinematics");


  trifinger_state_port_ =
      this->DeclareVectorInputPort(
              "x_trifinger", OutputVector<double>(trifinger_plant.num_positions(),
                                                  trifinger_plant.num_velocities(),
                                                  trifinger_plant.num_actuators()))
          .get_index();

  object_state_port_ =
      this->DeclareVectorInputPort(
              "x_object", StateVector<double>(object_plant.num_positions(),
                                               object_plant.num_velocities()))
          .get_index();

  // HARDCODED
  num_end_effector_positions_ = 9;
  num_object_positions_ = 7;
  num_end_effector_velocities_ = 9;
  num_object_velocities_ = 6;

  lcs_state_port_ =
      this->DeclareVectorOutputPort(
              "x_lcs",
              TimestampedVector<double>(
                  num_end_effector_positions_ + num_object_positions_ +
                  num_end_effector_velocities_ + num_object_velocities_),
              &TrifingerKinematics::ComputeLCSState)
          .get_index();
}

void TrifingerKinematics::ComputeLCSState(
    const drake::systems::Context<double>& context,
    TimestampedVector<double>* lcs_state) const {

  const OutputVector<double>* trifinger_output =
      (OutputVector<double>*)this->EvalVectorInput(context, trifinger_state_port_);

	const StateVector<double>* object_output =
			(StateVector<double>*)this->EvalVectorInput(context, object_state_port_);

  VectorXd q_trifinger = trifinger_output->GetPositions();
  VectorXd v_trifinger = trifinger_output->GetVelocities();

  int nq = object_output->GetPositions().size();
  int nv = object_output->GetVelocities().size();

  // Preallocate total vectors
  VectorXd q_object(nq);
  VectorXd v_object(nv);

  q_object.segment(0, nq) = object_output->GetPositions();
  v_object.segment(0, nv) = object_output->GetVelocities();

	q_object.segment(0, 4) = q_object.segment(0, 4).normalized(); // Normalize quaternions

  multibody::SetPositionsIfNew<double>(trifinger_plant_, q_trifinger,
                                       trifinger_context_);
  multibody::SetVelocitiesIfNew<double>(trifinger_plant_, v_trifinger,
                                        trifinger_context_);

  multibody::SetPositionsIfNew<double>(object_plant_, q_object,
                                       object_context_);
  multibody::SetVelocitiesIfNew<double>(object_plant_, v_object,
                                        object_context_);

  // HARDCODED for trifinger
  vector<RigidTransform<double>> end_effector_poses;
  for (int i = 0; i < 3; i++) {
    auto end_effector_pose = trifinger_plant_.EvalBodyPoseInWorld(
      *trifinger_context_, trifinger_plant_.GetBodyByName(end_effector_names_[i]));
    end_effector_poses.push_back(end_effector_pose);
  }

  const Eigen::VectorXd& q = object_plant_.GetPositions(*object_context_);

  auto object_pose = object_plant_.EvalBodyPoseInWorld(
            *object_context_, object_plant_.GetBodyByName(object_name_));

  vector<SpatialVelocity<double>> end_effector_spatial_velocities;
  for (int i = 0; i < 3; i++) {
    auto end_effector_spatial_velocity =
        trifinger_plant_.EvalBodySpatialVelocityInWorld(
            *trifinger_context_, trifinger_plant_.GetBodyByName(end_effector_names_[i]));
    end_effector_spatial_velocities.push_back(end_effector_spatial_velocity);
  }

  VectorXd end_effector_positions = VectorXd::Zero(num_end_effector_positions_);
  VectorXd end_effector_velocities =
      VectorXd::Zero(num_end_effector_velocities_);

  for (int i = 0; i < 3; i++) {
    end_effector_positions.segment(3*i, 3) = end_effector_poses.at(i).translation();
  }
 
  for (int i = 0; i < 3; i++) {
    end_effector_velocities.segment(3*i, 3) = end_effector_spatial_velocities.at(i).translational();
  }
  
  VectorXd lcs_state_vector(VectorXd::Zero(num_end_effector_positions_ 
      + num_object_positions_ + num_end_effector_velocities_ + num_object_velocities_));
  lcs_state_vector << end_effector_positions, q_object, end_effector_velocities, v_object;

  lcs_state->get_mutable_data() = lcs_state_vector;
  lcs_state->set_timestamp(trifinger_output->get_timestamp());

}

}  // namespace dairlib
