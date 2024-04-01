#include "examples/franka_ball_rolling/systems/franka_kinematics.h"

#include "common/find_resource.h"
#include "multibody/multibody_utils.h"

namespace dairlib {

using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using Eigen::Vector3d;
using Eigen::VectorXd;
using systems::OutputVector;
using systems::StateVector;
using systems::TimestampedVector;

namespace systems {

FrankaKinematics::FrankaKinematics(const drake::multibody::MultibodyPlant<double> &franka_plant,
                                   drake::systems::Context<double> *franka_context,
                                   const drake::multibody::MultibodyPlant<double> &object_plant,
                                   drake::systems::Context<double> *object_context,
                                   const std::string &end_effector_name,
                                   const std::string &object_name,
                                   bool include_end_effector_orientation,
                                   const SimulateFrankaParams &sim_param,
                                   bool project_state_option)
    : franka_plant_(franka_plant),
      franka_context_(franka_context),
      object_plant_(object_plant),
      object_context_(object_context),
      world_(franka_plant_.world_frame()),
      end_effector_name_(end_effector_name),
      object_name_(object_name),
      include_end_effector_orientation_(include_end_effector_orientation),
      project_state_option_(project_state_option){
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
  num_object_positions_ = 7;
  num_end_effector_velocities_ = 3 + include_end_effector_orientation_ * 3;
  num_object_velocities_ = 6;
  lcs_state_port_ =
      this->DeclareVectorOutputPort(
              "x_lcs",
              FrankaKinematicsVector<double>(
                  num_end_effector_positions_, num_object_positions_,
                  num_end_effector_velocities_, num_object_velocities_),
              &FrankaKinematics::ComputeLCSState)
          .get_index();

  end_effector_radius_ = sim_param.ee_radius;
  object_radius_ = sim_param.ball_radius;
}

void FrankaKinematics::ComputeLCSState(
    const drake::systems::Context<double>& context,
    FrankaKinematicsVector<double>* lcs_state) const {
  const OutputVector<double>* franka_output =
      (OutputVector<double>*)this->EvalVectorInput(context, franka_state_port_);
  const StateVector<double>* object_output =
      (StateVector<double>*)this->EvalVectorInput(context, object_state_port_);

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
  auto object_pose = object_plant_.EvalBodyPoseInWorld(
      *object_context_, object_plant_.GetBodyByName(object_name_));
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

  VectorXd object_position = q_object;
  VectorXd object_xyz = object_pose.translation();
  if (project_state_option_) {
      object_xyz = ProjectStateEstimate(end_effector_pose.translation(),
                                         object_pose.translation());
  }

  object_position << q_object.head(4), object_xyz;

  lcs_state->SetEndEffectorPositions(end_effector_positions);
  lcs_state->SetObjectPositions(object_position);
  lcs_state->SetEndEffectorVelocities(end_effector_velocities);
  lcs_state->SetObjectVelocities(v_object);
  lcs_state->set_timestamp(franka_output->get_timestamp());
}

Eigen::VectorXd FrankaKinematics::ProjectStateEstimate(
        const Eigen::VectorXd &end_effector_position,
        const Eigen::VectorXd &object_position) const {

  VectorXd dist_vec = object_position - end_effector_position;
  double R = object_radius_;
  double r = end_effector_radius_;

  if (dist_vec.norm() < (R+r)*(1)){
    Vector3d u(dist_vec(0), dist_vec(1), 0);
    double u_norm = u.norm();
    double du = sqrt((R+r)*(R+r) - dist_vec(2)*dist_vec(2)) - u_norm;

    return object_position + du * u / u_norm;
  }
  else {
    return object_position;
  }
}

}  // namespace systems
}  // namespace dairlib
