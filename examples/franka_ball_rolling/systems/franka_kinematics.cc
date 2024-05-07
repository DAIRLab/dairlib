#include "examples/franka_ball_rolling/systems/franka_kinematics.h"

#include "common/find_resource.h"
#include "multibody/multibody_utils.h"

namespace dairlib {

using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using Eigen::Vector3d;
using Eigen::VectorXd;
using systems::OutputVector;
using systems::StateVector;
using systems::TimestampedVector;

namespace systems {

FrankaKinematics::FrankaKinematics(
    const drake::multibody::MultibodyPlant<double>& full_model_plant,
    drake::systems::Context<double>& full_model_context,
    const drake::multibody::ModelInstanceIndex franka_index,
    const drake::multibody::ModelInstanceIndex object_index,
    const std::string& end_effector_name, const std::string& object_name,
    bool include_end_effector_orientation,
    const std::vector<drake::SortedPair<drake::geometry::GeometryId>>
        contact_pairs,
    C3Options c3_options, const SimulateFrankaParams& sim_param,
    bool project_state_option)
    : full_model_plant_(full_model_plant),
      full_model_context_(full_model_context),
      franka_index_(franka_index),
      object_index_(object_index),
      world_(full_model_plant_.world_frame()),
      end_effector_name_(end_effector_name),
      object_name_(object_name),
      include_end_effector_orientation_(include_end_effector_orientation),
      contact_pairs_(contact_pairs),
      c3_options_(std::move(c3_options)),
      project_state_option_(project_state_option) {
  this->set_name("franka_kinematics");
  franka_state_port_ =
      this->DeclareVectorInputPort(
              "x_franka", OutputVector<double>(
                              full_model_plant.num_positions(franka_index),
                              full_model_plant.num_velocities(franka_index),
                              full_model_plant.num_actuators(franka_index)))
          .get_index();

  object_state_port_ =
      this->DeclareVectorInputPort(
              "x_object", StateVector<double>(
                              full_model_plant.num_positions(object_index),
                              full_model_plant.num_velocities(object_index)))
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

  contact_jacobian_port_ =
      this->DeclareAbstractOutputPort(
              "contact_jacobian_full",
              &FrankaKinematics::ComputeFullContactJacobian)
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
  full_model_plant_.SetPositions(&full_model_context_, franka_index_, q_franka);
  full_model_plant_.SetVelocities(&full_model_context_, franka_index_,
                                  v_franka);
  full_model_plant_.SetPositions(&full_model_context_, object_index_, q_object);
  full_model_plant_.SetVelocities(&full_model_context_, object_index_,
                                  v_object);

  auto end_effector_pose = full_model_plant_.EvalBodyPoseInWorld(
      full_model_context_, full_model_plant_.GetBodyByName(end_effector_name_));
  auto object_pose = full_model_plant_.EvalBodyPoseInWorld(
      full_model_context_, full_model_plant_.GetBodyByName(object_name_));
  auto end_effector_spatial_velocity =
      full_model_plant_.EvalBodySpatialVelocityInWorld(
          full_model_context_,
          full_model_plant_.GetBodyByName(end_effector_name_));
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

void FrankaKinematics::ComputeFullContactJacobian(
    const drake::systems::Context<double>& context,
    MatrixXd* contact_jacobian) const {
  const OutputVector<double>* franka_output =
      (OutputVector<double>*)this->EvalVectorInput(context, franka_state_port_);
  const StateVector<double>* object_output =
      (StateVector<double>*)this->EvalVectorInput(context, object_state_port_);

  VectorXd q_franka = franka_output->GetPositions();
  VectorXd v_franka = franka_output->GetVelocities();
  VectorXd q_object = object_output->GetPositions();
  VectorXd v_object = object_output->GetVelocities();

  full_model_plant_.SetPositions(&full_model_context_, franka_index_, q_franka);
  full_model_plant_.SetVelocities(&full_model_context_, franka_index_,
                                  v_franka);
  full_model_plant_.SetPositions(&full_model_context_, object_index_, q_object);
  full_model_plant_.SetVelocities(&full_model_context_, object_index_,
                                  v_object);

  multibody::SetInputsIfNew<double>(
      full_model_plant_, VectorXd::Zero(full_model_plant_.num_actuators()),
      &full_model_context_);

  dairlib::solvers::ContactModel contact_model;
  if (c3_options_.contact_model == "stewart_and_trinkle") {
    contact_model = solvers::ContactModel::kStewartAndTrinkle;
  } else if (c3_options_.contact_model == "anitescu") {
    contact_model = solvers::ContactModel::kAnitescu;
  } else {
    throw std::runtime_error("unknown or unsupported contact model");
  }

  auto jacobian_point_pair = solvers::LCSFactory::ComputeContactJacobian(
      full_model_plant_, full_model_context_, contact_pairs_,
      c3_options_.num_friction_directions, c3_options_.mu, contact_model);

  MatrixXd J_contact_full = jacobian_point_pair.first;
  *contact_jacobian = J_contact_full;
}

Eigen::VectorXd FrankaKinematics::ProjectStateEstimate(
    const Eigen::VectorXd& end_effector_position,
    const Eigen::VectorXd& object_position) const {
  VectorXd dist_vec = object_position - end_effector_position;
  double R = object_radius_;
  double r = end_effector_radius_;

  if (dist_vec.norm() < (R + r) * (1)) {
    Vector3d u(dist_vec(0), dist_vec(1), 0);
    double u_norm = u.norm();
    double du = sqrt((R + r) * (R + r) - dist_vec(2) * dist_vec(2)) - u_norm;

    return object_position + du * u / u_norm;
  } else {
    return object_position;
  }
}

}  // namespace systems
}  // namespace dairlib
