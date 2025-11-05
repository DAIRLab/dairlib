#include "cartesian_pose_trajectory_generator.h"

#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/piecewise_quaternion.h>
#include <drake/math/rigid_transform.h>

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/text_logging.h"

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::string;

using drake::math::RigidTransform;
using drake::math::RollPitchYaw;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::PiecewiseQuaternionSlerp;
using drake::trajectories::Trajectory;

using Eigen::VectorXd;

namespace dairlib {
using multibody::SetPositionsIfNew;
using multibody::SetVelocitiesIfNew;
using systems::OutputVector;
namespace examples {
namespace magna {
namespace systems {

CartesianPoseTrajectoryGenerator::CartesianPoseTrajectoryGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* plant_context,
    std::string end_effector_name, bool trajectory_passthrough)
    : plant_(plant),
      plant_context_(plant_context),
      end_effector_name_(end_effector_name),
      is_passthrough_(trajectory_passthrough) {
  // Input/Output Setup
  // Get current state of the robot
  state_port_ = this->DeclareVectorInputPort(
                        "franka_x", OutputVector<double>(plant.num_positions(),
                                                         plant.num_velocities(),
                                                         plant.num_actuators()))
                    .get_index();

  if (is_passthrough_) {
    drake::log()->info(
        "Using trajectory inputs for CartesianPoseTrajectoryGenerator");
    // If using trajectory inputs, the input port is a trajectory

    auto empty_translation_trajectory =
        drake::trajectories::PiecewisePolynomial<double>();
    target_cartesian_translation_trajectory_port_ =
        this->DeclareAbstractInputPort(
                "franka_target_cartesian_translation_trajectory",
                drake::Value<drake::trajectories::Trajectory<double>>(
                    empty_translation_trajectory))
            .get_index();
    auto empty_rotation_trajectory =
        drake::trajectories::PiecewiseQuaternionSlerp<double>();
    target_cartesian_rotation_trajectory_port_ =
        this->DeclareAbstractInputPort(
                "franka_target_cartesian_rotation_trajectory",
                drake::Value<drake::trajectories::Trajectory<double>>(
                    empty_rotation_trajectory))
            .get_index();
  } else {
    // Update current and target pose in discrete forced update events
    // Get target cartesian pose
    target_cartesian_pose_port_ =
        this->DeclareVectorInputPort("franka_target_cartesian_pose",
                                     BasicVector<double>(6))
            .get_index();
  }
  target_cartesian_pose_index_ = this->DeclareDiscreteState(VectorXd::Zero(7));

  // Get current cartesian pose and time
  current_joint_position_index_ =
      this->DeclareDiscreteState(VectorXd::Zero(plant.num_positions()));
  current_cartesian_pose_index_ = this->DeclareDiscreteState(VectorXd::Zero(7));
  current_time_index_ = this->DeclareDiscreteState(VectorXd::Zero(1));

  DeclareForcedDiscreteUpdateEvent(
      &CartesianPoseTrajectoryGenerator::DiscreteVariableUpdate);

  PiecewisePolynomial<double> pp(VectorXd(0));
  Trajectory<double>& default_instantiation = pp;
  translation_trajectory_port_ =
      this->DeclareAbstractOutputPort(
              "translation_trajectory", default_instantiation,
              &CartesianPoseTrajectoryGenerator::CalcTranslationTrajectory)
          .get_index();
  PiecewiseQuaternionSlerp<double> pqs;
  Trajectory<double>& default_quat_instantiation = pqs;
  rotation_trajectory_port_ =
      this->DeclareAbstractOutputPort(
              "rotation_trajectory", default_quat_instantiation,
              &CartesianPoseTrajectoryGenerator::CalcRotationTrajectory)
          .get_index();
  PiecewisePolynomial<double> ppj(VectorXd(0));
  Trajectory<double>& default_joint_instantiation = ppj;
  joint_trajectory_port_ =
      this->DeclareAbstractOutputPort(
              "joint_trajectory", default_joint_instantiation,
              &CartesianPoseTrajectoryGenerator::CalcJointTrajectory)
          .get_index();
}

drake::systems::EventStatus
CartesianPoseTrajectoryGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  auto target_pose =
      discrete_state->get_mutable_value(target_cartesian_pose_index_);

  auto current_time = discrete_state->get_mutable_value(current_time_index_);
  auto current_pose =
      discrete_state->get_mutable_value(current_cartesian_pose_index_);
  auto current_joint_positions =
      discrete_state->get_mutable_value(current_joint_position_index_);

  const OutputVector<double>* current_state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

  current_joint_positions = current_state->GetPositions();
  SetPositionsIfNew<double>(plant_, current_state->GetPositions(),
                            plant_context_);
  SetVelocitiesIfNew<double>(plant_, current_state->GetVelocities(),
                             plant_context_);
  auto ee_pose =
      plant_.CalcRelativeTransform(*plant_context_, plant_.world_frame(),
                                   plant_.GetFrameByName("end_effector_frame"));

  if (current_time[0] == 0.0) {  // first iteration, initialize
    auto current_rotation = ee_pose.rotation().ToQuaternion();
    current_pose << ee_pose.translation(), current_rotation.x(),
        current_rotation.y(), current_rotation.z(), current_rotation.w();
    target_pose = current_pose;
    current_time[0] = context.get_time();
  }

  // If in passthrough mode, set target to current
  if (is_passthrough_) {
    auto current_rotation = ee_pose.rotation().ToQuaternion();
    current_pose << ee_pose.translation(), current_rotation.x(),
        current_rotation.y(), current_rotation.z(), current_rotation.w();
    target_pose = current_pose;
    current_time[0] = context.get_time();
    return drake::systems::EventStatus::Succeeded();
  }

  // Check if target has changed, if so, update and reset the current positions
  // and time
  const auto& target_cartesian_pose =
      this->EvalVectorInput(context, target_cartesian_pose_port_);
  VectorXd target = target_cartesian_pose->get_value();

  if (!target.isZero() && (target - target_pose).norm() > 1e-6) {
    // Update current pose
    auto current_rotation = ee_pose.rotation().ToQuaternion();
    current_pose << ee_pose.translation(), current_rotation.x(),
        current_rotation.y(), current_rotation.z(), current_rotation.w();
    auto target_rotation = RollPitchYaw<double>(target.tail<3>());
    // Create target pose
    auto target_quat = target_rotation.ToQuaternion();
    target_pose << target.head<3>(), target_quat.x(), target_quat.y(),
        target_quat.z(), target_quat.w();
    current_time[0] = context.get_time();
  }
  return drake::systems::EventStatus::Succeeded();
}

void CartesianPoseTrajectoryGenerator::CalcTranslationTrajectory(
    const drake::systems::Context<double>& context,
    Trajectory<double>* output) const {
  // Prepare casted trajectory to hold the output
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          output);

  // Get current and target cartesian pose
  auto current_cartesian_pose =
      context.get_discrete_state(current_cartesian_pose_index_).value();
  auto current_time = context.get_discrete_state(current_time_index_).value();

  if (is_passthrough_) {
    const auto& trajectory_input =
        this->EvalAbstractInput(context,
                                target_cartesian_translation_trajectory_port_)
            ->get_value<drake::trajectories::Trajectory<double>>();
    *casted_traj = *(PiecewisePolynomial<double>*)dynamic_cast<
        const PiecewisePolynomial<double>*>(&trajectory_input);
    if (casted_traj->get_number_of_segments() == 0) {
      MatrixXd current_translation(3, 1);
      current_translation << current_cartesian_pose.head<3>();
      *casted_traj = PiecewisePolynomial<double>(current_translation);
    }
    return;
  }

  auto target_cartesian_pose =
      context.get_discrete_state(target_cartesian_pose_index_).value();

  // If time < 1.0, maintain current pose
  if (context.get_time() < 1.0) {
    target_cartesian_pose = current_cartesian_pose;
  }

  // Create a cubic trajectory from current to target pose
  std::vector<double> breaks = {current_time[0], current_time[0] + 1.0};
  std::vector<MatrixXd> samples(2);
  samples[0] = MatrixXd::Zero(3, 1);
  samples[0] << current_cartesian_pose.head<3>();
  samples[1] = MatrixXd::Zero(3, 1);
  samples[1] << target_cartesian_pose.head<3>();
  *casted_traj =
      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
          breaks, samples, MatrixXd::Zero(3, 1), MatrixXd::Zero(3, 1));
}

void CartesianPoseTrajectoryGenerator::CalcRotationTrajectory(
    const drake::systems::Context<double>& context,
    Trajectory<double>* output) const {
  // Prepare casted trajectory to hold the output
  auto* casted_traj = (PiecewiseQuaternionSlerp<double>*)dynamic_cast<
      PiecewiseQuaternionSlerp<double>*>(output);

  // Get current and target cartesian pose
  auto current_cartesian_pose =
      context.get_discrete_state(current_cartesian_pose_index_).value();
  auto current_time = context.get_discrete_state(current_time_index_).value();

  auto target_cartesian_pose =
      context.get_discrete_state(target_cartesian_pose_index_).value();

  // If time < 1.0, maintain current pose
  if (context.get_time() < 1.0) {
    target_cartesian_pose = current_cartesian_pose;
  }

  if (is_passthrough_) {
    const auto& trajectory_input =
        this->EvalAbstractInput(context,
                                target_cartesian_rotation_trajectory_port_)
            ->get_value<drake::trajectories::Trajectory<double>>();
    *casted_traj = *(PiecewiseQuaternionSlerp<double>*)dynamic_cast<
        const PiecewiseQuaternionSlerp<double>*>(&trajectory_input);
    if (casted_traj->get_number_of_segments() != 0)
      return;
    else  // If no segments, create a constant trajectory at current orientation
      target_cartesian_pose = current_cartesian_pose;
  }

  // Create a cubic trajectory from current to target pose
  std::vector<double> breaks = {current_time[0], current_time[0] + 1.0};
  std::vector<Eigen::Quaternion<double>> samples(2);
  samples[0] = Eigen::Quaternion<double>(current_cartesian_pose.tail<4>());
  samples[1] = Eigen::Quaternion<double>(target_cartesian_pose.tail<4>());
  *casted_traj = PiecewiseQuaternionSlerp<double>(breaks, samples);
}

void CartesianPoseTrajectoryGenerator::CalcJointTrajectory(
    const drake::systems::Context<double>& context,
    Trajectory<double>* output) const {
  // Maintain joint positions as much as possible while moving end-effector
  // Prepare casted trajectory to hold the output
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          output);

  // Get current and target joint positions
  auto current_joint_positions =
      context.get_discrete_state(current_joint_position_index_).value();
  auto current_time = context.get_discrete_state(current_time_index_).value();

  auto target_joint_positions = current_joint_positions;

  // Create a cubic trajectory from current to target joint positions
  MatrixXd constant_joint_position(plant_.num_positions(), 1);
  constant_joint_position << current_joint_positions;
  *casted_traj = PiecewisePolynomial<double>(constant_joint_position);
}
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
