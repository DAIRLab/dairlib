#include "fingertips_target_kinematics_receiver.h"

#include <iostream>
#include <utility>

namespace dairlib::systems {
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

FingertipTargetKinematicsReceiver::FingertipTargetKinematicsReceiver(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    const std::string& fingertip_0_name, const std::string& fingertip_120_name,
    const std::string& fingertip_240_name,
    const unsigned int& target_kinematics_update_frequency)
    : plant_(plant),
      context_(context),
      fingertip_0_name_(fingertip_0_name),
      fingertip_120_name_(fingertip_120_name),
      fingertip_240_name_(fingertip_240_name),
      target_kinematics_update_frequency_(target_kinematics_update_frequency) {
  this->set_name("fingertips_target_kinematics_receiver");
  state_port_ =
      this->DeclareVectorInputPort("x_trifinger",
                                   OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();

  fingertips_target_kinematics_port_ =
      this->DeclareAbstractInputPort(
              "fingertips_target_kinematics",
              drake::Value<dairlib::lcmt_fingertips_target_kinematics>{})
          .get_index();

  PiecewisePolynomial<double> empty_pp_traj(Eigen::MatrixXd::Zero(9, 2));
  Trajectory<double>& traj_inst = empty_pp_traj;
  fingertips_target_traj_port_ =
      this->DeclareAbstractOutputPort("fingertips_target_traj", traj_inst,
                                      &FingertipTargetKinematicsReceiver::
                                          CopyToOutputFingertipsTargetTraj)
          .get_index();

  fingertips_target_kinematics_out_port_ =
      this->DeclareVectorOutputPort("fingertips_target",
                                    TimestampedVector<double>(18),
                                    &FingertipTargetKinematicsReceiver::
                                        CopyToOutputFingertipsTargetKinematics)
          .get_index();

  lcm_cur_fingertips_pos_port_ =
      this->DeclareAbstractOutputPort("lcm_cur_fingertips_pos",
                                      &FingertipTargetKinematicsReceiver::
                                          CopytoLCMCurrentFingertipPositions)
          .get_index();

  lcm_target_fingertips_pos_port_ =
      this->DeclareAbstractOutputPort("lcm_target_fingertips_pos",
                                      &FingertipTargetKinematicsReceiver::
                                          CopytoLCMTargetFingertipPositions)
          .get_index();

  // Declare update event.
  DeclareForcedDiscreteUpdateEvent(
      &FingertipTargetKinematicsReceiver::DiscreteVariableUpdate);

  // Discrete state which stores the desired fingertips position.
  fingertips_target_pos_idx_ = DeclareDiscreteState(Eigen::VectorXd::Zero(9));
  fingertips_target_vel_idx_ = DeclareDiscreteState(Eigen::VectorXd::Zero(9));
  start_fingertips_pos_traj_idx_ =
      DeclareDiscreteState(Eigen::VectorXd::Zero(9));
  start_fingertips_vel_traj_idx_ =
      DeclareDiscreteState(Eigen::VectorXd::Zero(9));
  prev_target_timestamp_idx_ =
      DeclareDiscreteState(Eigen::VectorXd::Ones(1) * (-1));
  start_time_traj_idx_ = DeclareDiscreteState(Eigen::VectorXd::Ones(1) * (-1));
  cur_fingertips_pos_idx_ = DeclareDiscreteState(Eigen::VectorXd::Zero(9));
  cur_fingertips_vel_idx_ = DeclareDiscreteState(Eigen::VectorXd::Zero(9));
}

drake::systems::EventStatus
FingertipTargetKinematicsReceiver::DiscreteVariableUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const OutputVector<double>* trifinger_state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  const auto& fingertips_target_kinematics_lcm_msg =
      this->EvalInputValue<dairlib::lcmt_fingertips_target_kinematics>(
          context, fingertips_target_kinematics_port_);

  // Evaluate the current positions of the fingertips.
  Eigen::VectorXd q_trifinger = trifinger_state->GetPositions();
  multibody::SetPositionsIfNew<double>(plant_, q_trifinger, context_);

  auto fingertip_0_pos =
      plant_
          .EvalBodyPoseInWorld(*context_,
                               plant_.GetBodyByName(fingertip_0_name_))
          .translation();
  auto fingertip_0_vel =
      plant_
          .EvalBodySpatialVelocityInWorld(
              *context_, plant_.GetBodyByName(fingertip_0_name_))
          .translational();
  auto fingertip_120_pos =
      plant_
          .EvalBodyPoseInWorld(*context_,
                               plant_.GetBodyByName(fingertip_120_name_))
          .translation();
  auto fingertip_120_vel =
      plant_
          .EvalBodySpatialVelocityInWorld(
              *context_, plant_.GetBodyByName(fingertip_120_name_))
          .translational();
  auto fingertip_240_pos =
      plant_
          .EvalBodyPoseInWorld(*context_,
                               plant_.GetBodyByName(fingertip_240_name_))
          .translation();
  auto fingertip_240_vel =
      plant_
          .EvalBodySpatialVelocityInWorld(
              *context_, plant_.GetBodyByName(fingertip_240_name_))
          .translational();
  Eigen::VectorXd cur_fingertips_pos(9);
  Eigen::VectorXd cur_fingertips_vel(9);
  cur_fingertips_pos << fingertip_0_pos, fingertip_120_pos, fingertip_240_pos;
  cur_fingertips_vel << fingertip_0_vel, fingertip_120_vel, fingertip_240_vel;
  discrete_state->get_mutable_vector(cur_fingertips_pos_idx_)
      .set_value(cur_fingertips_pos);
  discrete_state->get_mutable_vector(cur_fingertips_vel_idx_)
      .set_value(cur_fingertips_vel);

  Eigen::VectorXd fingertips_target_pos = cur_fingertips_pos;
  auto current_msg_timestamp = fingertips_target_kinematics_lcm_msg->utime;
  auto previous_msg_timestamp = static_cast<int64_t>(
      context.get_discrete_state(prev_target_timestamp_idx_)[0]);

  // check if the obtained message from lcm input port is still old one,
  // if so, no update is performed on the discrete states.
  if (current_msg_timestamp == previous_msg_timestamp) {
    return drake::systems::EventStatus::Succeeded();
  }
  if (!fingertips_target_kinematics_lcm_msg->isAbsoluteTargetPos) {
    fingertips_target_pos += Eigen::VectorXd::Map(
        fingertips_target_kinematics_lcm_msg->targetPos, 9);
  } else if (previous_msg_timestamp != -1) {
    fingertips_target_pos = Eigen::VectorXd::Map(
        fingertips_target_kinematics_lcm_msg->targetPos, 9);
  }
  discrete_state->get_mutable_vector(start_time_traj_idx_)
      .set_value(Eigen::VectorXd::Ones(1) * trifinger_state->get_timestamp());
  discrete_state->get_mutable_vector(start_fingertips_pos_traj_idx_)
      .set_value(cur_fingertips_pos);
  discrete_state->get_mutable_vector(start_fingertips_vel_traj_idx_)
      .set_value(cur_fingertips_vel);
  discrete_state->get_mutable_vector(fingertips_target_pos_idx_)
      .set_value(fingertips_target_pos);
  discrete_state->get_mutable_vector(fingertips_target_vel_idx_)
      .set_value(Eigen::VectorXd::Map(
          fingertips_target_kinematics_lcm_msg->targetVel, 9));
  discrete_state->get_mutable_vector(prev_target_timestamp_idx_)
      .set_value((Eigen::VectorXd::Ones(1) *
                  fingertips_target_kinematics_lcm_msg->utime));
  return drake::systems::EventStatus::Succeeded();
}

void FingertipTargetKinematicsReceiver::CopyToOutputFingertipsTargetTraj(
    const drake::systems::Context<double>& context,
    Trajectory<double>* target_traj) const {
  // retrieve fingertips target positions from the current discrete state.
  auto fingertips_target_pos =
      context.get_discrete_state(fingertips_target_pos_idx_).get_value();
  auto fingertips_target_vel =
      context.get_discrete_state(fingertips_target_vel_idx_).get_value();
  auto start_fingertips_pos_traj =
      context.get_discrete_state(start_fingertips_pos_traj_idx_).get_value();
  auto start_fingertips_vel_traj =
      context.get_discrete_state(start_fingertips_vel_traj_idx_).get_value();

  double target_kinematics_update_period =
      1.0 / target_kinematics_update_frequency_;

  // generate a CubicSpline trajectory that can be fed to OSC controller.
  Eigen::VectorXd knots(2);
  auto start_time_traj =
      context.get_discrete_state(start_time_traj_idx_).get_value()[0];
  knots << start_time_traj, start_time_traj + target_kinematics_update_period;

  Eigen::MatrixXd samples(start_fingertips_pos_traj.size(), 2);
  samples.col(0) = start_fingertips_pos_traj;
  samples.col(1) = fingertips_target_pos;

  Eigen::MatrixXd samples_dot(start_fingertips_vel_traj.size(), 2);
  samples_dot.col(0) = start_fingertips_vel_traj;

  // If target velocities are not set (default value is NaN), we set target
  // velocities as ones from first-order hold interpolation.
  if (!fingertips_target_vel.array().isNaN().any()) {
    samples_dot.col(1) = fingertips_target_vel;
  } else {
    samples_dot.col(1) = (fingertips_target_pos - start_fingertips_pos_traj) /
                         target_kinematics_update_period;
  }

  auto traj =
      PiecewisePolynomial<double>::CubicHermite(knots, samples, samples_dot);
  auto casted_target_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          target_traj);
  *casted_target_traj = traj;
}

void FingertipTargetKinematicsReceiver::CopyToOutputFingertipsTargetKinematics(
    const drake::systems::Context<double>& context,
    TimestampedVector<double>* fingertips_target) const {
  // retrieve fingertips target positions from the current discrete state.
  Eigen::VectorXd fingertips_target_pos_vel;
  fingertips_target_pos_vel
      << context.get_discrete_state(fingertips_target_pos_idx_).get_value(),
      context.get_discrete_state(fingertips_target_vel_idx_).get_value();
  fingertips_target->SetDataVector(fingertips_target_pos_vel);
  fingertips_target->set_timestamp(context.get_time());
}

void FingertipTargetKinematicsReceiver::CopytoLCMCurrentFingertipPositions(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_fingertips_position* lcm_cur_fingertips_pos) const {
  auto cur_fingertips_pos =
      context.get_discrete_state(cur_fingertips_pos_idx_).get_value();
  lcm_cur_fingertips_pos->utime =
      static_cast<int64_t>(context.get_time() * 1e6);

  for (int i = 0; i < cur_fingertips_pos.size(); i++) {
    lcm_cur_fingertips_pos->curPos[i] = cur_fingertips_pos[i];
  }
}

void FingertipTargetKinematicsReceiver::CopytoLCMTargetFingertipPositions(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_fingertips_position* lcm_target_fingertips_pos) const {
  auto target_fingertips_pos =
      context.get_discrete_state(fingertips_target_pos_idx_).get_value();
  lcm_target_fingertips_pos->utime =
      static_cast<int64_t>(context.get_time() * 1e6);

  for (int i = 0; i < target_fingertips_pos.size(); i++) {
    lcm_target_fingertips_pos->curPos[i] = target_fingertips_pos[i];
  }
}
}  // namespace dairlib::systems
