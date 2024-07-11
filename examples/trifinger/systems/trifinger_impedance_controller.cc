#include "trifinger_impedance_controller.h"

#include <iostream>

namespace dairlib::systems {
TrifingerImpedanceControl::TrifingerImpedanceControl(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    const std::string& fingertip_0_name, const std::string& fingertip_120_name,
    const std::string& fingertip_240_name,
    const Eigen::Matrix3d& Kp_fingertip_0,
    const Eigen::Matrix3d& Kd_fingertip_0,
    const Eigen::Matrix3d& Kp_fingertip_120,
    const Eigen::Matrix3d& Kd_fingertip_120,
    const Eigen::Matrix3d& Kp_fingertip_240,
    const Eigen::Matrix3d& Kd_fingertip_240)
    : plant_(plant),
      context_(context),
      fingertip_0_name_(fingertip_0_name),
      fingertip_120_name_(fingertip_120_name),
      fingertip_240_name_(fingertip_240_name),
      Kp_fingertip_0_(Kp_fingertip_0),
      Kd_fingertip_0_(Kd_fingertip_0),
      Kp_fingertip_120_(Kp_fingertip_120),
      Kd_fingertip_120_(Kd_fingertip_120),
      Kp_fingertip_240_(Kp_fingertip_240),
      Kd_fingertip_240_(Kd_fingertip_240) {
  state_port_ =
      this->DeclareVectorInputPort("trifinger_state",
                                   OutputVector<double>(plant_.num_positions(),
                                                        plant_.num_velocities(),
                                                        plant_.num_actuators()))
          .get_index();
  fingertips_target_port_ =
      this->DeclareVectorInputPort("fingertips_target",
                                   TimestampedVector<double>(9))
          .get_index();
  commanded_torque_port_ =
      this->DeclareVectorOutputPort(
              "commanded_torque",
              TimestampedVector<double>(plant_.num_actuators()),
              &TrifingerImpedanceControl::UpdateCommandedTorques)
          .get_index();

  impedance_debug_output_port_ =
      this->DeclareAbstractOutputPort(
              "impedance_debug_output",
              &TrifingerImpedanceControl::CopyTrifingerImpedanceDebugOutput)
          .get_index();

  // retrieve bodies and body frames of 3 fingertips and world frame.
  fingertip_0_body_ = &plant_.GetBodyByName(fingertip_0_name);
  fingertip_120_body_ = &plant_.GetBodyByName(fingertip_120_name);
  fingertip_240_body_ = &plant_.GetBodyByName(fingertip_240_name);
  fingertip_0_frame_ = &fingertip_0_body_->body_frame();
  fingertip_120_frame_ = &fingertip_120_body_->body_frame();
  fingertip_240_frame_ = &fingertip_240_body_->body_frame();
  world_frame_ = &plant_.world_frame();
}

void TrifingerImpedanceControl::UpdateCommandedTorques(
    const drake::systems::Context<double>& context,
    TimestampedVector<double>* output) const {
  const OutputVector<double>* trifinger_state =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  const TimestampedVector<double>* all_fingertips_target =
      (TimestampedVector<double>*)this->EvalVectorInput(
          context, fingertips_target_port_);

  Eigen::VectorXd trifinger_positions = trifinger_state->GetPositions();
  Eigen::VectorXd trifinger_velocities = trifinger_state->GetVelocities();
  dairlib::multibody::SetPositionsIfNew<double>(plant_, trifinger_positions,
                                                context_);
  dairlib::multibody::SetVelocitiesIfNew<double>(plant_, trifinger_velocities,
                                                 context_);

  Eigen::MatrixXd trifinger_mass_matrix(plant_.num_velocities(),
                                        plant_.num_velocities());
  plant_.CalcMassMatrix(*context_, &trifinger_mass_matrix);

  Eigen::VectorXd bias(plant_.num_velocities());
  Eigen::VectorXd grav = plant_.CalcGravityGeneralizedForces(*context_);
  plant_.CalcBiasTerm(*context_, &bias);

  this->CalcCommandedTorqueForFinger(finger_0_outputs_, trifinger_velocities,
                                     all_fingertips_target, fingertip_0_body_,
                                     fingertip_0_frame_, trifinger_mass_matrix,
                                     Kp_fingertip_0_, Kd_fingertip_0_, 0);

  this->CalcCommandedTorqueForFinger(
      finger_120_outputs_, trifinger_velocities, all_fingertips_target,
      fingertip_120_body_, fingertip_120_frame_, trifinger_mass_matrix,
      Kp_fingertip_120_, Kd_fingertip_120_, 3);

  this->CalcCommandedTorqueForFinger(
      finger_240_outputs_, trifinger_velocities, all_fingertips_target,
      fingertip_240_body_, fingertip_240_frame_, trifinger_mass_matrix,
      Kp_fingertip_240_, Kd_fingertip_240_, 6);

  Eigen::VectorXd cur_commanded_torque(9);
  cur_commanded_torque << finger_0_outputs_.commanded_torque,
      finger_120_outputs_.commanded_torque,
      finger_240_outputs_.commanded_torque;

  // compensate for gravity
  cur_commanded_torque += -grav;

  output->SetDataVector(cur_commanded_torque);
  output->set_timestamp(trifinger_state->get_timestamp());
}

void TrifingerImpedanceControl::CalcCommandedTorqueForFinger(
    SingleFingerImpedanceControllerOutputs& outputs,
    const Eigen::VectorXd& cur_trifinger_velocities,
    const TimestampedVector<double>* all_fingertips_target,
    const drake::multibody::RigidBody<double>* fingertip_body,
    const drake::multibody::RigidBodyFrame<double>* fingertip_frame,
    const Eigen::MatrixXd& trifinger_mass_matrix, const Eigen::Matrix3d& Kp,
    const Eigen::Matrix3d& Kd, int finger_index) const {
  Eigen::Matrix3Xd J_full(3, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      *context_, drake::multibody::JacobianWrtVariable::kV, *fingertip_frame,
      Eigen::Vector3d::Zero(), *world_frame_, *world_frame_, &J_full);
  auto J_fingertip = J_full.block(0, finger_index, 3, 3);

  // compute \delta x = x_{des} - x
  auto cur_fingertip_pos =
      plant_.EvalBodyPoseInWorld(*context_, *fingertip_body).translation();
  auto fingertip_target_pos =
      all_fingertips_target->get_value().segment(finger_index, 3);
  auto fingertip_delta_pos = fingertip_target_pos - cur_fingertip_pos;

  // compute \delta v = v_{des} - v
  auto fingertip_delta_vel =
      -J_fingertip * cur_trifinger_velocities.segment(finger_index, 3);

  // compute effective inertia (or mass matrix in the end-effector space).
  Eigen::Matrix3d M_finger_inv =
      trifinger_mass_matrix.block(finger_index, finger_index, 3, 3).inverse();
  Eigen::Matrix3d M_task_space_fingertip =
      (J_fingertip * M_finger_inv * J_fingertip.transpose()).inverse();

  Eigen::Vector3d commanded_torque =
      J_fingertip.transpose() * M_task_space_fingertip *
      (Kp * fingertip_delta_pos + Kd * fingertip_delta_vel);

  // TODO: can we use std::move to avoid copying?
  outputs.commanded_torque = commanded_torque;
  outputs.cur_fingertip_pos = cur_fingertip_pos;
  outputs.fingertip_target_pos = fingertip_target_pos;
  outputs.fingertip_delta_pos = fingertip_delta_pos;
  outputs.fingertip_delta_vel = fingertip_delta_vel;
}

void TrifingerImpedanceControl::CopyTrifingerImpedanceDebugOutput(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_trifinger_impedance_debug* output) const {
  this->CopySingleFingerImpedanceDebugOutput(finger_0_outputs_,
                                            output->finger_0);
  this->CopySingleFingerImpedanceDebugOutput(finger_120_outputs_,
                                        output->finger_120);
  this->CopySingleFingerImpedanceDebugOutput(finger_240_outputs_,
                                        output->finger_240);
}

void TrifingerImpedanceControl::CopySingleFingerImpedanceDebugOutput(
    const SingleFingerImpedanceControllerOutputs& outputs,
    dairlib::lcmt_single_finger_impedance_debug& output) const {
  output.y_dim = 3;
  output.fingertip_y = CopyVectorXdToStdVector(outputs.cur_fingertip_pos);
  output.fingertip_y_des =
      CopyVectorXdToStdVector(outputs.fingertip_target_pos);
  output.fingertip_error_y =
      CopyVectorXdToStdVector(outputs.fingertip_delta_pos);
  output.fingertip_error_ydot =
      CopyVectorXdToStdVector(outputs.fingertip_delta_vel);
}
}  // namespace dairlib::systems
