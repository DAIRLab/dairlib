#include "alip_mpc_output_reciever.h"


using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector2d;
using drake::systems::Context;
using drake::systems::BasicVector;

namespace dairlib::systems::controllers {

AlipMpcOutputReceiver::AlipMpcOutputReceiver() {
  alip_mpc_input_port_ = DeclareAbstractInputPort(
      "lcmt_alip_mpc_output", drake::Value<lcmt_alip_mpc_output>()
    ).get_index();
  fsm_output_port_ = DeclareAbstractOutputPort(
      "lcmt_fsm_info", &AlipMpcOutputReceiver::CopyFsm).get_index();
  footstep_target_output_port_ = DeclareVectorOutputPort(
      "footstep_target_in_stance_frame", 3,
      &AlipMpcOutputReceiver::CopyFootstepTarget).get_index();
  slope_parameters_output_port_ = DeclareVectorOutputPort(
      "kx_ky", 2, &AlipMpcOutputReceiver::CopySlopeParameters).get_index();
  pitch_output_port_ = DeclareVectorOutputPort(
      "pitch", 1, &AlipMpcOutputReceiver::CopyPitch).get_index();
  ankle_torque_output_port_ = DeclareAbstractOutputPort(
      "u_alip", &AlipMpcOutputReceiver::CopyAnkleTorque).get_index();
}

void AlipMpcOutputReceiver::CopyFsm(const Context<double> &context,
                                    lcmt_fsm_info *fsm_info) const {
  *fsm_info = EvalAbstractInput(
      context, alip_mpc_input_port_)->get_value<lcmt_alip_mpc_output>().fsm;
}

void AlipMpcOutputReceiver::CopyFootstepTarget(const Context<double> &context,
                                               BasicVector<double> *out) const {
  const auto& output = EvalAbstractInput(
      context, alip_mpc_input_port_)->get_value<lcmt_alip_mpc_output>();
  out->get_mutable_value() = Vector3d::Map(output.next_footstep_in_stance_frame);
}

void AlipMpcOutputReceiver::CopySlopeParameters(const Context<double>& context,
                                                BasicVector<double>* out) const {
  const auto& output = EvalAbstractInput(
      context, alip_mpc_input_port_)->get_value<lcmt_alip_mpc_output>();
  Vector3d p = Vector3d::Map(output.next_footstep_in_stance_frame);
  Eigen::Matrix2d A;
  Vector2d b(p(2), 0.0);
  A << p(0), p(1), -p(1), p(0);
  Vector2d kx_ky = A.inverse() * b;

  // handle degenerate case when p = 0
  if (kx_ky.hasNaN()) {
    kx_ky = Vector2d::Zero();
  }

  out->SetFromVector(kx_ky);
}

void AlipMpcOutputReceiver::CopyPitch(const Context<double> &context,
                                      BasicVector<double> *out) const {
  const auto& output = EvalAbstractInput(
      context, alip_mpc_input_port_)->get_value<lcmt_alip_mpc_output>();
  Vector3d p = Vector3d::Map(output.next_footstep_in_stance_frame);
  Eigen::Matrix2d A;
  Vector2d b(p(2), 0.0);
  A << p(0), p(1), -p(1), p(0);
  Vector2d kx_ky = A.inverse() * b;

  // handle degenerate case when p = 0
  if (kx_ky.hasNaN()) {
    kx_ky = Vector2d::Zero();
  }

  out->get_mutable_value() << atan(kx_ky(0));
}

void AlipMpcOutputReceiver::CopyAnkleTorque(const Context<double> &context,
                                            lcmt_saved_traj *out) const {
  *out =  EvalAbstractInput(
      context, alip_mpc_input_port_)->get_value<lcmt_alip_mpc_output>().u_traj;
}
}
