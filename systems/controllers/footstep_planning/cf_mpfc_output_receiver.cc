#include "cf_mpfc_output_receiver.h"

#include "systems/framework/output_vector.h"
#include "common/eigen_utils.h"
#include "common/polynomial_utils.h"
#include "multibody/multibody_utils.h"

#include <iostream>

namespace dairlib::systems::controllers {

using drake::systems::Context;
using drake::systems::BasicVector;
using drake::trajectories::Trajectory;
using drake::trajectories::PiecewisePolynomial;

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector2d;


CFMPFCOutputReceiver::CFMPFCOutputReceiver(
    const drake::multibody::MultibodyPlant<double>& plant) {
  input_port_state_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(plant)).get_index();
  input_port_mpfc_output_ = DeclareAbstractInputPort(
      "lcmt_cf_mpfc_output", drake::Value<lcmt_cf_mpfc_output>()).get_index();

  output_port_fsm_ = DeclareAbstractOutputPort(
      "fsm_info", &CFMPFCOutputReceiver::CopyFsm).get_index();

  output_port_pitch_ = DeclareVectorOutputPort(
      "toe_pitch", 1, &CFMPFCOutputReceiver::CopyPitch).get_index();

  output_port_footstep_target_ = DeclareVectorOutputPort(
      "footstep_target", 3,
      &CFMPFCOutputReceiver::CopyFootstepTarget).get_index();

  PiecewisePolynomial<double> pp;
  drake::trajectories::Trajectory<double>& traj_instance = pp;
  output_port_r_traj_ = DeclareAbstractOutputPort(
      "rtraj", traj_instance,&CFMPFCOutputReceiver::CopyRTraj).get_index();

  output_port_ankle_torque_ = DeclareVectorOutputPort(
      "u", 1, &CFMPFCOutputReceiver::CopyAnkleTorque).get_index();

  traj_cache_ = DeclareCacheEntry(
      "lcm_traj_cache", &CFMPFCOutputReceiver::CalcInputTraj,
      {input_port_ticket(input_port_mpfc_output_)}).cache_index();
}

void CFMPFCOutputReceiver::CopyFsm(
    const Context<double> &context, lcmt_fsm_info *out) const {
  *out = get_mpfc_output(context).fsm;
}

void CFMPFCOutputReceiver::CopyPitch(
    const Context<double> &context, BasicVector<double> *pitch) const {
  const auto &output = get_mpfc_output(context);
  Vector3d p = Vector3d::Map(output.next_footstep_in_stance_frame);
  Eigen::Matrix2d A;
  Vector2d b(p(2), 0.0);
  A << p(0), p(1), -p(1), p(0);
  Vector2d kx_ky = A.inverse() * b;

  // handle degenerate case when p = 0
  if (kx_ky.hasNaN()) { kx_ky = Vector2d::Zero(); }

  pitch->get_mutable_value() << atan(kx_ky(0));
}

void CFMPFCOutputReceiver::CopyFootstepTarget(
    const Context<double> &context,BasicVector<double> *target) const {
  target->get_mutable_value() = Vector3d::Map(
      get_mpfc_output(context).next_footstep_in_stance_frame);
}

void CFMPFCOutputReceiver::CalcInputTraj(
    const Context<double> &context,
    drake::trajectories::PiecewisePolynomial<double>* traj) const {
  const auto &mpfc_output = get_mpfc_output(context);
  LcmTrajectory lcm_traj(mpfc_output.trajs);

  *traj = PiecewisePolynomial<double>::FirstOrderHold(
      lcm_traj.GetTrajectory("input_traj").time_vector,
      lcm_traj.GetTrajectory("input_traj").datapoints);
}

void CFMPFCOutputReceiver::CopyAnkleTorque(
    const Context<double> &context, BasicVector<double> *u) const {

  double t = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, input_port_state_))->get_timestamp();

  const auto& input_traj =
      get_cache_entry(traj_cache_).Eval<PiecewisePolynomial<double>>(context);
  u->get_mutable_value()(0) = input_traj.value(t)(0);
}

void CFMPFCOutputReceiver::CopyRTraj(
    const Context<double> &context, Trajectory<double> *out) const {

  double t = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, input_port_state_))->get_timestamp();
  const auto &mpfc_output = get_mpfc_output(context);

  LcmTrajectory lcm_traj(mpfc_output.trajs);
  LcmTrajectory::Trajectory traj = lcm_traj.GetTrajectory("state_traj");

  *dynamic_cast<PiecewisePolynomial<double>*>(out) =
      PiecewisePolynomial<double>::CubicHermite(
          traj.time_vector,
          traj.datapoints.block(2, 0, 1, traj.datapoints.cols()),
          traj.datapoints.block(5, 0, 1, traj.datapoints.cols())
      );
}

}