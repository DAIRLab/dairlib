#include "cf_mpfc_output_receiver.h"

#include "systems/framework/output_vector.h"

namespace dairlib::systems::controllers {

using drake::systems::Context;
using drake::systems::BasicVector;
using drake::trajectories::Trajectory;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::PiecewiseQuaternionSlerp;

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector2d;


CFMPFCOutputReceiver::CFMPFCOutputReceiver(
    std::vector<std::string> ordered_left_contact_names,
    std::vector<std::string> ordered_right_contact_names,
    const drake::multibody::MultibodyPlant<double> &plant) :
    left_contact_names_(ordered_left_contact_names),
    right_contact_names_(ordered_right_contact_names) {

  input_port_state_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(plant)).get_index();
  input_port_mpfc_output_ = DeclareAbstractInputPort(
      "lcmt_cf_mpfc_output", drake::Value<lcmt_cf_mpfc_output>()).get_index();

  for (const std::string &name : ordered_left_contact_names) {
    RegisterContact(name);
  }
  for (const std::string &name : ordered_right_contact_names) {
    RegisterContact(name);
  }

  output_port_fsm_ = DeclareAbstractOutputPort(
      "fsm_info", &CFMPFCOutputReceiver::CopyFsm).get_index();

  output_port_pitch_ = DeclareVectorOutputPort(
      "toe_pitch", 1, &CFMPFCOutputReceiver::CopyPitch).get_index();

  output_port_footstep_target_ = DeclareVectorOutputPort(
      "footstep_target", 3,
      &CFMPFCOutputReceiver::CopyFootstepTarget).get_index();

  PiecewiseQuaternionSlerp<double> slerp;
  Trajectory<double> &model_slerp = slerp;
  output_port_orientation_ = DeclareAbstractOutputPort(
      "wbo_trajectory", model_slerp,
      &CFMPFCOutputReceiver::CopyOrientationTraj).get_index();
  wbo_trajectory_cache_ = DeclareCacheEntry(
      "wbo_traj_cache", &CFMPFCOutputReceiver::CalcOrientationTraj).cache_index();

  PiecewisePolynomial<double> pp;
  Trajectory<double> &model_pp = pp;
  output_port_com_ = DeclareAbstractOutputPort(
      "com_traj", model_pp, &CFMPFCOutputReceiver::CopyComTraj).get_index();

  com_trajectory_cache_ = DeclareCacheEntry(
      "com_traj_cache", &CFMPFCOutputReceiver::CalcComTraj).cache_index();

}

void CFMPFCOutputReceiver::RegisterContact(const std::string &name) {
  DRAKE_DEMAND(not desired_force_output_port_map_.contains(name));
  desired_force_output_port_map_[name] = DeclareVectorOutputPort(
      "lambda_c_des_" + name, 3,
      [this, name](const Context<double> &context, BasicVector<double> *f) {
        this->CalcDesiredForce(name, context, f);
      }
  ).get_index();
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

void CFMPFCOutputReceiver::CopyComTraj(
    const Context<double> &context, Trajectory<double> *traj) const {
  auto pp_traj = dynamic_cast<PiecewisePolynomial<double>*>(traj);
  *pp_traj = get_cache_entry(
      com_trajectory_cache_).Eval<PiecewisePolynomial<double>>(context);
}

void CFMPFCOutputReceiver::CopyOrientationTraj(
    const Context<double> &context, Trajectory<double> *traj) const {
  auto slerp_traj = dynamic_cast<PiecewiseQuaternionSlerp<double>*>(traj);
  *slerp_traj = get_cache_entry(
      wbo_trajectory_cache_).Eval<PiecewiseQuaternionSlerp<double>>(context);
}

void CFMPFCOutputReceiver::CalcDesiredForce(
    const std::string &contact_name, const Context<double> &context,
    BasicVector<double> *f) const {
  // TODO (@Brian-Acosta) figure out the right way to do this
  f->get_mutable_value().setZero();
}

}