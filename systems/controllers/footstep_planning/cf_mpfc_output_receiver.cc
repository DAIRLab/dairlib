#include "cf_mpfc_output_receiver.h"

#include "systems/framework/output_vector.h"
#include "common/eigen_utils.h"
#include "multibody/multibody_utils.h"

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
    const std::vector<int>& fsm_states,
    std::vector<alip_utils::PointOnFramed> contact_points,
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context) :
    left_contact_names_(ordered_left_contact_names),
    right_contact_names_(ordered_right_contact_names),
    plant_(plant),
    context_(context) {

  DRAKE_DEMAND(fsm_states.size() == contact_points.size());

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

  lcm_traj_cache_ = DeclareCacheEntry(
      "lcm_traj_cache", &CFMPFCOutputReceiver::CalcLcmTraj,
      {input_port_ticket(input_port_mpfc_output_)}).cache_index();

  output_port_orientation_ = DeclareAbstractOutputPort(
      "wbo_trajectory", model_slerp,
      &CFMPFCOutputReceiver::CopyOrientationTraj,
      {cache_entry_ticket(lcm_traj_cache_)}).get_index();

  wbo_trajectory_cache_ = DeclareCacheEntry(
      "wbo_traj_cache", &CFMPFCOutputReceiver::CalcOrientationTraj,
      {cache_entry_ticket(lcm_traj_cache_)}).cache_index();

  force_traj_cache_ = DeclareCacheEntry(
      "force_traj_cache", &CFMPFCOutputReceiver::CalcForceTraj,
      {cache_entry_ticket(lcm_traj_cache_)}).cache_index();

  PiecewisePolynomial<double> pp;
  Trajectory<double> &model_pp = pp;
  output_port_com_ = DeclareAbstractOutputPort(
      "com_traj", model_pp, &CFMPFCOutputReceiver::CopyComTraj).get_index();

  com_trajectory_cache_ = DeclareCacheEntry(
      "com_traj_cache", &CFMPFCOutputReceiver::CalcComTraj).cache_index();

  for (int i = 0; i < fsm_states.size(); ++i) {
    DRAKE_DEMAND(not fsm_to_stance_foot_map_.contains(fsm_states.at(i)));
    fsm_to_stance_foot_map_.insert({fsm_states.at(i), contact_points.at(i)});
  }
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

  auto fsm = get_mpfc_output(context).fsm;
  int fsm_state = fsm.fsm_state;
  f->SetZero();

  if (mode_match(contact_name, fsm_state)) {

    double t = dynamic_cast<const OutputVector<double>*>(
        EvalVectorInput(context, input_port_state_))->get_timestamp();

    const auto& contact_names = contacts_this_mode(fsm_state);
    int f_idx = std::find(contact_names.begin(), contact_names.end(), contact_name) - contact_names.begin();
    const auto& f_traj = get_cache_entry(force_traj_cache_).Eval<PiecewisePolynomial<double>>(context);
    f->get_mutable_value() = f_traj.value(t).block<3, 1>(3 * f_idx, 0);
  }
}

void CFMPFCOutputReceiver::CalcLcmTraj(
    const Context<double> &context, LcmTrajectory *lcm_traj) const {
  const auto &mpfc_output = get_mpfc_output(context);
  *lcm_traj = LcmTrajectory(mpfc_output.srb_trajs);
}

void CFMPFCOutputReceiver::CalcForceTraj(
    const drake::systems::Context<double> &context,
    drake::trajectories::PiecewisePolynomial<double> *traj) const {
  const auto& lcm_traj = get_cache_entry(lcm_traj_cache_).Eval<LcmTrajectory>(context);
  const auto& force_traj = lcm_traj.GetTrajectory("force_traj");
  *traj = PiecewisePolynomial<double>::FirstOrderHold(
      force_traj.time_vector, force_traj.datapoints);
}

void CFMPFCOutputReceiver::CalcComTraj(
    const Context<double> &context, PiecewisePolynomial<double> *traj) const {
  const auto& lcm_traj = get_cache_entry(lcm_traj_cache_).Eval<LcmTrajectory>(context);
  const auto& com_traj = lcm_traj.GetTrajectory("com_traj");
  const auto& com_traj_dot = lcm_traj.GetTrajectory("com_traj_dot");

  int fsm = get_mpfc_output(context).fsm.fsm_state;
  Vector3d p;

  VectorXd q = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, input_port_state_))->GetPositions();

  multibody::SetPositionsIfNew<double>(plant_, q, context_);
  const auto& stance = fsm_to_stance_foot_map_.at(fsm);
  plant_.CalcPointsPositions(
      *context_, stance.second, stance.first, plant_.world_frame(), &p);
  p = multibody::ReExpressWorldVector3InBodyYawFrame(
      plant_, *context_, floating_base_, p);

  Eigen::MatrixXd foot_pos(
      com_traj.datapoints.rows(), com_traj.datapoints.cols());

  for (int i = 0; i < com_traj.datapoints.cols(); ++i) {
    foot_pos.col(i) = p;
  }

  *traj = PiecewisePolynomial<double>::CubicHermite(
      com_traj.time_vector,
      com_traj.datapoints + foot_pos,
      com_traj_dot.datapoints);
}

void CFMPFCOutputReceiver::CalcOrientationTraj(
    const Context<double> &context,PiecewiseQuaternionSlerp<double> *traj) const {
  const auto& lcm_traj = get_cache_entry(lcm_traj_cache_).Eval<LcmTrajectory>(context);
  const auto& acom_traj = lcm_traj.GetTrajectory("acom_traj");

  std::vector<drake::Quaternion<double>> quats(acom_traj.datapoints.cols());
  for (int i = 0; i < acom_traj.datapoints.cols(); ++i) {
    Eigen::Vector4d q = acom_traj.datapoints.col(i);
    quats.at(i) = drake::Quaternion<double>(q);
  }
  *traj = PiecewiseQuaternionSlerp<double>(
      CopyVectorXdToStdVector(acom_traj.time_vector), quats);
}

}