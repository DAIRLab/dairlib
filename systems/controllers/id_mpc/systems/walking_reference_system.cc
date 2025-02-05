#include <cmath>

#include "walking_reference_system.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using drake::systems::State;
using drake::systems::Context;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;

WalkingReferenceSystem::WalkingReferenceSystem(
    const ConstrainedDynamicsInfo &dynamics,
    Context<double> *plant_context, const GaitParams &params) :
    dynamics_(dynamics),
    plant_(dynamics.get_plant()),
    plant_context_(plant_context), params_(params) {

  input_port_state_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(plant_)).get_index();

  input_port_vdes_ = DeclareVectorInputPort("vdes", 2).get_index();

  ds_intervals_ = std::round(params_.t_ds / params_.mpc_dt);
  ss_intervals_ = std::round(params_.t_ss / params_.mpc_dt);

  DRAKE_DEMAND(ds_intervals_ >= 1);
  DRAKE_DEMAND(ss_intervals_ >= 1);
}

fsm_info WalkingReferenceSystem::CalcFSM(
    double t, const fsm_info &curr_fsm) const {
  fsm_info fsm = curr_fsm;

  if (t >= curr_fsm.next_switch_time) {
    fsm.prev_switch_time = t;
    fsm.next_switch_time =
        curr_fsm.is_double_stance() ? t + params_.t_ss : t + params_.t_ds;
    fsm.state = fsm_info::next_fsm(curr_fsm.state);
  }
  return fsm;
}

EventStatus WalkingReferenceSystem::UnrestrictedUpdate(
    const Context<double> &context, State<double> *state) const {

  const auto &robot_output =
      get_input_port_state().Eval<OutputVector<double>>(context);
  const auto vdes = get_input_port_vdes().Eval(context);

  multibody::SetPositionsAndVelocitiesIfNew<double>(
      plant_, robot_output.GetState(), plant_context_);

  double t = robot_output.get_timestamp();
  const auto &fsm = state->get_abstract_state<fsm_info>(fsm_info_idx_);
  fsm_info next_fsm = CalcFSM(t, fsm);

  auto &mpc_reference =
      state->get_mutable_abstract_state<MPCReference>(reference_state_idx_);

  auto fsm_vector = CalcGaitTiming(t, next_fsm, &mpc_reference);

  mpc_reference.q_traj_ = CalcPositionTraj(fsm.state, vdes, mpc_reference.knot_times_);
  mpc_reference.quat_traj_ = CalcOrientationTraj();
  mpc_reference.v_traj_ = CalcVelocityTraj(vdes);
  mpc_reference.lambda_traj_ = CalcLambdaTraj(mpc_reference.knot_times_, fsm_vector, mpc_reference.active_contacts_);
  mpc_reference.u_traj_ = CalcInputTraj(mpc_reference.knot_times_, fsm_vector);

  state->get_mutable_abstract_state<fsm_info>(fsm_info_idx_) = next_fsm;

  // TODO (@Brian-Acosta) add the swing foot traj

  return EventStatus::Succeeded();
}

namespace {

void ResetContacts(int N, MPCReference *ref) {
  ref->knot_times_ = std::vector<double>(N, 0);
  ref->touchdown_ee_names_ = std::vector<std::string>(N, "");
  ref->touchdown_ee_points_ = std::vector<Vector3d>(N, Vector3d::Zero());
  ref->active_contacts_ = std::vector<std::vector<std::string>>(
      N, std::vector<std::string>{});
}

int AdaptiveNumIntervals(double t_remain_this_mode,
                         const GaitParams& params,
                         bool is_double_stance) {

  int intervals_this_mode = std::ceil(t_remain_this_mode / params.mpc_dt);
  int nominal_intervals = is_double_stance ?
                          std::round(params.t_ds / params.mpc_dt) :
                          std::round(params.t_ss / params.mpc_dt);

  return std::min(intervals_this_mode, nominal_intervals);
}

std::vector<fsm_info::fsm_state> GetFSMStateVector(
    int intervals, fsm_info fsm, const GaitParams& params) {

  int N = params.mpc_N + 1;

  std::vector<fsm_info::fsm_state> states_per_knot;
  for (int i = 0; i < intervals; ++i) {
    states_per_knot.at(i) = fsm.state;
  }
  int switch_knot = intervals;
  fsm_info::fsm_state state = fsm.state;

  for (int i = intervals; i < N; ++i) {
    // Roll over the fsm state if needed
    if (i - switch_knot == 0) {
      state = fsm_info::next_fsm(state);
      switch_knot = fsm_info::is_double_stance(state) ?
                    i + params.t_ds : i + params.t_ss;
    }
    states_per_knot.at(i) = state;
  }
  return states_per_knot;
}

}

std::vector<fsm_info::fsm_state> WalkingReferenceSystem::CalcGaitTiming(
    double t, const fsm_info &fsm, MPCReference *mpc_reference) const {

  // impact timing
  double t_remain_this_mode = fsm.next_switch_time - t;
  int intervals = AdaptiveNumIntervals(t_remain_this_mode, params_, fsm.is_double_stance());
  int N = params_.mpc_N + 1;
  ResetContacts(N, mpc_reference);

  // Set time vector, adapting time to fit the correct number of knots into
  // the current phase
  for (int i = 0; i <= intervals; ++i) {
    mpc_reference->knot_times_.at(i) = t + i * t_remain_this_mode / intervals;
  }
  for (int i = intervals + 1; i < N; ++i) {
    int n = i - intervals;
    double t_now = mpc_reference->knot_times_.at(intervals) + n * params_.mpc_dt;
    mpc_reference->knot_times_.at(i) = t_now;
  }
  auto fsm_vector = GetFSMStateVector(intervals, fsm, params_);

  for (int i = 1; i <= params_.mpc_N; ++i) {
    SetContactsAtKnot(i, fsm_vector.at(i), mpc_reference);
    if (fsm_vector.at(i - 1) != fsm_vector.at(i)) {
      if (fsm_info::is_double_stance(fsm_vector.at(i))) {
        mpc_reference->touchdown_ee_names_.at(i) =
            fsm_vector.at(i) == fsm_info::kPostRightDouble ?
            params_.right_foot_body_name : params_.left_foot_body_name;
        mpc_reference->touchdown_ee_points_.at(i) = params_.foot_midpoint;
      }
    }
  }
  return fsm_vector;
}

void WalkingReferenceSystem::SetContactsAtKnot(
    int i, fsm_info::fsm_state state, MPCReference *ref) const {
  if (state != fsm_info::kLeft) {
    ref->AppendContactsToKnot(i, params_.right_foot_contacts);
  }
  if (state != fsm_info::kRight) {
    ref->AppendContactsToKnot(i, params_.left_foot_contacts);
  }
}

PiecewisePolynomial<double> WalkingReferenceSystem::CalcPositionTraj(
    fsm_info::fsm_state fsm_state, const Vector2d& vdes,
    const std::vector<double>& breaks) const {

  bool is_left = fsm_state == fsm_info::kLeft or fsm_state == fsm_info::kPostLeftDouble;

  std::string contact_frame = is_left ? params_.left_foot_body_name :
                                        params_.right_foot_body_name;

  Vector3d p_W;
  plant_.CalcPointsPositions(
      *plant_context_, plant_.GetBodyByName(contact_frame).body_frame(),
      params_.foot_midpoint, plant_.world_frame(), &p_W);

  auto R_yaw = multibody::GetBodyYawRotation_R_WB<double>(
      plant_, *plant_context_, params_.floating_base_name);

  Vector3d pelvis_offset_from_stance_foot = is_left ?
      -0.5 * params_.stance_width * R_yaw * Vector3d::UnitY() :
       0.5 * params_.stance_width * R_yaw * Vector3d::UnitY();

  Vector3d pelvis_offset = p_W + pelvis_offset_from_stance_foot;

  Vector4d quat_ref = plant_.GetPositions(*plant_context_).head<4>();
  quat_ref(1) = 0;
  quat_ref(2) = 0;
  quat_ref.normalize();

  Vector2d v_W = R_yaw.topLeftCorner<2, 2>() * vdes;

  VectorXd q0 = params_.standing_pose_q;
  q0.head<4>() = quat_ref;
  q0.segment<3>(4) += pelvis_offset;

  VectorXd qf = q0;
  qf.segment<2>(4) += v_W * (breaks.back() - breaks.front());

  return PiecewisePolynomial<double>::FirstOrderHold(
      {breaks.front(), breaks.back()},
      {q0, qf});
}

PiecewisePolynomial<double> WalkingReferenceSystem::CalcOrientationTraj() const {
  Vector4d quat_ref = plant_.GetPositions(*plant_context_).head<4>();
  quat_ref(1) = 0;
  quat_ref(2) = 0;
  quat_ref.normalize();

  return PiecewisePolynomial<double>(quat_ref);
}

PiecewisePolynomial<double> WalkingReferenceSystem::CalcVelocityTraj(
    const Vector2d& vdes) const {
  auto R_yaw = multibody::GetBodyYawRotation_R_WB<double>(
      plant_, *plant_context_, params_.floating_base_name);
  Vector2d v_W = R_yaw.topLeftCorner<2, 2>() * vdes;
  VectorXd v = VectorXd::Zero(dynamics_.nv());
  v.segment<2>(3) = v_W;

  return PiecewisePolynomial<double>(v);
}

PiecewisePolynomial<double> WalkingReferenceSystem::CalcInputTraj(
    const std::vector<double>& breaks,
    const std::vector<fsm_info::fsm_state> fsm_states) const {
  std::vector<MatrixXd> uu;
  for (const auto& state : fsm_states) {
    if (fsm_info::is_double_stance(state)) {
      uu.push_back(params_.standing_pose_u);
    } else {
      VectorXd u = VectorXd::Zero(dynamics_.nu());
      const std::vector<int>& active_u_idxs = state == fsm_info::kLeft ?
          params_.left_leg_actuator_idxs : params_.right_leg_actuator_idxs;
      for (int i : active_u_idxs) {
        u(i) = 2.0 * params_.standing_pose_u(i);
      }
      uu.push_back(u);
    }
  }
  return PiecewisePolynomial<double>::FirstOrderHold(breaks, uu);
}

PiecewisePolynomial<double> WalkingReferenceSystem::CalcLambdaTraj(
    const std::vector<double>& breaks,
    const std::vector<fsm_info::fsm_state>& fsm_states,
    const std::vector<std::vector<std::string>>& active_contacts) const {
  std::vector<MatrixXd> ll;
  for (int i = 0; i < fsm_states.size(); ++i) {
    fsm_info::fsm_state state = fsm_states.at(i);
    if (fsm_info::is_double_stance(state)) {
      ll.push_back(params_.standing_pose_lambda);
    } else {
      VectorXd l2 = 2.0 * params_.standing_pose_lambda;
      VectorXd l = dynamics_.get_lambda_for_active_contacts<double>(active_contacts.at(i), l2);
      const std::vector<int>& active_l_idxs = state == fsm_info::kLeft ?
          params_.left_leg_holonomic_constraint_idxs :
          params_.right_leg_holonomic_constraint_idxs;
      for (int j : active_l_idxs) {
        l(j) = l2(j);
      }
      ll.push_back(l);
    }
  }
  return PiecewisePolynomial<double>::FirstOrderHold(breaks, ll);
}

std::vector<double> WalkingReferenceSystem::CalcSSPhaseVector(
    const fsm_info fsm,
    const std::vector<double>& breaks,
    const std::vector<fsm_info::fsm_state>& fsm_vector) const {

}

PiecewisePolynomial<double> WalkingReferenceSystem::CalcSwingFootTraj(
    const std::vector<double> &breaks, const std::vector<fsm_info::fsm_state> &fsm_states) const {

}

}