#include <cmath>
#include "walking_reference_system.h"
#include "systems/framework/output_vector.h"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::systems::State;
using drake::systems::Context;
using drake::systems::EventStatus;

WalkingReferenceSystem::WalkingReferenceSystem(
    const drake::multibody::MultibodyPlant<double> &plant,
    Context<double> *plant_context, const GaitParams &params) :
    plant_(plant), plant_context_(plant_context), params_(params) {

  input_port_state_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(plant_)).get_index();

  input_port_vdes_ = DeclareVectorInputPort("vdes", 2).get_index();

  ds_intervals_ = std::round<double>(params_.t_ds / params_.mpc_dt);
  ss_intervals_ = std::round<double>(params_.t_ss / params_.mpc_dt);

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

  double t = robot_output.get_timestamp();
  const auto &fsm = state->get_abstract_state<fsm_info>(fsm_info_idx_);
  fsm_info next_fsm = CalcFSM(t, fsm);

  auto &mpc_reference =
      state->get_mutable_abstract_state<MPCReference>(reference_state_idx_);

  CalcGaitTiming(t, next_fsm, &mpc_reference);

  state->get_mutable_abstract_state<fsm_info>(fsm_info_idx_) = next_fsm;

  // TODO (@Brian-Acosta) add the desired trajectories

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

void WalkingReferenceSystem::CalcGaitTiming(
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

}