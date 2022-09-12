#include <iostream>
#include "alip_minlp_footstep_controller.h"
#include "common/eigen_utils.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib::systems::controllers {

using multibody::ReExpressWorldVector3InBodyYawFrame;
using multibody::GetBodyYawRotation_R_WB;
using multibody::SetPositionsAndVelocitiesIfNew;
using geometry::ConvexFoothold;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

using drake::AbstractValue;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::State;

AlipMINLPFootstepController::AlipMINLPFootstepController(
    const MultibodyPlant<double>& plant, Context<double>* plant_context,
    std::vector<int> left_right_stance_fsm_states,
    std::vector<int> post_left_right_fsm_states,
    std::vector<double> left_right_stance_durations,
    double double_stance_duration,
    std::vector<PointOnFramed> left_right_foot,
    const AlipMINLPGains& gains) :
    plant_(plant),
    context_(plant_context),
    left_right_stance_fsm_states_(left_right_stance_fsm_states),
    post_left_right_fsm_states_(post_left_right_fsm_states),
    double_stance_duration_(double_stance_duration),
    gains_(gains) {

  // just alternating single stance phases for now.
  DRAKE_DEMAND(left_right_stance_fsm_states_.size() == 2);
  DRAKE_DEMAND(left_right_stance_durations.size() == 2);
  DRAKE_DEMAND(left_right_foot.size() == 2);
  DRAKE_DEMAND(gains_.t_commit > gains_.t_min);

  nq_ = plant_.num_positions();
  nv_ = plant_.num_velocities();
  nu_ = plant_.num_actuators();

  // TODO: @Brian-Acosta Add double stance here when appropriate
  for (int i = 0; i < left_right_stance_fsm_states_.size(); i++){
    stance_duration_map_.insert({left_right_stance_fsm_states_.at(i), left_right_stance_durations.at(i)});
    stance_foot_map_.insert({left_right_stance_fsm_states_.at(i), left_right_foot.at(i)});
  }

  // Must declare the discrete states before assigning their output ports so
  // that the indexes can be used to call DeclareStateOutputPort
  fsm_state_idx_ = DeclareDiscreteState(1);
  next_impact_time_state_idx_ = DeclareDiscreteState(1);
  prev_impact_time_state_idx_ = DeclareDiscreteState(1);
  initial_conditions_state_idx_ = DeclareDiscreteState(4+3);

  // Build the optimization problem
  auto trajopt = AlipMINLP(plant_.CalcTotalMass(*context_), gains_.hdes);
  for (int n = 0; n < gains_.nmodes; n++) {
    trajopt.AddMode(gains_.knots_per_mode);
  }
  auto xd = trajopt.MakeXdesTrajForVdes(
      Vector2d::Zero(), gains_.stance_width, stance_duration_map_.at(0),
      gains_.knots_per_mode);
  trajopt.AddTrackingCost(xd, gains_.Q);
  trajopt.AddInputCost(gains_.R(0,0));
  trajopt.UpdateNominalStanceTime(left_right_stance_durations.at(0),
                                  left_right_stance_durations.at(1));
  trajopt.SetMinimumStanceTime(gains_.t_min);
  trajopt.SetMaximumStanceTime(gains_.t_max);
  trajopt.SetInputLimit(gains_.u_max);
  trajopt.Build();
  trajopt.CalcOptimalFootstepPlan(Vector4d::Zero(), Vector3d::Zero());
  alip_minlp_idx_ = DeclareAbstractState(*AbstractValue::Make<AlipMINLP>(trajopt));

  if (gains_.filter_alip_state) {
    auto filter = S2SKalmanFilter(gains_.filter_data);
    std::pair<S2SKalmanFilter, S2SKalmanFilterData> model_filter =
        {filter, gains_.filter_data};
    alip_filter_idx_ = DeclareAbstractState(drake::Value<
        std::pair<S2SKalmanFilter, S2SKalmanFilterData>>(model_filter));
  }

  // State Update
  this->DeclarePerStepUnrestrictedUpdateEvent(
      &AlipMINLPFootstepController::UnrestrictedUpdate);

  // Input ports
  state_input_port_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(nq_, nv_, nu_))
      .get_index();
  vdes_input_port_ = DeclareVectorInputPort("vdes_x_y", 2).get_index();
  foothold_input_port_ = DeclareAbstractInputPort(
      "footholds", drake::Value<std::vector<ConvexFoothold>>())
      .get_index();

  // output ports

  next_impact_time_output_port_ = DeclareStateOutputPort(
      "t_next", next_impact_time_state_idx_)
      .get_index();

  prev_impact_time_output_port_ = DeclareVectorOutputPort(
      "t_prev", 1, &AlipMINLPFootstepController::CopyPrevImpactTimeOutput).get_index();
  fsm_output_port_ = DeclareVectorOutputPort(
      "fsm", 1, &AlipMINLPFootstepController::CopyFsmOutput).get_index();
  footstep_target_output_port_ = DeclareVectorOutputPort(
      "p_SW", 3, &AlipMINLPFootstepController::CopyNextFootstepOutput)
      .get_index();
  com_traj_output_port_ = DeclareAbstractOutputPort(
      "lcmt_saved_traj", &AlipMINLPFootstepController::CopyCoMTrajOutput)
      .get_index();
  mpc_debug_output_port_ = DeclareAbstractOutputPort(
      "lcmt_mpc_debug", &AlipMINLPFootstepController::CopyMpcSolutionToLcm)
      .get_index();
}

drake::systems::EventStatus AlipMINLPFootstepController::UnrestrictedUpdate(
    const Context<double> &context, State<double> *state) const {

  // evaluate input ports
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));
  const Vector2d vdes =
      this->EvalVectorInput(context, vdes_input_port_)->get_value();
  double t_next_impact =
      state->get_discrete_state(next_impact_time_state_idx_).get_value()(0);
  double t_prev_impact =
      state->get_discrete_state(prev_impact_time_state_idx_).get_value()(0);
  auto footholds = this->EvalAbstractInput(context, foothold_input_port_)->
      get_value<std::vector<ConvexFoothold>>();

  // get variables from robot_output
  const VectorXd robot_state = robot_output->GetState();
  double t = robot_output->get_timestamp();

  // re-express footholds in robot yaw frame
  SetPositionsAndVelocitiesIfNew<double>(plant_, robot_state, context_);
  for (auto& foothold : footholds) {
    foothold.ReExpressInNewFrame(
        GetBodyYawRotation_R_WB<double>(plant_, *context_, "pelvis"));
  }

  // initialize local variables
  auto& trajopt = state->get_mutable_abstract_state<AlipMINLP>(alip_minlp_idx_);
  int fsm_idx =
      static_cast<int>(state->get_discrete_state(fsm_state_idx_).get_value()(0));
  bool warmstart = true;
  bool committed = false;
  bool fsm_switch = false;
  Vector3d CoM_w;
  Vector3d p_w;
  Vector3d p_next;
  Vector3d L;

  // On the first iteration, we don't want to switch immediately or warmstart
  if (t_next_impact == 0.0) {
    std::cout << "first iteration!" << std::endl;
    t_next_impact = t + stance_duration_map_.at(0);
    t_prev_impact = t;
    warmstart = false;
  }

  // Check if it's time to switch the fsm or commit to the current footstep
  if (t >= t_next_impact) {
    warmstart = false;
    fsm_switch = true;
    fsm_idx ++;
    fsm_idx = fsm_idx >= left_right_stance_fsm_states_.size() ? 0 : fsm_idx;
    t_prev_impact = t;
    t_next_impact = t + stance_duration_map_.at(curr_fsm(fsm_idx));
  } else if ((t_next_impact - t) < gains_.t_commit) {
    committed = true;
  }

  // shorthands for the current stance foot
  const int fsm_state = curr_fsm(fsm_idx);
  int stance = left_right_stance_fsm_states_.at(fsm_idx) == 0? -1 : 1;

  // get the alip state
  alip_utils::CalcAlipState(
      plant_, context_, robot_state, {stance_foot_map_.at(fsm_state)},
      &CoM_w, &L, &p_w);
  plant_.CalcPointsPositions(*context_, stance_foot_map_.at(next_fsm(fsm_idx)).second,
      stance_foot_map_.at(next_fsm(fsm_idx)).first, plant_.world_frame(), &p_next);

  p_next = ReExpressWorldVector3InBodyYawFrame<double>(plant_, *context_, "pelvis", p_next);
  CoM_w = ReExpressWorldVector3InBodyYawFrame<double>(plant_, *context_, "pelvis", CoM_w);
  p_w = ReExpressWorldVector3InBodyYawFrame<double>(plant_, *context_, "pelvis", p_w);
  L = ReExpressWorldVector3InBodyYawFrame<double>(plant_, *context_, "pelvis", L);

  Vector4d x;
  x.head<2>() = CoM_w.head<2>() - p_w.head<2>();
  x.tail<2>() = L.head<2>();
  if (gains_.filter_alip_state) {
    auto& [filter, filter_data] = state->get_mutable_abstract_state<
        std::pair<S2SKalmanFilter,S2SKalmanFilterData>>(alip_filter_idx_);
    filter_data.A =
        alip_utils::CalcA(CoM_w(2) - p_w(2), plant_.CalcTotalMass(*context_));

    // split this assignment into 2 lines to avoid Eigen compiler error
    Vector2d u = (p_w - p_next).head<2>();
    u = fsm_switch ? u : Vector2d::Zero();

    filter.Update(filter_data, u, x, t);
    x = filter.x();
  }

  VectorXd ic = VectorXd::Zero(7);
  ic.head<4>() = x;
  ic.tail<3>() = p_w;

  trajopt.set_H(CoM_w(2) - p_w(2));

  auto xd  = trajopt.MakeXdesTrajForVdes(
      vdes, gains_.stance_width, stance_duration_map_.at(fsm_state),
      gains_.knots_per_mode, stance);

  xd.at(0) = trajopt.MakeXdesTrajForCurrentStep(
      vdes, t - t_prev_impact, t_next_impact - t,
      stance_duration_map_.at(fsm_state), gains_.stance_width, stance,
      gains_.knots_per_mode);

  // Update the trajopt problem data and solve
  if (committed) {
    trajopt.ActivateInitialTimeConstraint(t_next_impact - t);
  } else {
    trajopt.UpdateInitialTimeConstraint(gains_.t_max - (t - t_prev_impact));
  }
  trajopt.ChangeFootholds(footholds);
  trajopt.UpdateTrackingCost(xd);
  trajopt.UpdateNominalStanceTime(
      t_next_impact - t, stance_duration_map_.at(fsm_state));

  ConvexFoothold inflating_workspace;
  ConvexFoothold fixed_workspace;
  Vector3d robot_center = CoM_w;
  robot_center(2) = p_w(2);
  fixed_workspace.AddFace(-stance * Vector3d::UnitY(),
                          robot_center - 0.025 * stance * Vector3d::UnitY());
  fixed_workspace.AddFace(stance * Vector3d::UnitY(),
                          robot_center + 0.5 * stance * Vector3d::UnitY());
  fixed_workspace.AddFace(Vector3d::UnitX(), robot_center + Vector3d::UnitX());
  fixed_workspace.AddFace(-Vector3d::UnitX(), robot_center - Vector3d::UnitX());

  inflating_workspace.AddFace(Vector3d::UnitX(), p_next + Vector3d::UnitX());
  inflating_workspace.AddFace(Vector3d::UnitY(), p_next + Vector3d::UnitY());
  inflating_workspace.AddFace(-Vector3d::UnitX(), p_next + Vector3d::UnitX());
  inflating_workspace.AddFace(-Vector3d::UnitY(), p_next + Vector3d::UnitX());

  trajopt.UpdateNextFootstepReachabilityConstraint(
      fixed_workspace, inflating_workspace);

  trajopt.CalcOptimalFootstepPlan(x, p_w, warmstart);

  // Update discrete states
  double t0 = trajopt.GetTimingSolution()(0);
  state->get_mutable_discrete_state(fsm_state_idx_).set_value(
      fsm_idx*VectorXd::Ones(1));
  state->get_mutable_discrete_state(next_impact_time_state_idx_).set_value(
      (t + t0) * VectorXd::Ones(1));
  state->get_mutable_discrete_state(prev_impact_time_state_idx_).set_value(
      t_prev_impact * VectorXd::Ones(1));
  state->get_mutable_discrete_state(initial_conditions_state_idx_).set_value(ic);

  return drake::systems::EventStatus::Succeeded();
}

void AlipMINLPFootstepController::CopyNextFootstepOutput(
    const Context<double> &context, BasicVector<double> *p_B_FC) const {
  auto& trajopt = context.get_abstract_state<AlipMINLP>(alip_minlp_idx_);
  const auto& pp = trajopt.GetFootstepSolution();
  const auto& xx = trajopt.GetStateSolution();
  Vector3d footstep_in_com_yaw_frame = Vector3d::Zero();
  footstep_in_com_yaw_frame.head(2) = (pp.at(1) - pp.at(0)).head(2) - xx.front().back().head(2);
  footstep_in_com_yaw_frame(2) = -gains_.hdes;
  p_B_FC->set_value(footstep_in_com_yaw_frame);
}

void AlipMINLPFootstepController::CopyCoMTrajOutput(
    const Context<double> &context, lcmt_saved_traj *traj_msg) const {
  DRAKE_ASSERT(traj_msg != nullptr);

  const auto& trajopt =
      context.get_abstract_state<AlipMINLP>(alip_minlp_idx_);
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));
  double t_prev_switch =
      context.get_discrete_state(prev_impact_time_state_idx_).get_value()(0);
  double t_next_switch =
      context.get_discrete_state(next_impact_time_state_idx_).get_value()(0);
  int fsm_idx = static_cast<int>(
      context.get_discrete_state(fsm_state_idx_).get_value()(0));

  multibody::SetPositionsAndVelocitiesIfNew<double>(
      plant_, robot_output->GetState(), context_);

  Vector3d stance_pos;
  plant_.CalcPointsPositions(*context_,
                             stance_foot_map_.at(curr_fsm(fsm_idx)).second,
                             stance_foot_map_.at(curr_fsm(fsm_idx)).first,
                             plant_.world_frame(),
                             &stance_pos);
  Vector3d swing_pos;
  plant_.CalcPointsPositions(*context_,
                             stance_foot_map_.at(next_fsm(fsm_idx)).second,
                             stance_foot_map_.at(next_fsm(fsm_idx)).first,
                             plant_.world_frame(),
                             &swing_pos);
  double ground_height = std::min(stance_pos(2), swing_pos(2));

  double t0 = robot_output->get_timestamp();
  const auto& xx = trajopt.GetStateSolution();
  const auto& pp = trajopt.GetFootstepSolution();
  const auto& tt = trajopt.GetTimingSolution();

  LcmTrajectory::Trajectory com_traj;

  com_traj.traj_name = "com_traj";
  com_traj.datatypes = vector<std::string>(3, "double");

  int nk = gains_.knots_per_mode;
  int nm = gains_.nmodes;
  int N = (nk - 1) * nm + 1;
  double s = 1.0 / (nk - 1);

  MatrixXd com_knots = MatrixXd::Zero(3, N);
  VectorXd t = VectorXd::Zero(N);

  // Note for tracking the MPC output: this solution is in the body yaw frame
  for (int n = 0; n < nm; n++) {
    const Vector3d& pcurr = pp.at(n);
    const Vector3d& pnext = pp.at(std::min(n+1, nm-1));
    for (int k = 0; k < nk - 1; k++) {
      int idx = n * (nk-1) + k;
      double tk = t0 + tt(n) * k * s;
      double lerp = (tk - t_prev_switch) / (t_next_switch - t_prev_switch);
      t(idx) = tk;
      com_knots.col(idx).head(2) = pcurr.head(2) + xx.at(n).at(k).head(2);
      com_knots(2, idx) = ground_height +  gains_.hdes + lerp * (pnext - pcurr)(2);
    }
    t0 += tt(n);
  }
  t(N-1) = 2 * t(N-2) - t(N-3);
  com_knots.topRightCorner(2, 1) = xx.back().back().head(2) + pp.back().head(2);
  com_knots.bottomRightCorner(1, 1)(0,0) = ground_height +  gains_.hdes;

  // If we've basically already finished this mode,
  // let's move to the next so we don't get weird trajectory stuff
  if (t(nk-2) - t(0) < .0001) {
    com_knots = com_knots.rightCols((nm-1) * (nk-1));
    t = t.tail((nm-1)*(nk-1) + 1);
    t(0) = robot_output->get_timestamp();
  }

  com_traj.datapoints = com_knots;
  com_traj.time_vector = t;

  LcmTrajectory lcm_traj({com_traj}, {"com_traj"}, "com_traj", "com_traj");
  *traj_msg = lcm_traj.GenerateLcmObject();
}

void AlipMINLPFootstepController::CopyMpcSolutionToLcm(
    const Context<double> &context, lcmt_mpc_debug *mpc_debug) const {
  // Get the debug info from the context
  const auto& trajopt =
      context.get_abstract_state<AlipMINLP>(alip_minlp_idx_);
  const auto& ic =
      context.get_discrete_state(initial_conditions_state_idx_).get_value();
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));
  int utime = static_cast<int>(robot_output->get_timestamp() * 1e6);
  double fsmd = context.get_discrete_state(fsm_state_idx_).get_value()(0);
  int fsm = curr_fsm(static_cast<int>(fsmd));

  const auto& pp_sol = trajopt.GetFootstepSolution();
  const auto& xx_sol = trajopt.GetStateSolution();
  const auto& uu_sol = trajopt.GetInputSolution();
  const auto& tt_sol = trajopt.GetTimingSolution();

  mpc_debug->utime = utime;
  mpc_debug->fsm_state = fsm;
  mpc_debug->nx = 4;
  mpc_debug->nu = 1;
  mpc_debug->np = 3;
  mpc_debug->nm = gains_.nmodes;
  mpc_debug->nk = gains_.knots_per_mode;
  mpc_debug->nk_minus_one = mpc_debug->nk - 1;
  mpc_debug->x0 = CopyVectorXdToStdVector(ic.head<4>());
  mpc_debug->p0 = CopyVectorXdToStdVector(ic.tail<3>());

  mpc_debug->pp.clear();
  mpc_debug->xx.clear();
  mpc_debug->uu.clear();
  for (int n = 0; n < gains_.nmodes; n++) {
    mpc_debug->pp.push_back(CopyVectorXdToStdVector(pp_sol.at(n)));
    std::vector<std::vector<double>> xx_temp;
    std::vector<std::vector<double>> uu_temp;
    for (int k = 0; k < gains_.knots_per_mode; k++) {
      xx_temp.push_back(CopyVectorXdToStdVector(xx_sol.at(n).at(k)));
    }
    for(int k = 0; k < gains_.knots_per_mode - 1; k++) {
      uu_temp.push_back(CopyVectorXdToStdVector(uu_sol.at(n).at(k)));
    }
    mpc_debug->xx.push_back(xx_temp);
    mpc_debug->uu.push_back(uu_temp);
  }
  mpc_debug->tt = CopyVectorXdToStdVector(tt_sol);
}

void AlipMINLPFootstepController::CopyFsmOutput(
    const Context<double> &context, BasicVector<double> *fsm) const {
  double t_prev =
      context.get_discrete_state(prev_impact_time_state_idx_).get_value()(0);
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));
  int fsm_idx = static_cast<int>(
      context.get_discrete_state(fsm_state_idx_).get_value()(0));

  if(robot_output->get_timestamp() - t_prev < double_stance_duration_) {
    fsm->set_value(VectorXd::Ones(1) * post_left_right_fsm_states_.at(fsm_idx));
  } else {
    fsm->set_value(VectorXd::Ones(1) * left_right_stance_fsm_states_.at(fsm_idx));
  }
}

void AlipMINLPFootstepController::CopyPrevImpactTimeOutput(
    const Context<double> &context, BasicVector<double> *t) const {
  double t_prev =
      context.get_discrete_state(prev_impact_time_state_idx_).get_value()(0);
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));
  if (robot_output->get_timestamp() - t_prev < double_stance_duration_) {
    t->set_value(VectorXd::Ones(1) * t_prev);
  } else {
    t->set_value(VectorXd::Ones(1) * (t_prev + double_stance_duration_));
  }
}


}