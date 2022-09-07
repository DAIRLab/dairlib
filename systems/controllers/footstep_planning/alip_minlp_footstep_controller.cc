#include <iostream>
#include "alip_minlp_footstep_controller.h"
#include "common/eigen_utils.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib::systems::controllers {

using multibody::ReExpressWorldVector3InBodyYawFrame;
using multibody::GetBodyYawRotation_R_WB;
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
    const drake::multibody::MultibodyPlant<double> &plant,
    drake::systems::Context<double> *plant_context,
    std::vector<int> left_right_stance_fsm_states,
    std::vector<double> left_right_stance_durations,
    std::vector<PointOnFramed> left_right_foot,
    const AlipMINLPGains& gains) :
    plant_(plant),
    context_(plant_context),
    left_right_stance_fsm_states_(left_right_stance_fsm_states),
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
  trajopt.SetNominalStanceTime(left_right_stance_durations.at(0),
                               left_right_stance_durations.at(1));
  trajopt.SetMinimumStanceTime(gains_.t_min);
  trajopt.Build();
  trajopt.CalcOptimalFootstepPlan(Vector4d::Zero(), Vector3d::Zero());
  std::cout << "solution is: " << trajopt.GetFootstepSolution().at(0).transpose() << std::endl;
  alip_minlp_index_ = DeclareAbstractState(*AbstractValue::Make<AlipMINLP>(trajopt));

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
  fsm_output_port_ = DeclareStateOutputPort("fsm", fsm_state_idx_).get_index();
  next_impact_time_output_port_ = DeclareStateOutputPort(
      "t_next", next_impact_time_state_idx_)
      .get_index();
  prev_impact_time_output_port_ = DeclareStateOutputPort(
      "t_prev", prev_impact_time_state_idx_)
      .get_index();
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

  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));
  const Vector2d vdes =
      this->EvalVectorInput(context, vdes_input_port_)->get_value();
  std::vector<ConvexFoothold> footholds =
      this->EvalAbstractInput(context, foothold_input_port_)
      ->get_value<std::vector<ConvexFoothold>>();

  multibody::SetPositionsAndVelocitiesIfNew<double>(
      plant_, robot_output->GetState(), context_);
  for (auto& foothold : footholds) {
    foothold.ReExpressInNewFrame(
        GetBodyYawRotation_R_WB<double>(plant_, *context_, "pelvis"));
  }

  double t = robot_output->get_timestamp();
  double t_next_impact =
      state->get_discrete_state(next_impact_time_state_idx_).get_value()(0);
  double t_prev_impact =
      state->get_discrete_state(prev_impact_time_state_idx_).get_value()(0);

  // On the first iteration, we don't want to switch immediately,
  // and we don't want to warmstart
  bool warmstart = false;
  if (t_next_impact == 0.0) {
    std::cout << "first iteration!" << std::endl;
    t_next_impact = t + stance_duration_map_.at(0);
    t_prev_impact = t;
    warmstart = false;
  }

  auto& trajopt =
      state->get_mutable_abstract_state<AlipMINLP>(alip_minlp_index_);
  trajopt.ChangeFootholds(footholds);

  int fsm_idx = static_cast<int>(
      state->get_discrete_state(fsm_state_idx_).get_value()(0));

  if (t >= t_next_impact) {
    warmstart = false;
    std::cout << "updating fsm" << std::endl;
    trajopt.DeactivateInitialTimeConstraint();
    fsm_idx ++;
    fsm_idx = fsm_idx >= left_right_stance_fsm_states_.size() ? 0 : fsm_idx;
    state->get_mutable_discrete_state(fsm_state_idx_).set_value(fsm_idx*VectorXd::Ones(1));
    t_prev_impact = t;
    t_next_impact = t + stance_duration_map_.at(left_right_stance_fsm_states_.at(fsm_idx));
  } else if ((t_next_impact - t) < gains_.t_commit) {
    trajopt.ActivateInitialTimeConstraint(t_next_impact - t);
  }
  int stance = left_right_stance_fsm_states_.at(fsm_idx) == 0? -1 : 1;

  Vector3d CoM_w, p_w, L;
  alip_utils::CalcAlipState(
      plant_, context_, robot_output->GetState(),
      {stance_foot_map_.at(left_right_stance_fsm_states_.at(fsm_idx))},
      &CoM_w, &L, &p_w);
//  CoM_w = ReExpressWorldVector3InBodyYawFrame<double>(
//      plant_, *context_, "pelvis", CoM_w);
//  p_w = ReExpressWorldVector3InBodyYawFrame(
//      plant_, *context_, "pelvis", p_w);

  trajopt.set_H(CoM_w(2) - p_w(2));

  auto xd  = trajopt.MakeXdesTrajForVdes(
      vdes, gains_.stance_width,
      stance_duration_map_.at(left_right_stance_fsm_states_.at(fsm_idx)),
      gains_.knots_per_mode, stance);

    xd.at(0) = trajopt.MakeXdesTrajForCurrentStep(
        vdes, t - t_prev_impact, t_next_impact - t,
        stance_duration_map_.at(left_right_stance_fsm_states_.at(fsm_idx)),
        gains_.stance_width, stance, gains_.knots_per_mode);
  
  trajopt.UpdateTrackingCost(xd);
  trajopt.SetNominalStanceTime(t_next_impact - t,
          stance_duration_map_.at(left_right_stance_fsm_states_.at(fsm_idx)));

  Vector4d x;
  x.head<2>() = CoM_w.head<2>() - p_w.head<2>();
  x.tail<2>() = L.head<2>();
  trajopt.CalcOptimalFootstepPlan(x, p_w, warmstart);
  VectorXd ic = VectorXd::Zero(7);
  ic.head<4>() = x;
  ic.tail<3>() = p_w;

  double t0 = trajopt.GetTimingSolution()(0);
  state->get_mutable_discrete_state(next_impact_time_state_idx_).set_value(
      (t + t0) * VectorXd::Ones(1));
  state->get_mutable_discrete_state(prev_impact_time_state_idx_).set_value(
      t_prev_impact * VectorXd::Ones(1));
  state->get_mutable_discrete_state(
      initial_conditions_state_idx_).set_value(ic);

  return drake::systems::EventStatus::Succeeded();
}

void AlipMINLPFootstepController::CopyNextFootstepOutput(
    const Context<double> &context, BasicVector<double> *p_B_FC) const {
  auto& trajopt = context.get_abstract_state<AlipMINLP>(alip_minlp_index_);
  const auto& pp = trajopt.GetFootstepSolution();
  const auto& xx = trajopt.GetStateSolution();
  Vector3d footstep_in_com_yaw_frame = Vector3d::Zero();
  footstep_in_com_yaw_frame.head(2) = pp.at(1).head<2>(); //(pp.at(1) - pp.at(0)).head(2) - xx.front().back().head(2);
  footstep_in_com_yaw_frame(2) = 0; //-gains_.hdes;
  p_B_FC->set_value(footstep_in_com_yaw_frame);
}

void AlipMINLPFootstepController::CopyCoMTrajOutput(
    const Context<double> &context, lcmt_saved_traj *traj_msg) const {
  DRAKE_ASSERT(traj_msg != nullptr);

  const auto& trajopt =
      context.get_abstract_state<AlipMINLP>(alip_minlp_index_);
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));
  double t0 = robot_output->get_timestamp();

  const auto& xx = trajopt.GetStateSolution();
  const auto& pp = trajopt.GetFootstepSolution();
  const auto& tt = trajopt.GetTimingSolution();

  LcmTrajectory::Trajectory com_traj;

  com_traj.traj_name = "com_traj";
  for (int i = 0; i < 3; i++) {
    com_traj.datatypes.emplace_back("double");
  }

  int nk = gains_.knots_per_mode;
  int nm = gains_.nmodes;
  int n = (nk - 1) * nm + 1;
  double knot_frac = 1.0 / (nk - 1);

  MatrixXd com_knots = MatrixXd::Zero(3, n);
  VectorXd t = VectorXd::Zero(n);

  // TODO: Make this readable
  for (int i = 0; i < gains_.nmodes; i++) {
    for (int k = 0; k < gains_.knots_per_mode - 1; k++) {
      int idx = i * (nk-1) + k;
      const Vector3d& pn = (i == gains_.nmodes - 1) ? pp.at(i) : pp.at(i + 1);
      t(idx) = t0 + tt(i) * k * knot_frac;
      com_knots.col(idx).head(2) = pp.at(i).head(2) + xx.at(i).at(k).head(2);
      com_knots(2, idx) = gains_.hdes + k * knot_frac * (pn - pp.at(i))(2);
    }
    t0 += tt(i);
  }
  t(n-1) = 2 * t(n-2) - t(n-3);
  com_knots.topRightCorner(2, 1) = xx.back().back().head(2) + pp.back().head(2);
  com_knots.bottomRightCorner(1, 1)(0,0) = gains_.hdes;

  com_traj.datapoints = com_knots;
  com_traj.time_vector = t;

  LcmTrajectory lcm_traj({com_traj}, {"com_traj"}, "com_traj", "com_traj");
  *traj_msg = lcm_traj.GenerateLcmObject();
}

void AlipMINLPFootstepController::CopyMpcSolutionToLcm(
    const Context<double> &context, lcmt_mpc_debug *mpc_debug) const {
  // Get the debug info from the context
  const auto& trajopt =
      context.get_abstract_state<AlipMINLP>(alip_minlp_index_);
  const auto& ic =
      context.get_discrete_state(initial_conditions_state_idx_).get_value();
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));
  int utime = static_cast<int>(robot_output->get_timestamp() * 1e6);
  double fsmd = context.get_discrete_state(fsm_state_idx_).get_value()(0);
  int fsm = left_right_stance_fsm_states_.at(static_cast<int>(fsmd));

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

}