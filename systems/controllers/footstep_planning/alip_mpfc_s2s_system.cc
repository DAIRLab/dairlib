#include "alip_mpfc_s2s_system.h"
#include "common/eigen_utils.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include <iostream>

namespace dairlib::systems::controllers {

using multibody::ReExpressWorldVector3InBodyYawFrame;
using multibody::GetBodyYawRotation_R_WB;
using multibody::SetPositionsAndVelocitiesIfNew;
using geometry::ConvexPolygonSet;
using geometry::ConvexPolygon;
using alip_utils::Stance;

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

Alips2sMPFCSystem::Alips2sMPFCSystem(
    const MultibodyPlant<double>& plant, Context<double>* plant_context,
    std::vector<int> left_right_stance_fsm_states,
    std::vector<int> post_left_right_fsm_states,
    std::vector<PointOnFramed> left_right_foot,
    const alip_s2s_mpfc_params& mpfc_params) :
    plant_(plant),
    context_(plant_context),
    trajopt_(mpfc_params),
    left_right_stance_fsm_states_(left_right_stance_fsm_states),
    post_left_right_fsm_states_(post_left_right_fsm_states),
    double_stance_duration_(mpfc_params.gait_params.double_stance_duration),
    single_stance_duration_(mpfc_params.gait_params.single_stance_duration) {

  // just alternating single stance phases for now.
  DRAKE_DEMAND(left_right_stance_fsm_states_.size() == 2);
  DRAKE_DEMAND(left_right_foot.size() == 2);

  nq_ = plant_.num_positions();
  nv_ = plant_.num_velocities();
  nu_ = plant_.num_actuators();

  for (int i = 0; i < left_right_stance_fsm_states_.size(); i++){
    stance_foot_map_.insert(
        {left_right_stance_fsm_states_.at(i), left_right_foot.at(i)});
  }

  // Must declare the discrete states before assigning their output ports so
  // that the indexes can be used to call DeclareStateOutputPort
  fsm_state_idx_ = DeclareDiscreteState(1);
  next_impact_time_state_idx_ = DeclareDiscreteState(1);
  prev_impact_time_state_idx_ = DeclareDiscreteState(1);
  initial_conditions_state_idx_ = DeclareDiscreteState(4+3);

  mpc_solution_idx_ = DeclareAbstractState(
      drake::Value<alip_s2s_mpfc_solution>()
  );

  footholds_idx_ = DeclareAbstractState(
      drake::Value<ConvexPolygonSet>(ConvexPolygonSet::MakeFlatGround())
  );


  // State Update
  this->DeclareForcedUnrestrictedUpdateEvent(
      &Alips2sMPFCSystem::UnrestrictedUpdate);

  // Input ports
  state_input_port_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(nq_, nv_, nu_))
      .get_index();
  vdes_input_port_ = DeclareVectorInputPort("vdes_x_y", 2).get_index();
  foothold_input_port_ = DeclareAbstractInputPort(
      "footholds", drake::Value<ConvexPolygonSet>())
      .get_index();

  // output ports
  mpc_output_port_ = DeclareAbstractOutputPort(
      "lcmt_alip_mpc_output", &Alips2sMPFCSystem::CopyMpcOutput
  ).get_index();
  mpc_debug_output_port_ = DeclareAbstractOutputPort(
      "lcmt_mpc_debug", &Alips2sMPFCSystem::CopyMpcDebugToLcm
  ).get_index();
  fsm_output_port_ = DeclareVectorOutputPort(
      "fsm", 1, &Alips2sMPFCSystem::CopyFsmOutput).get_index();
}

drake::systems::EventStatus Alips2sMPFCSystem::UnrestrictedUpdate(
    const Context<double> &context, State<double> *state) const {

  // evaluate input ports
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));
  const Vector2d& vdes =
      this->EvalVectorInput(context, vdes_input_port_)->get_value();
  double t_next_impact =
      state->get_discrete_state(next_impact_time_state_idx_).get_value()(0);
  double t_prev_impact =
      state->get_discrete_state(prev_impact_time_state_idx_).get_value()(0);
  auto foothold_set = this->EvalAbstractInput(context, foothold_input_port_)->
      get_value<ConvexPolygonSet>();

  // get state and time from robot_output, set plant context
  const VectorXd robot_state = robot_output->GetState();
  double t = robot_output->get_timestamp();
  SetPositionsAndVelocitiesIfNew<double>(plant_, robot_state, context_);

  // re-express footholds in robot yaw frame from world frame
  foothold_set.ReExpressInNewFrame(
      GetBodyYawRotation_R_WB<double>(plant_, *context_, "pelvis"));

  // initialize local variables
  int fsm_idx =
      static_cast<int>(state->get_discrete_state(fsm_state_idx_).get_value()(0));

  Vector3d CoM_w = Vector3d::Zero();
  Vector3d p_w = Vector3d::Zero();
  Vector3d p_next_in_ds = Vector3d::Zero();
  Vector3d L = Vector3d::Zero();

  // On the first iteration, we don't want to switch immediately or warmstart
  if (t_next_impact == 0.0) {
    t_next_impact = t + single_stance_duration_ + double_stance_duration_;
    t_prev_impact = t;
  }

  // Check if it's time to switch the fsm or commit to the current footstep
  if (t >= t_next_impact) {
    fsm_idx ++;
    fsm_idx = fsm_idx >= left_right_stance_fsm_states_.size() ? 0 : fsm_idx;
    t_prev_impact = t;
    t_next_impact = t + double_stance_duration_ + single_stance_duration_;
  }

  const int fsm_state = curr_fsm(fsm_idx);
  Stance stance = left_right_stance_fsm_states_.at(fsm_idx) == 0? Stance::kLeft : Stance::kRight;

  double ds_fraction = std::clamp(
      (t - t_prev_impact) / double_stance_duration_, 0.0, 1.0);
  std::vector<double> CoP_fractions = {ds_fraction, 1.0 - ds_fraction};

  // get the alip state
  alip_utils::CalcAlipState(
      plant_, context_, robot_state,
      {stance_foot_map_.at(fsm_state), stance_foot_map_.at(next_fsm(fsm_idx))},
      CoP_fractions, &CoM_w, &L, &p_w);

  plant_.CalcPointsPositions(
      *context_,
      stance_foot_map_.at(fsm_state).second,
      stance_foot_map_.at(fsm_state).first,
      plant_.world_frame(),
      &p_next_in_ds
  );

  p_next_in_ds = ReExpressWorldVector3InBodyYawFrame<double>(
      plant_, *context_, "pelvis", p_next_in_ds);
  Vector3d CoM_b = ReExpressWorldVector3InBodyYawFrame<double>(
      plant_, *context_, "pelvis", CoM_w);
  Vector3d p_b = ReExpressWorldVector3InBodyYawFrame<double>(
      plant_, *context_, "pelvis", p_w);
  Vector3d L_b = ReExpressWorldVector3InBodyYawFrame<double>(
      plant_, *context_, "pelvis", L);

  Vector4d x = Vector4d::Zero();
  x.head<2>() = CoM_b.head<2>() - p_b.head<2>();
  x.tail<2>() = L_b.head<2>();

  double time_left_in_this_mode = t_next_impact - t;
  bool is_ds = t - t_prev_impact < double_stance_duration_;

  if (is_ds) {
    double tds = double_stance_duration_ - (t - t_prev_impact);
    x = alip_utils::CalcReset(
        trajopt_.gait_params().height,
        trajopt_.gait_params().mass,
        tds, x, p_b, p_next_in_ds,
        trajopt_.gait_params().reset_discretization_method
    );
    p_b = p_next_in_ds;
    time_left_in_this_mode = single_stance_duration_;
  }

  VectorXd init_alip_state_and_stance_pos = VectorXd::Zero(7);
  init_alip_state_and_stance_pos.head<4>() = x;
  init_alip_state_and_stance_pos.tail<3>() = p_b;

  ConvexPolygonSet footholds_filt;
  const auto& prev_footholds = state->get_abstract_state<ConvexPolygonSet>(
      footholds_idx_
  );

  if (!foothold_set.empty()) {
    footholds_filt = foothold_set.GetSubsetCloseToPoint(p_next_in_ds, 1.8);
  } else {
    std::cerr << "WARNING: No new footholds specified!\n";
  }

  if (footholds_filt.empty()) {
    footholds_filt = prev_footholds;
  }

  const auto mpc_solution = trajopt_.Solve(
      x, p_b, time_left_in_this_mode, vdes, stance, footholds_filt
  );

  // Update discrete states
  state->get_mutable_discrete_state(fsm_state_idx_).set_value(
      fsm_idx*VectorXd::Ones(1));

  if (mpc_solution.success and not is_ds) {
    state->get_mutable_discrete_state(next_impact_time_state_idx_).set_value(
        (t + mpc_solution.t_sol) * VectorXd::Ones(1));
  } else {
    state->get_mutable_discrete_state(next_impact_time_state_idx_).set_value(
        (t_next_impact) * VectorXd::Ones(1));
  }

  state->get_mutable_discrete_state(prev_impact_time_state_idx_).set_value(
      t_prev_impact * VectorXd::Ones(1));
  state->get_mutable_discrete_state(initial_conditions_state_idx_).set_value(
      init_alip_state_and_stance_pos);
  if (mpc_solution.success) {
    state->get_mutable_abstract_state<alip_s2s_mpfc_solution>(mpc_solution_idx_) = mpc_solution;
  }
  state->get_mutable_abstract_state<ConvexPolygonSet>(footholds_idx_) = footholds_filt;

  return drake::systems::EventStatus::Succeeded();
}

void Alips2sMPFCSystem::CopyFsmOutput(
    const Context<double> &context, BasicVector<double> *fsm) const {
  fsm->get_mutable_value() << GetFsmForOutput(context);
}

void Alips2sMPFCSystem::CopyMpcOutput(
    const Context<double> &context, lcmt_alip_mpc_output* mpc_output) const {

  const auto& mpc_sol =
      context.get_abstract_state<alip_s2s_mpfc_solution>(mpc_solution_idx_);

  // Copy next footstep
  Vector3d footstep_in_stance_frame = mpc_sol.pp.at(1) - mpc_sol.pp.at(0);
  for (int i = 0; i < 3; i++) {
    mpc_output->next_footstep_in_stance_frame[i] = footstep_in_stance_frame(i);
  }

  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));

  // copy fsm info
  mpc_output->fsm.fsm_state = GetFsmForOutput(context);
  mpc_output->fsm.prev_switch_time_us = 1e6 * GetPrevImpactTimeForOutput(context);
  mpc_output->fsm.timestamp_us = 1e6 * robot_output->get_timestamp();
  mpc_output->fsm.next_switch_time_us = 1e6 *
      context.get_discrete_state(next_impact_time_state_idx_).get_value()(0);

  // copy ankle torque traj
  CopyAnkleTorque(context, &(mpc_output->u_traj));
}

namespace {

vector<int> get_foothold_indices(const vector<VectorXd>& binary_vars) {
  vector<int> indices;
  for (const auto& v : binary_vars) {
    int i = 0;
    while(v(i) < 0.5 and i < v.rows()) {
      ++i;
    }
    indices.push_back(i);
  }
  return indices;
}

ConvexPolygonSet get_foothold_sequence(const vector<VectorXd>& binary_vars,
                                       const ConvexPolygonSet& set) {
  auto indices = get_foothold_indices(binary_vars);
  const auto& foothold_list = set.polygons();
  ConvexPolygonSet ret;
  for (const auto& i : indices) {
    ret.append(foothold_list.at(i));
  }
  return ret;
}

}

void Alips2sMPFCSystem::CopyMpcDebugToLcm(
    const Context<double> &context, lcmt_alip_s2s_mpfc_debug *mpc_debug) const {

  const auto& ic =
      context.get_discrete_state(initial_conditions_state_idx_).get_value();
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));


  int utime = static_cast<int>(robot_output->get_timestamp() * 1e6);
  double fsmd = context.get_discrete_state(fsm_state_idx_).get_value()(0);
  int fsm = curr_fsm(static_cast<int>(fsmd));

  const auto& mpc_sol =
      context.get_abstract_state<alip_s2s_mpfc_solution>(mpc_solution_idx_);

  mpc_debug->utime = utime;
  mpc_debug->fsm_state = fsm;
  mpc_debug->solve_time_us = mpc_sol.total_time * 1e6;
  mpc_debug->solution_result = std::to_string(mpc_sol.solution_result);

  mpc_debug->nx = 4;
  mpc_debug->np = 3;
  mpc_debug->nmodes = mpc_sol.xx.size();

  mpc_debug->initial_state.reserve(4);
  Eigen::Map<Vector4d>(mpc_debug->initial_state.data(), 4) = ic.head<4>();

  mpc_debug->initial_stance_foot.reserve(3);
  Eigen::Map<Vector3d>(mpc_debug->initial_stance_foot.data(), 3) = ic.tail<3>();

  mpc_debug->nominal_first_stance_time = mpc_sol.t_nom;
  mpc_debug->solution_first_stance_time = mpc_sol.t_sol;

  mpc_debug->pp.clear();
  mpc_debug->xx.clear();

  for (int i = 0; i < mpc_sol.xx.size() ; ++i) {
    vector<double> x(4);
    vector<double> p(3);
    Vector4d::Map(x.data()) = mpc_sol.xx.at(i);
    Vector3d::Map(p.data()) = mpc_sol.pp.at(i);
    mpc_debug->xx.push_back(x);
    mpc_debug->pp.push_back(p);
  }

  Vector2d::Map(mpc_debug->desired_velocity) = mpc_sol.desired_velocity;
  get_foothold_sequence(mpc_sol.mu, mpc_sol.input_footholds).CopyToLcm(
      &(mpc_debug->foothold_sequence)
  );
}


int Alips2sMPFCSystem::GetFsmForOutput(
    const Context<double> &context) const {
  double t_prev =
      context.get_discrete_state(prev_impact_time_state_idx_).get_value()(0);
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));
  int fsm_idx = static_cast<int>(
      context.get_discrete_state(fsm_state_idx_).get_value()(0));

  if (robot_output->get_timestamp() - t_prev < double_stance_duration_) {
    return post_left_right_fsm_states_.at(fsm_idx);
  }
  return left_right_stance_fsm_states_.at(fsm_idx);
}

double Alips2sMPFCSystem::GetPrevImpactTimeForOutput(
    const Context<double> &context) const {
  double t_prev =
      context.get_discrete_state(prev_impact_time_state_idx_).get_value()(0);
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));
  if (robot_output->get_timestamp() - t_prev < double_stance_duration_) {
    return t_prev;
  }
  return t_prev + double_stance_duration_;
}


void Alips2sMPFCSystem::CopyAnkleTorque(
    const Context<double> &context, lcmt_saved_traj *traj) const {
  double t  = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_))->get_timestamp();

  LcmTrajectory::Trajectory input_traj;
  input_traj.traj_name = "input_traj";
  input_traj.datatypes = vector<std::string>(1, "double");
  input_traj.datapoints = Eigen::RowVector2d::Zero();
  input_traj.time_vector = Eigen::Vector2d(t, std::numeric_limits<double>::infinity());
  LcmTrajectory lcm_traj(
      {input_traj}, {"input_traj"}, "input_traj", "input_traj", false);
  *traj = lcm_traj.GenerateLcmObject();
}


}