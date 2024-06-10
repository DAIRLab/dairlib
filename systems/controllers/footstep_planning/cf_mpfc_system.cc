#include "cf_mpfc_system.h"

#include "common/eigen_utils.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include <iostream>

namespace dairlib::systems::controllers {

using multibody::ReExpressWorldVector3InBodyYawFrame;
using multibody::ReExpressBodyYawVector3InWorldFrame;
using multibody::GetBodyYawRotation_R_WB;
using multibody::SetPositionsAndVelocitiesIfNew;

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



CFMPFCSystem::CFMPFCSystem(
    const MultibodyPlant<double>& plant, Context<double>* plant_context,
    drake::multibody::ModelInstanceIndex model_instance,
    std::vector<int> left_right_stance_fsm_states,
    std::vector<int> post_left_right_fsm_states,
    std::vector<PointOnFramed> left_right_foot,
    const cf_mpfc_params& mpfc_params,
    std::function<Matrix3d(const MultibodyPlant<double>&, const Context<double>&)> acom_function) :
    plant_(plant),
    context_(plant_context),
    model_instance_(model_instance),
    acom_function_(acom_function),
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
      drake::Value<cf_mpfc_solution>()
  );

  // State Update
  this->DeclareForcedUnrestrictedUpdateEvent(
      &CFMPFCSystem::UnrestrictedUpdate);

  // Input ports
  state_input_port_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(nq_, nv_, nu_))
      .get_index();
  vdes_input_port_ = DeclareVectorInputPort("vdes_x_y", 2).get_index();

  // output ports
  mpc_output_port_ = DeclareAbstractOutputPort(
      "lcmt_alip_mpc_output", &CFMPFCSystem::CopyMpcOutput
  ).get_index();

  fsm_output_port_ = DeclareVectorOutputPort(
      "fsm", 1, &CFMPFCSystem::CopyFsmOutput).get_index();
}

drake::systems::EventStatus CFMPFCSystem::UnrestrictedUpdate(
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

  // get state and time from robot_output, set plant context
  const VectorXd robot_state = robot_output->GetState();
  double t = robot_output->get_timestamp();
  SetPositionsAndVelocitiesIfNew<double>(plant_, robot_state, context_);

  // initialize local variables
  int fsm_idx =
      static_cast<int>(state->get_discrete_state(fsm_state_idx_).get_value()(0));

  Vector3d CoM_w = Vector3d::Zero();
  Vector3d p_w = Vector3d::Zero();

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

  double t_elapsed_this_mode = t - t_prev_impact;
  double tnom_remaining = single_stance_duration_ + double_stance_duration_ - t_elapsed_this_mode;
  tnom_remaining = std::max(tnom_remaining, 0.0);

  const int fsm_state = curr_fsm(fsm_idx);
  Stance stance = left_right_stance_fsm_states_.at(fsm_idx) == 0? Stance::kLeft : Stance::kRight;


  const alip_utils::PointOnFramed& stance_foot = stance_foot_map_.at(fsm_state);
  // Get the state
  plant_.CalcPointsPositions(
      *context_, stance_foot.second,
      stance_foot.first,
      plant_.world_frame(),
      &p_w
  );

  const auto& floating_base_frame = plant_.get_body(
      *(plant_.GetFloatingBaseBodies().begin())).body_frame();

  Matrix3d I = cf_mpfc_utils::CalcRotationalInertiaAboutCoM(
      plant_, *context_, model_instance_, floating_base_frame
  ).CopyToFullMatrix3();

  Vector3d p_b = ReExpressWorldVector3InBodyYawFrame<double>(
      plant_, *context_, "pelvis", p_w);

  cf_mpfc_utils::CentroidalState<double> x = cf_mpfc_utils::GetCentroidalState(
      plant_, *context_, model_instance_, acom_function_, stance_foot);

  VectorXd init_alip_state_and_stance_pos = VectorXd::Zero(7);


  auto prev_sol = state->get_abstract_state<cf_mpfc_solution>(mpc_solution_idx_);

  const auto mpc_solution = trajopt_.Solve(
      x, p_b, tnom_remaining, vdes, stance, I, prev_sol
  );

  // Update discrete states
  state->get_mutable_discrete_state(fsm_state_idx_).set_value(
      fsm_idx*VectorXd::Ones(1));

  state->get_mutable_discrete_state(next_impact_time_state_idx_).set_value(
      (t_next_impact) * VectorXd::Ones(1));
  state->get_mutable_discrete_state(prev_impact_time_state_idx_).set_value(
      t_prev_impact * VectorXd::Ones(1));
  state->get_mutable_discrete_state(initial_conditions_state_idx_).set_value(
      init_alip_state_and_stance_pos);
  if (mpc_solution.success) {
    state->get_mutable_abstract_state<cf_mpfc_solution>(mpc_solution_idx_) = mpc_solution;
  }

  return drake::systems::EventStatus::Succeeded();
}

void CFMPFCSystem::CopyFsmOutput(
    const Context<double> &context, BasicVector<double> *fsm) const {
  fsm->get_mutable_value() << GetFsmForOutput(context);
}

void CFMPFCSystem::CopyMpcOutput(
    const Context<double> &context, lcmt_alip_mpc_output* mpc_output) const {

  const auto& mpc_sol =
      context.get_abstract_state<cf_mpfc_solution>(mpc_solution_idx_);

  // Copy next footstep
  Vector3d footstep_in_stance_frame = mpc_sol.pp.at(1) - mpc_sol.pp.at(0);
  footstep_in_stance_frame(2) = 0;
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

int CFMPFCSystem::GetFsmForOutput(
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

double CFMPFCSystem::GetPrevImpactTimeForOutput(
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


void CFMPFCSystem::CopyAnkleTorque(
    const Context<double> &context, lcmt_saved_traj *traj) const {
  double t  = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_))->get_timestamp();
  const auto& mpc_sol =
      context.get_abstract_state<cf_mpfc_solution>(mpc_solution_idx_);

  // TODO (@Brian-Acosta) copy ankle torque for testing, then change OSC for
  //  force tracking

//  throw std::runtime_error("uninitialized");
  double u_sol = 0;

  LcmTrajectory::Trajectory input_traj;
  input_traj.traj_name = "input_traj";
  input_traj.datatypes = vector<std::string>(1, "double");
  input_traj.datapoints = Eigen::RowVector2d::Constant(u_sol);
  input_traj.time_vector = Eigen::Vector2d(t, std::numeric_limits<double>::infinity());
  LcmTrajectory lcm_traj(
      {input_traj}, {"input_traj"}, "input_traj", "input_traj", false);
  *traj = lcm_traj.GenerateLcmObject();
}


}