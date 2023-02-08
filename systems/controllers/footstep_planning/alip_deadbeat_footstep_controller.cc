#include "alip_deadbeat_footstep_controller.h"
#include "common/eigen_utils.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include <iostream>

namespace dairlib::systems::controllers {

using multibody::ReExpressWorldVector3InBodyYawFrame;
using multibody::GetBodyYawRotation_R_WB;
using multibody::SetPositionsAndVelocitiesIfNew;
using geometry::ConvexFootholdSet;
using geometry::ConvexFoothold;
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

AlipDeadbeatFootstepController::AlipDeadbeatFootstepController(
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
    single_stance_duration_(left_right_stance_durations.front()),
    gains_(gains) {

  // just alternating single stance phases for now.
  DRAKE_DEMAND(left_right_stance_fsm_states_.size() == 2);
  DRAKE_DEMAND(left_right_stance_durations.size() == 2);
  DRAKE_DEMAND(left_right_foot.size() == 2);
  DRAKE_DEMAND(gains_.t_commit > gains_.t_min);

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

  if (gains_.filter_alip_state) {
    auto filter = S2SKalmanFilter(gains_.filter_data);
    std::pair<S2SKalmanFilter, S2SKalmanFilterData> model_filter =
        {filter, gains_.filter_data};
    alip_filter_idx_ = DeclareAbstractState(drake::Value<
        std::pair<S2SKalmanFilter, S2SKalmanFilterData>>(model_filter));
  }

  // State Update
  this->DeclarePerStepUnrestrictedUpdateEvent(
      &AlipDeadbeatFootstepController::UnrestrictedUpdate);

  // Input ports
  state_input_port_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(nq_, nv_, nu_))
      .get_index();
  vdes_input_port_ = DeclareVectorInputPort("vdes_x_y", 2).get_index();
  foothold_input_port_ = DeclareAbstractInputPort(
      "footholds", drake::Value<ConvexFootholdSet>())
      .get_index();

  // output ports
  next_impact_time_output_port_ = DeclareStateOutputPort(
      "t_next", next_impact_time_state_idx_)
      .get_index();
  prev_impact_time_output_port_ = DeclareVectorOutputPort(
      "t_prev", 1, &AlipDeadbeatFootstepController::CopyPrevImpactTimeOutput)
      .get_index();
  fsm_output_port_ = DeclareVectorOutputPort(
      "fsm", 1, &AlipDeadbeatFootstepController::CopyFsmOutput)
      .get_index();
  footstep_target_output_port_ = DeclareVectorOutputPort(
      "p_SW", 3, &AlipDeadbeatFootstepController::CopyNextFootstepOutput)
      .get_index();
}

drake::systems::EventStatus AlipDeadbeatFootstepController::UnrestrictedUpdate(
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
      get_value<ConvexFootholdSet>();

  // get state and time from robot_output, set plant context
  const VectorXd robot_state = robot_output->GetState();
  double t = robot_output->get_timestamp();
  SetPositionsAndVelocitiesIfNew<double>(plant_, robot_state, context_);

  // re-express footholds in robot yaw frame from world frame
  foothold_set.ReExpressInNewFrame(
      GetBodyYawRotation_R_WB<double>(plant_, *context_, "pelvis"));

  // initialize local variables
  int fsm_idx = static_cast<int>(
      state->get_discrete_state(fsm_state_idx_).get_value()(0));

  bool fsm_switch = false;
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
    fsm_switch = true;
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
      stance_foot_map_.at(fsm_state).first, plant_.world_frame(),
      &p_next_in_ds);

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

  ConvexFoothold workspace;
  Vector3d com_xy(CoM_b(0), CoM_b(1), p_b(2));

  workspace.AddFace( Vector3d::UnitY(),  com_xy + 10 * Vector3d::UnitY());
  workspace.AddFace(-Vector3d::UnitY(), com_xy -10 * Vector3d::UnitY());
  workspace.AddFace(Vector3d::UnitX(), com_xy + 10 * Vector3d::UnitX());
  workspace.AddFace(-Vector3d::UnitX(), com_xy - 10 * Vector3d::UnitX());

  state->get_mutable_discrete_state(fsm_state_idx_).set_value(
      fsm_idx*VectorXd::Ones(1));
  state->get_mutable_discrete_state(next_impact_time_state_idx_).set_value(
      (t_next_impact) * VectorXd::Ones(1));
  state->get_mutable_discrete_state(prev_impact_time_state_idx_).set_value(
      t_prev_impact * VectorXd::Ones(1));

  return drake::systems::EventStatus::Succeeded();
}

void AlipDeadbeatFootstepController::CopyNextFootstepOutput(
    const Context<double> &context, BasicVector<double> *p_B_FC) const {
  // TODO: Make footstep discrete state
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