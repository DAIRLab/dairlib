#include "alip_one_step_footstep_controller.h"
#include "common/eigen_utils.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include <iostream>

namespace dairlib::systems::controllers {

using multibody::ReExpressWorldVector3InBodyYawFrame;
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

AlipOneStepFootstepController::AlipOneStepFootstepController(
    const MultibodyPlant<double>& plant, Context<double>* plant_context,
    std::vector<int> left_right_stance_fsm_states,
    std::vector<int> post_left_right_fsm_states,
    std::vector<double> left_right_stance_durations,
    double double_stance_duration,
    double stance_width,
    std::vector<PointOnFramed> left_right_foot) :
    plant_(plant),
    context_(plant_context),
    m_(plant.CalcTotalMass(*plant_context)),
    left_right_stance_fsm_states_(left_right_stance_fsm_states),
    post_left_right_fsm_states_(post_left_right_fsm_states),
    double_stance_duration_(double_stance_duration),
    single_stance_duration_(left_right_stance_durations.front()),
    stance_width_(stance_width) {

  // just alternating single stance phases for now.
  DRAKE_DEMAND(left_right_stance_fsm_states_.size() == 2);
  DRAKE_DEMAND(left_right_stance_durations.size() == 2);
  DRAKE_DEMAND(left_right_foot.size() == 2);

  for (int i = 0; i < left_right_stance_fsm_states_.size(); i++){
    stance_foot_map_.insert(
        {left_right_stance_fsm_states_.at(i), left_right_foot.at(i)});
    stance_foot_map_.insert(
        {post_left_right_fsm_states_.at(i), left_right_foot.at(i)});
  }

  // Must declare the discrete states before assigning their output ports so
  // that the indexes can be used to call DeclareStateOutputPort
  fsm_state_idx_ = DeclareDiscreteState(1);
  next_impact_time_state_idx_ = DeclareDiscreteState(1);
  prev_impact_time_state_idx_ = DeclareDiscreteState(1);
  footstep_target_state_idx_ = DeclareDiscreteState(3);

  // State Update
  this->DeclarePerStepUnrestrictedUpdateEvent(
      &AlipOneStepFootstepController::UnrestrictedUpdate);

  // Input ports
  state_input_port_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(plant)
  ).get_index();
  vdes_input_port_ = DeclareVectorInputPort("vdes_x_y", 2).get_index();

  // output ports
  next_impact_time_output_port_ = DeclareStateOutputPort(
      "t_next", next_impact_time_state_idx_
    ).get_index();
  prev_impact_time_output_port_ = DeclareStateOutputPort(
      "t_prev", prev_impact_time_state_idx_
    ).get_index();
  footstep_target_output_port_ = DeclareStateOutputPort(
      "p_SW", footstep_target_state_idx_
  ).get_index();
  fsm_output_port_ = DeclareStateOutputPort(
      "fsm", fsm_state_idx_
  ).get_index();

}

drake::systems::EventStatus AlipOneStepFootstepController::UnrestrictedUpdate(
    const Context<double> &context, State<double> *state) const {

  // evaluate input ports
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));
  const Vector2d& vdes =
      this->EvalVectorInput(context, vdes_input_port_)->get_value();



  // get state and time from robot_output, set plant context
  const VectorXd robot_state = robot_output->GetState();
  double t = robot_output->get_timestamp();
  SetPositionsAndVelocitiesIfNew<double>(plant_, robot_state, context_);

  // declare local fsm variables
  int fsm_state;
  double t_next_impact;
  double t_prev_impact;

  const double two_stride_period =
      2 * (single_stance_duration_ + double_stance_duration_);
  const double one_stride_period =
      single_stance_duration_ + double_stance_duration_;
  double periods = t / two_stride_period;
  double remainder = periods - std::floor(periods);
  double phase = remainder * two_stride_period;
  double period_start = (periods - remainder) * two_stride_period;


  if (phase < double_stance_duration_) { // Post idx 0 double stance
    fsm_state = post_left_right_fsm_states_.back();
    t_prev_impact = period_start;
    t_next_impact = period_start + double_stance_duration_;
  } else if (phase < one_stride_period) { // idx 0 single stance
    fsm_state = left_right_stance_fsm_states_.front();
    t_prev_impact = period_start + double_stance_duration_;
    t_next_impact = period_start + one_stride_period;
  } else if (phase < one_stride_period + double_stance_duration_) { // post idx 0 double stance
    fsm_state = post_left_right_fsm_states_.front();
    t_prev_impact = period_start + one_stride_period;
    t_next_impact = period_start + one_stride_period + double_stance_duration_;
  } else { // idx 1 single stance
    fsm_state = left_right_stance_fsm_states_.back();
    t_prev_impact = period_start + one_stride_period + double_stance_duration_;
    t_next_impact = period_start + two_stride_period;
  }

  // Forward kinematics quantities
  Vector3d CoM_w = Vector3d::Zero();
  Vector3d p_w = Vector3d::Zero();
  Vector3d p_next_in_ds = Vector3d::Zero();
  Vector3d L = Vector3d::Zero();

  // get the alip state
  alip_utils::CalcAlipState(
      plant_, context_, robot_state,
      {stance_foot_map_.at(fsm_state)},
      {1.0}, &CoM_w, &L, &p_w);

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

  Vector2d footstep_target = Vector2d::Zero();

  CalcFootStepAndStanceFootHeight(
      p_b, CoM_b, L_b, vdes, t, t_next_impact, fsm_state, &footstep_target);

  auto target = state->get_mutable_discrete_state(
      footstep_target_state_idx_).get_mutable_value();
  target.head<2>() = footstep_target;
  state->get_mutable_discrete_state(fsm_state_idx_).set_value(
      fsm_state*VectorXd::Ones(1));
  state->get_mutable_discrete_state(next_impact_time_state_idx_).set_value(
      t_next_impact * VectorXd::Ones(1));
  state->get_mutable_discrete_state(prev_impact_time_state_idx_).set_value(
      t_prev_impact * VectorXd::Ones(1));

  return drake::systems::EventStatus::Succeeded();
}


void AlipOneStepFootstepController::CalcFootStepAndStanceFootHeight(
    const Vector3d& stance_foot_pos_yaw_frame,
    const Vector3d& com_pos_yaw_frame,
    const Vector3d& L_yaw_frame,
    const Vector2d& vdes_xy,
    const double curr_time,
    const double end_time_of_this_interval,
    int fsm_state,
    Vector2d* x_fs) const {

  double H = com_pos_yaw_frame(2) - stance_foot_pos_yaw_frame(2);
  double omega = sqrt(9.81 / H);
  double T = single_stance_duration_ + double_stance_duration_;
  double L_x_n = m_ * H * (stance_width_ / 2) *
      (omega * sinh(omega * T) / (1 + cosh(omega * T)));
  double L_y_des = vdes_xy(0) * H * m_;
  double L_x_offset = -vdes_xy(1) * H * m_;

  Vector4d x_alip = Vector4d::Zero();
  x_alip.head<2>() = (com_pos_yaw_frame - stance_foot_pos_yaw_frame).head<2>();
  x_alip.tail<2>() = L_yaw_frame.head<2>();
  Vector4d alip_pred =
      alip_utils::CalcAd(H, m_, end_time_of_this_interval - curr_time) * x_alip;

  bool is_right_support = (fsm_state == left_right_stance_fsm_states_[1]);

  Vector2d L_i = alip_pred.tail<2>();
  Vector2d L_f = is_right_support ?
                 Vector2d(L_x_offset + L_x_n, L_y_des) :
                 Vector2d(L_x_offset - L_x_n, L_y_des);
  double p_x_ft_to_com = ( L_f(1) - cosh(omega*T) * L_i(1) ) /
      (m_ * H * omega * sinh(omega*T));
  double p_y_ft_to_com = -( L_f(0) - cosh(omega*T) * L_i(0) ) /
      (m_ * H * omega * sinh(omega*T));
  *x_fs = Vector2d(-p_x_ft_to_com, -p_y_ft_to_com);

  /// Imposing guards
  Vector2d stance_foot_wrt_com_in_local_frame =
      multibody::ReExpressWorldVector2InBodyYawFrame<double>(
          plant_, *context_,"pelvis",
          (stance_foot_pos_yaw_frame - com_pos_yaw_frame).head<2>());

  if (is_right_support) {
    double line_pos =
        std::max(0.05, stance_foot_wrt_com_in_local_frame(1));
    (*x_fs)(1) = std::max(line_pos, (*x_fs)(1));
  } else {
    double line_pos =
        std::min(-0.05, stance_foot_wrt_com_in_local_frame(1));
    (*x_fs)(1) = std::min(line_pos, (*x_fs)(1));
  }

  // Cap by the step length
  double dist = x_fs->norm();
  if (dist > 0.55) {
    (*x_fs) = (*x_fs) * 0.55 / dist;
  }

  // get the footstep command in the stance frame
  *x_fs += alip_pred.head<2>();
}

}