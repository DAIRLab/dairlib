#include "alip_minlp_footstep_controller.h"
#include "systems/framework/output_vector.h"
#include "multibody/multibody_utils.h"

namespace dairlib::systems::controllers {

using geometry::ConvexFoothold;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

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
}

// TODO: express footholds and alip state in pelvis yaw frame
drake::systems::EventStatus AlipMINLPFootstepController::UnrestrictedUpdate(
    const Context<double> &context, State<double> *state) const {
  auto start = std::chrono::high_resolution_clock::now();
  // First, evaluate the output ports
  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));
  const Vector2d vdes =
      this->EvalVectorInput(context, vdes_input_port_)->get_value();
  const std::vector<ConvexFoothold> footholds =
      this->EvalAbstractInput(context, foothold_input_port_)
      ->get_value<std::vector<ConvexFoothold>>();

  double t = robot_output->get_timestamp();
  double t_next_impact =
      state->get_discrete_state(next_impact_time_state_idx_).get_value()(0);
  if (t_next_impact == 0.0) { t_next_impact = t + stance_duration_map_.at(0);}
  double t_prev_impact =
      state->get_discrete_state(prev_impact_time_state_idx_).get_value()(0);
  if (t_prev_impact == 0.0) { t_prev_impact = t; }

  auto& trajopt =
      state->get_mutable_abstract_state<AlipMINLP>(alip_minlp_index_);
  trajopt.ChangeFootholds(footholds);

  int fsm_idx = static_cast<int>(
      state->get_discrete_state(fsm_state_idx_).get_value()(0));

  bool warmstart = true;
  if (t >= t_next_impact) {
    warmstart = false;
    std::cout << "updating fsm" << std::endl;
    trajopt.DeactivateInitialTimeConstraint();
    fsm_idx ++;
    fsm_idx = fsm_idx >= left_right_stance_fsm_states_.size() ? 0 : fsm_idx;
    state->get_mutable_discrete_state(fsm_state_idx_).set_value(fsm_idx*VectorXd::Ones(1));
    state->get_mutable_discrete_state(prev_impact_time_state_idx_).set_value(t * VectorXd::Ones(1));
    t_prev_impact = t;
    t_next_impact = t + stance_duration_map_.at(left_right_stance_fsm_states_.at(fsm_idx));
  } else if ((t_next_impact - t) < gains_.T_min_until_touchdown) {
    trajopt.ActivateInitialTimeConstraint(t_next_impact - t);
  }
  int stance = left_right_stance_fsm_states_.at(fsm_idx) == 0? -1 : 1;

  Vector3d CoM_w, p_w, L;
  alip_utils::CalcAlipState(
      plant_, context_, robot_output->GetState(),
      {stance_foot_map_.at(left_right_stance_fsm_states_.at(fsm_idx))},
      &CoM_w, &L, &p_w);

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

  double t0 = trajopt.GetTimingSolution()(0);
  state->get_mutable_discrete_state(next_impact_time_state_idx_).set_value(
      (t + t0) * VectorXd::Ones(1));

  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "solve time: " << elapsed.count() << std::endl;
  return drake::systems::EventStatus::Succeeded();
}

void AlipMINLPFootstepController::CopyNextFootstepOutput(
    const Context<double> &context, BasicVector<double> *p_B_FC) const {
  auto& trajopt = context.get_abstract_state<AlipMINLP>(alip_minlp_index_);
  p_B_FC->set_value(trajopt.GetFootstepSolution().at(1));
}

void AlipMINLPFootstepController::CopyCoMTrajOutput(const drake::systems::Context<
    double> &context, lcmt_saved_traj *traj_msg) const {

}

}