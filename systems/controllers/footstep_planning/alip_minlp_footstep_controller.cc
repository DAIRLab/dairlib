#include "alip_minlp_footstep_controller.h"
#include "systems/framework/output_vector.h"

namespace dairlib::systems::controllers {

using geometry::ConvexFoothold;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using drake::AbstractValue;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::State;
using drake::systems::DiscreteState;
using drake::systemms::AbstractState;

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
  nu_ = plant.num_actuators();

  // TODO: @Brian-Acosta Add double stance here when appropriate
  for (int i = 0; i < left_right_stance_fsm_states_.size(); i++){
    stance_duration_map_[i] = left_right_stance_durations.at(i);
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
  trajopt.Build();
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
      "p_SW", 3, &AlipMINLPFootstepController::CopyNextFootstepOutput,
      {abstract_state_ticket(alip_minlp_index_)})
      .get_index();
  com_traj_output_port_ = DeclareAbstractOutputPort(
      "lcmt_saved_traj", &AlipMINLPFootstepController::CopyCoMTrajOutput,
      {abstract_state_ticket(alip_minlp_index_)})
      .get_index();
}

drake::systems::EventStatus AlipMINLPFootstepController::UnrestrictedUpdate(
    const Context<double> &context, State<double> *state) const {

}


}