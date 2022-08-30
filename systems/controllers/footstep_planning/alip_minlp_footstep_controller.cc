#include "alip_minlp_footstep_controller.h"

namespace dairlib::systems::controllers {

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

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

  // TODO: @Brian-Acosta Add double stance here when appropriate
  for (int i = 0; i < left_right_stance_fsm_states_.size(); i++){
    stance_duration_map_[i] = left_right_stance_durations.at(i);
  }

  // Must declare the discrete states before assigning their output ports so
  // that the indexes can be used to call DeclareStateOutputPort
  fsm_state_idx_ = DeclareDiscreteState(1);
  next_impact_time_state_idx_ = DeclareDiscreteState(1);
  prev_impact_time_state_idx_ = DeclareDiscreteState(1);

  AlipMINLP alip_trajopt(plant_.CalcTotalMass(*context_), gains_.hdes);


  fsm_output_port_ = DeclareStateOutputPort("fsm", fsm_state_idx_).get_index();
  next_impact_time_output_port_ = DeclareStateOutputPort("t_next", next_impact_time_state_idx_).get_index();
  prev_impact_time_output_port_ = DeclareStateOutputPort("t_prev", prev_impact_time_state_idx_).get_index();



}


}