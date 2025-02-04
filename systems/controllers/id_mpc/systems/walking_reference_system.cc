#include "walking_reference_system.h"
#include "systems/framework/output_vector.h"

namespace dairlib::systems::controllers::id_mpc {

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
}

fsm_info WalkingReferenceSystem::CalcFSM(
    double t, const fsm_info &curr_fsm) const {
  fsm_info fsm = curr_fsm;

  bool is_double = (curr_fsm.state == fsm_info::kPostRightDouble ||
                    curr_fsm.state == fsm_info::kPostLeftDouble);

  if (t >= curr_fsm.next_switch_time) {
    fsm.prev_switch_time = t;
    fsm.next_switch_time = is_double ? t + params_.t_ss : t + params_.t_ds;
    fsm.state = fsm_info::next_fsm(curr_fsm.state);
  }
  return fsm;
}

EventStatus WalkingReferenceSystem::UnrestrictedUpdate(
    const Context<double> &context, State<double> *state) const {

  const auto& robot_output =
      get_input_port_state().Eval<OutputVector<double>>(context);

  double t = robot_output.get_timestamp();
  const auto& fsm = state->get_abstract_state<fsm_info>(fsm_info_idx_);
  fsm_info next_fsm = CalcFSM(t, fsm);

  auto& mpc_reference = state->get_mutable_abstract_state<MPCReference>(reference_state_idx_);

  CalcReference(t, next_fsm, &mpc_reference);
  state->get_mutable_abstract_state<fsm_info>(fsm_info_idx_) = next_fsm;

  return EventStatus::Succeeded();
}

void WalkingReferenceSystem::CalcReference(
    double t, const fsm_info &fsm, MPCReference *mpc_reference) const {
  
}

}