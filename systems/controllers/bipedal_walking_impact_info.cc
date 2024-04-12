#include "bipedal_walking_impact_info.h"

#include "common/blending_utils.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/impact_info_vector.h"

namespace dairlib {
namespace systems {
namespace controllers {

using drake::systems::Context;

BipedalWalkingImpactInfo::BipedalWalkingImpactInfo(
    const drake::multibody::MultibodyPlant<double> &plant,
    std::vector<int> left_right_fsm_states,
    std::vector<int> post_left_right_fsm_states) {

  input_port_state_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(plant)
  ).get_index();
  input_port_prev_switch_time_ = DeclareVectorInputPort(
      "prev_switch_time", 1
  ).get_index();
  input_port_next_switch_time_ = DeclareVectorInputPort(
      "next_switch_time", 1
  ).get_index();
  input_port_fsm_ = DeclareVectorInputPort("fsm", 1).get_index();

  DeclareVectorOutputPort(
      "impact info",
      ImpactInfoVector<double>(0,0,0),
      &BipedalWalkingImpactInfo::CalcImpactInfo
  );

  for (int i = 0; i < left_right_fsm_states.size(); ++i) {
    const auto& state = left_right_fsm_states.at(i);
    state_begins_with_impact_.insert({state, false});
    state_ends_with_impact_.insert({state, true});
    post_impact_state_.insert({state, post_left_right_fsm_states.at(i)});
  }
  for (const auto& state: post_left_right_fsm_states) {
    state_begins_with_impact_.insert({state, true});
    state_ends_with_impact_.insert({state, false});
    post_impact_state_.insert({state, state});
  }

}

void BipedalWalkingImpactInfo::CalcImpactInfo(
    const Context<double> &context, ImpactInfoVector<double> *impact) const {

  auto robot_output = dynamic_cast<const OutputVector<double>*>(
      EvalVectorInput(context, input_port_state_));
  double timestamp = robot_output->get_timestamp();

  impact->set_timestamp(timestamp);
  impact->SetAlpha(0);

  int fsm = EvalVectorInput(context, input_port_fsm_)->get_value()(0);
  double t_impact = -1;

  if (state_ends_with_impact_.at(fsm)) {
    t_impact = EvalVectorInput(
        context, input_port_next_switch_time_)->get_value()(0);
  } else if (state_begins_with_impact_.at(fsm)) {
    t_impact = EvalVectorInput(
        context, input_port_prev_switch_time_)->get_value()(0);

  }

  if (abs(timestamp - t_impact) < window_) {
    impact->SetAlpha(
        blend_sigmoid(t_impact - timestamp, tau_, window_));
  }

  impact->SetCurrentContactMode(post_impact_state_.at(fsm));
}

}
}
}