#include "timed_gate.h"

#include <iostream>
#include <utility>

#include "common/find_resource.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "multibody/multibody_utils.h"
namespace dairlib {

TimedGate::TimedGate(double time_to_switch) : 
  time_to_switch_(time_to_switch + 3.0) { // 3.0 delay aligned with ic3_trajectory_generator
  this->set_name("timed_gate");

  c3_actor_port_ =
      this->DeclareAbstractInputPort("c3_actor",
                                     drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

	c3_object_port_ =
      this->DeclareAbstractInputPort("c3_object",
                                     drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  
  ic3_actor_port_ =
      this->DeclareAbstractInputPort("ic3_actor",
                                     drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

	ic3_object_port_ =
      this->DeclareAbstractInputPort("ic3_object",
                                     drake::Value<lcmt_timestamped_saved_traj>())
          .get_index();

  actor_output_port =
      this->DeclareAbstractOutputPort(
              "actor_trajectory_output",
              dairlib::lcmt_timestamped_saved_traj(),
              &TimedGate::OutputActorTrajectory)
          .get_index();

  object_output_port =
      this->DeclareAbstractOutputPort(
              "object_trajectory_output",
              dairlib::lcmt_timestamped_saved_traj(),
              &TimedGate::OutputObjectTrajectory)
          .get_index();

	this->DeclarePerStepDiscreteUpdateEvent(
			&TimedGate::SetFirstCallTime);

	t0_idx_ = this->DeclareDiscreteState(1); // first time output called
	called_ = false;

}

void TimedGate::OutputActorTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {

  double t0 = context.get_discrete_state(t0_idx_).GetAtIndex(0);
  
  const auto* c3_actor_input_lcm = this->EvalAbstractInput(context, c3_actor_port_);
	const auto* ic3_actor_input_lcm = this->EvalAbstractInput(context, ic3_actor_port_);
  
	const auto& c3_actor_input = c3_actor_input_lcm->get_value<lcmt_timestamped_saved_traj>();
  const auto& ic3_actor_input = ic3_actor_input_lcm->get_value<lcmt_timestamped_saved_traj>();
  
  double time = context.get_time() - t0;


  if (time < time_to_switch_) {

    if (time < 3.0) {
      std::cout << "Not tracking, time: " << time << std::endl;
    } else {
      std::cout << "tracking iC3, time: " << time << std::endl;
    }

    *output_traj = ic3_actor_input;
  } else {
    std::cout << "tracking C3, time: " << time << std::endl;
    *output_traj = c3_actor_input;
  }

}

void TimedGate::OutputObjectTrajectory(
    const drake::systems::Context<double>& context,
    dairlib::lcmt_timestamped_saved_traj* output_traj) const {
  double t0 = context.get_discrete_state(t0_idx_).GetAtIndex(0);
  
  const auto* c3_object_input_lcm = this->EvalAbstractInput(context, c3_object_port_);
	const auto* ic3_object_input_lcm = this->EvalAbstractInput(context, ic3_object_port_);

	const auto& c3_object_input = c3_object_input_lcm->get_value<lcmt_timestamped_saved_traj>();
  const auto& ic3_object_input = ic3_object_input_lcm->get_value<lcmt_timestamped_saved_traj>();

  if (context.get_time() - t0 < time_to_switch_) {
    *output_traj = ic3_object_input;
  } else {
    *output_traj = c3_object_input;
  }

}

drake::systems::EventStatus TimedGate::SetFirstCallTime(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  auto& vec = discrete_state->get_mutable_vector(t0_idx_);
  if (!called_) {  
    vec.SetAtIndex(0, context.get_time());
		called_ = true;
  }
  return drake::systems::EventStatus::Succeeded();
}

} // namespace dairlib