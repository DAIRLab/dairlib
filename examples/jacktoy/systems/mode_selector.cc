#include "examples/jacktoy/systems/mode_selector.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"

namespace dairlib {
namespace systems {

ModeSelector::ModeSelector() {
  this->set_name("mode_selector");

  // Boolean select line
  is_c3_mode_input_port_ = this->DeclareVectorInputPort(
        "is_c3_mode", drake::systems::BasicVector<double>(1))
        .get_index();

  c3_actor_trajectory_port_ =
      this->DeclareAbstractInputPort(
              "c3_actor_trajectory_input",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();
  c3_object_trajectory_port_ =
      this->DeclareAbstractInputPort(
              "c3_object_trajectory_input",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  repositioning_actor_trajectory_port_ =
      this->DeclareAbstractInputPort(
              "repositioning_actor_trajectory_input",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();
  repositioning_object_trajectory_port_ =
      this->DeclareAbstractInputPort(
              "repositioning_object_trajectory_input",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
          .get_index();

  tracking_actor_trajectory_port_ =
        this->DeclareAbstractOutputPort(
                "actor_tracking_trajectory_output",
                dairlib::lcmt_timestamped_saved_traj(),
                &ModeSelector::OutputActorTrackingTrajectory)
            .get_index();
  
  tracking_object_trajectory_port_ =
        this->DeclareAbstractOutputPort(
                "object_tracking_trajectory_output",
                dairlib::lcmt_timestamped_saved_traj(),
                &ModeSelector::OutputObjectTrackingTrajectory)
            .get_index();

}

void ModeSelector::OutputActorTrackingTrajectory(
        const drake::systems::Context<double>& context,
        dairlib::lcmt_timestamped_saved_traj* output_traj) const {
  // Get the select line
  const bool c3_select = 
        this->EvalInputValue<bool>(context, is_c3_mode_input_port_);

  // Get the c3 trajectories
  const dairlib::lcmt_timestamped_saved_traj& c3_actor_traj = 
        this->EvalAbstractInput(context, c3_actor_trajectory_port_)
            ->get_value<dairlib::lcmt_timestamped_saved_traj>();

  // Get the repositioning trajectories
  const dairlib::lcmt_timestamped_saved_traj& repositioning_actor_traj = 
        this->EvalAbstractInput(context, repositioning_actor_trajectory_port_)
            ->get_value<dairlib::lcmt_timestamped_saved_traj>();

  // If select line is true, output the c3 trajectories
  if (c3_select) {
    *output_traj = c3_actor_traj;
  } else {
    *output_traj = repositioning_actor_traj;
  }
}

void ModeSelector::OutputObjectTrackingTrajectory(
        const drake::systems::Context<double>& context,
        dairlib::lcmt_timestamped_saved_traj* output_traj) const {
  // Get the select line
  const bool c3_select = 
        this->EvalInputValue<bool>(context, is_c3_mode_input_port_);

  // Get the c3 trajectories
  const dairlib::lcmt_timestamped_saved_traj& c3_object_traj = 
        this->EvalAbstractInput(context, c3_object_trajectory_port_)
            ->get_value<dairlib::lcmt_timestamped_saved_traj>();

  // Get the repositioning trajectories
  const dairlib::lcmt_timestamped_saved_traj& repositioning_object_traj = 
        this->EvalAbstractInput(context, repositioning_object_trajectory_port_)
            ->get_value<dairlib::lcmt_timestamped_saved_traj>();

  // If select line is true, output the c3 trajectories
  if (c3_select) {
    *output_traj = c3_object_traj;
  } else {
    *output_traj = repositioning_object_traj;
  }
}

}  // namespace systems
}  // namespace dairlib