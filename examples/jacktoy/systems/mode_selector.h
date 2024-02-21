// This is a system that takes in two pairs of possible c3 trajectories along 
// with the mode_selection binary input based on which it selects one of the 
// two trajectories to output. 
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

class is_c3_mode_ModeSelector: public drake::systems::LeafSystem<double> {
 public:
  ModeSelector();

  // Input ports
  const drake::systems::InputPort<double>& get_input_port_is_c3_mode() const {
    return this->get_input_port(is_c3_mode_);
  }

  const drake::systems::InputPort<double>& get_input_port_c3_actor_trajectory() 
    const {return this->get_input_port(c3_actor_trajectory_port_);}

  const drake::systems::InputPort<double>& get_input_port_c3_object_trajectory() 
    const {return this->get_input_port(c3_object_trajectory_port_);}

  const drake::systems::InputPort<double>& get_input_port_repositioning_actor_trajectory() 
    const {return this->get_input_port(repositioning_actor_trajectory_port_);}

  const drake::systems::InputPort<double>& get_input_port_repositioning_object_trajectory() 
    const {return this->get_input_port(repositioning_object_trajectory_port_);}

  // Selected trajectory output port
  const drake::systems::OutputPort<double>& get_output_port_actor_tracking_trajectory()
      const {return this->get_output_port(tracking_actor_trajectory_port_);}

  const drake::systems::OutputPort<double>& get_output_port_object_tracking_trajectory()
      const {return this->get_output_port(tracking_object_trajectory_port_);}

  private:
  void OutputActorTrackingTrajectory(        
        const drake::systems::Context<double>& context,
        dairlib::lcmt_timestamped_saved_traj* output_traj) const;

  void OutputObjectTrackingTrajectory(
        const drake::systems::Context<double>& context,
        dairlib::lcmt_timestamped_saved_traj* output_traj) const;

  drake::systems::InputPortIndex is_c3_mode_;
  drake::systems::InputPortIndex c3_actor_trajectory_port_;
  drake::systems::InputPortIndex c3_object_trajectory_port_;
  drake::systems::InputPortIndex repositioning_actor_trajectory_port_;
  drake::systems::InputPortIndex repositioning_object_trajectory_port_;
  drake::systems::OutputPortIndex tracking_trajectory_port_;
  drake::systems::OutputPortIndex tracking_actor_trajectory_port_;
  drake::systems::OutputPortIndex tracking_object_trajectory_port_;
};

}  // namespace systems
}  // namespace dairlib