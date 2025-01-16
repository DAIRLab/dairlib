#include "drake/systems/framework/leaf_system.h"
#include <vector>
#include "Eigen/Dense"
#include "Eigen/Core"

namespace dairlib {
namespace systems {

class C3ModeVisualizer: public drake::systems::LeafSystem<double> {
	// This is a system that reads the is_c3_mode over lcm along with the current 
  // lcs state of the system and outputs a trajectory for visualization.
  // This trajectory is a single 3d point at the current location of the end
  // effector if we are in C3 mode. It is a single point at the origin if we are
  // not in C3 mode.
 public:
  C3ModeVisualizer();

  // Input ports
  const drake::systems::InputPort<double>& get_input_port_is_c3_mode() const {
    return this->get_input_port(is_c3_mode_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_curr_lcs_state() const {
    return this->get_input_port(curr_lcs_state_);
  }

  // Output port
  const drake::systems::OutputPort<double>& get_output_port_c3_mode_visualization_traj() const {
    return this->get_output_port(c3_mode_visualization_traj_port_);
  }

 private:
  void OutputC3ModeVisualization(
        const drake::systems::Context<double>& context,
         dairlib::lcmt_timestamped_saved_traj* c3_mode_visualization_traj) const;

  drake::systems::InputPortIndex is_c3_mode_input_port_;
  drake::systems::InputPortIndex curr_lcs_state_;
  drake::systems::OutputPortIndex c3_mode_visualization_traj_port_;
};

}  // namespace systems
}  // namespace dairlib
