#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// A Drake system in the visualizer diagram for outputting a trajectory
/// (lcmt_timestamped_saved_traj) based on the current C3 mode.  When in C3
/// mode, the trajectory follows the EE location (to highlight the EE in pink).
/// When not in C3 mode, the trajectory is at the origin (to hide the pink EE in
/// the visualizer).
class C3ModeVisualizer : public drake::systems::LeafSystem<double> {
 public:
  explicit C3ModeVisualizer(
      const drake::multibody::MultibodyPlant<double>& plant);

  // Input ports
  const drake::systems::InputPort<double>& get_input_port_is_c3_mode() const {
    return this->get_input_port(is_c3_mode_input_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_curr_lcs_state()
      const {
    return this->get_input_port(curr_lcs_state_);
  }

  // Output port
  const drake::systems::OutputPort<double>&
  get_output_port_c3_mode_visualization_traj() const {
    return this->get_output_port(c3_mode_visualization_traj_port_);
  }

 private:
  void OutputC3ModeVisualization(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_timestamped_saved_traj* c3_mode_visualization_traj) const;

  drake::systems::InputPortIndex is_c3_mode_input_port_;
  drake::systems::InputPortIndex curr_lcs_state_;
  drake::systems::OutputPortIndex c3_mode_visualization_traj_port_;
  const drake::multibody::MultibodyPlant<double>& plant_;
};

}  // namespace systems
}  // namespace dairlib
