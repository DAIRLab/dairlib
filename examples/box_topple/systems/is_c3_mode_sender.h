#include "drake/systems/framework/leaf_system.h"
#include <vector>
#include "Eigen/Dense"
#include "Eigen/Core"

namespace dairlib {
namespace systems {

class IsC3ModeSender: public drake::systems::LeafSystem<double> {
	// This is a system that reads the all_sample_costs output of 
	// std::vector<Eigen::vector3d> from the sampling controller and outputs it as an lcm message.
 public:
  IsC3ModeSender();

  // Input ports
  const drake::systems::InputPort<double>& get_input_port_is_c3_mode() const {
    return this->get_input_port(is_c3_mode_input_port_);
  }

  // Output port
  const drake::systems::OutputPort<double>& get_output_port_is_c3_mode() const {
    return this->get_output_port(is_c3_mode_output_port_);
  }

 private:
  void OutputIsC3Mode(
        const drake::systems::Context<double>& context,
        dairlib::lcmt_timestamped_saved_traj* output_is_c3_mode) const;

  drake::systems::InputPortIndex is_c3_mode_input_port_;
  drake::systems::OutputPortIndex is_c3_mode_output_port_;
};

}  // namespace systems
}  // namespace dairlib
