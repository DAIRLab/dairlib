#include "drake/systems/framework/leaf_system.h"
#include <vector>
#include "Eigen/Dense"
#include "Eigen/Core"

namespace dairlib {
namespace systems {

class SampleLocationSender: public drake::systems::LeafSystem<double> {
	// This is a system that reads the all_sample_locations output of 
	// std::vector<Eigen::vector3d> from the sampling controller and outputs it as an lcm message.
 public:
  SampleLocationSender();

  // Input ports
  const drake::systems::InputPort<double>& get_input_port_sample_locations() const {
    return this->get_input_port(sample_locations_input_port_);
  }

  // Output port
  const drake::systems::OutputPort<double>& get_output_port_sample_locations() const {
    return this->get_output_port(sample_locations_output_port_);
  }

 private:
  void OutputSampleLocations(
        const drake::systems::Context<double>& context,
        dairlib::lcmt_timestamped_saved_traj* output_traj) const;

  drake::systems::InputPortIndex sample_locations_input_port_;
  drake::systems::OutputPortIndex sample_locations_output_port_;
};

}  // namespace systems
}  // namespace dairlib
