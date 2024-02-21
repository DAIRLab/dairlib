#include "drake/systems/framework/leaf_system.h"
#include <vector>
#include "Eigen/Dense"
#include "Eigen/Core"

namespace dairlib {
namespace systems {

class SampleCostSender: public drake::systems::LeafSystem<double> {
	// This is a system that reads the all_sample_costs output of 
	// std::vector<Eigen::vector3d> from the sampling controller and outputs it as an lcm message.
 public:
  SampleCostSender();

  // Input ports
  const drake::systems::InputPort<double>& get_input_port_sample_costs() const {
    return this->get_input_port(sample_costs_input_port_);
  }

  // Output port
  const drake::systems::OutputPort<double>& get_output_port_sample_costs() const {
    return this->get_output_port(sample_costs_output_port_);
  }

 private:
  void OutputSampleCosts(
        const drake::systems::Context<double>& context,
        TimestampedVector<double>* output_costs) const;

  drake::systems::InputPortIndex sample_costs_input_port_;
  drake::systems::OutputPortIndex sample_costs_output_port_;
};

}  // namespace systems
}  // namespace dairlib
