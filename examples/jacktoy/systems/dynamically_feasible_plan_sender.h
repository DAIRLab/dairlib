#include "drake/systems/framework/leaf_system.h"
#include <vector>
#include "Eigen/Dense"
#include "Eigen/Core"

namespace dairlib {
namespace systems {

class DynamicallyFeasiblePlanSender: public drake::systems::LeafSystem<double> {
	// This is a system that reads the all_sample_locations output of 
	// std::vector<Eigen::vector3d> from the sampling controller and outputs it as an lcm message.
 public:
  DynamicallyFeasiblePlanSender(std::string name);

  // Input ports
  const drake::systems::InputPort<double>& get_input_port_dynamically_feasible_plan() const {
    return this->get_input_port(dynamically_feasible_plan_input_port_);
  }

  // Output port
  const drake::systems::OutputPort<double>& get_output_port_dynamically_feasible_plan() const {
    return this->get_output_port(dynamically_feasible_plan_output_port_);
  }

 private:
  void OutputDynamicallyFeasiblePlan(
        const drake::systems::Context<double>& context,
        dairlib::lcmt_timestamped_saved_traj* output_traj) const;

  drake::systems::InputPortIndex dynamically_feasible_plan_input_port_;
  drake::systems::OutputPortIndex dynamically_feasible_plan_output_port_;
};

}  // namespace systems
}  // namespace dairlib
