#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

#include "drake/systems/framework/leaf_system.h"

#include "examples/jacktoy/systems/sample_cost_sender.h"
#include <iostream>

namespace dairlib {
namespace systems {

SampleCostSender::SampleCostSender() {
  this->set_name("sample_cost_sender");

  sample_costs_input_port_ = this->DeclareAbstractInputPort(
          "sample_costs_input",
          std::vector<double>())
      .get_index();

  sample_costs_output_port_ = this->DeclareAbstractOutputPort(
          "sample_costs_output",
          TimestampedVector<double>(),
          &SampleCostSender::OutputSampleCosts)
      .get_index();
}

void SampleCostSender::OutputSampleCosts(
        const drake::systems::Context<double>& context,
        TimestampedVector<double>* output_costs) const {
  
	// Evaluate input port to get the sample locations
  const std::vector<double>& sample_costs =
      this->EvalInputValue<std::vector<double>>(
                                context, sample_costs_input_port_
                                ).value();

	// Output the sample costs
  output_costs->SetDataVector(sample_costs);
  output_costs->set_timestamp(context.get_time() * 1e6);
}

}  // namespace systems
}  // namespace dairlib