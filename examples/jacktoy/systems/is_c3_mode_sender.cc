#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

#include "drake/systems/framework/leaf_system.h"

#include "examples/jacktoy/systems/is_c3_mode_sender.h"
#include <iostream>

namespace dairlib {
namespace systems {

IsC3ModeSender::IsC3ModeSender() {
  this->set_name("is_c3_mode_sender");

  is_c3_mode_input_port_ = this->DeclareAbstractInputPort(
          "is_c3_mode_input",
          bool())
      .get_index();

  is_c3_mode_output_port_ = this->DeclareAbstractOutputPort(
          "is_c3_mode_output",
          TimestampedVector<bool>(),
          &IsC3ModeSender::OutputIsC3Mode)
      .get_index();
}

void IsC3ModeSender::OutputIsC3Mode(
        const drake::systems::Context<double>& context,
        TimestampedVector<double>* output_costs) const {
  
	// Evaluate input port to get the sample locations
  const bool& is_c3_mode =
      this->EvalInputValue<std::vector<double>>(
                                context, is_c3_mode_input_port_
                                ).value();

	// Output the sample costs
  output_costs->SetDataVector(is_c3_mode);
  output_costs->set_timestamp(context.get_time() * 1e6);
}

}  // namespace systems
}  // namespace dairlib