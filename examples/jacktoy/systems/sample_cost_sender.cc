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
          drake::Value<std::vector<double>>{})
      .get_index();

  sample_costs_output_port_ = this->DeclareAbstractOutputPort(
          "sample_costs_output",
          dairlib::lcmt_timestamped_saved_traj(),
          &SampleCostSender::OutputSampleCosts)
      .get_index();
}

void SampleCostSender::OutputSampleCosts(
        const drake::systems::Context<double>& context,
        dairlib::lcmt_timestamped_saved_traj* output_costs) const {
  
	// Evaluate input port to get the sample locations
  const std::vector<double>& sample_cost_vector =
      *this->EvalInputValue<std::vector<double>>(
                                context, sample_costs_input_port_
                                );

  // Create a matrix of sample costs
  Eigen::MatrixXd cost_datapoints = 
      Eigen::MatrixXd::Zero(1, sample_cost_vector.size());
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(sample_cost_vector.size());

  for (int i; i < sample_cost_vector.size(); i++) {
    cost_datapoints(0, i) = sample_cost_vector[i];
    timestamps(i) = context.get_time();
  }

  LcmTrajectory::Trajectory sample_costs;
  sample_costs.traj_name = "sample_costs";
  sample_costs.datatypes = std::vector<std::string>(1, "double");
  sample_costs.datapoints = cost_datapoints;
  sample_costs.time_vector = timestamps.cast<double>();

  LcmTrajectory cost_traj({sample_costs}, {"sample_costs"},
                          "sample_costs",
                          "sample_costs", false);

	// Output the sample costs
  output_costs->saved_traj = cost_traj.GenerateLcmObject();
  output_costs->utime = context.get_time() * 1e6;
}

}  // namespace systems
}  // namespace dairlib