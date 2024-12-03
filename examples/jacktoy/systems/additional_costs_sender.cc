#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

#include "drake/systems/framework/leaf_system.h"

#include "examples/jacktoy/systems/additional_costs_sender.h"
#include <iostream>

namespace dairlib {
namespace systems {

AdditionalCostsSender::AdditionalCostsSender(std::string name) {
  if(name != "sample_cost_sender")
    this->set_name(name);

  additional_costs_input_port_ = this->DeclareAbstractInputPort(
          "additional_costs_input",
          drake::Value<std::vector<double>>{})
      .get_index();

  additional_costs_output_port_ = this->DeclareAbstractOutputPort(
          "additional_costs_output",
          dairlib::lcmt_timestamped_saved_traj(),
          &AdditionalCostsSender::OutputAdditionalCosts)
      .get_index();
}

void AdditionalCostsSender::OutputAdditionalCosts(
        const drake::systems::Context<double>& context,
        dairlib::lcmt_timestamped_saved_traj* output_costs) const {
  
	// Evaluate input port to get the sample locations
  const std::vector<double>& additional_costs_vector =
      *this->EvalInputValue<std::vector<double>>(
                                context, additional_costs_input_port_
                                );

  // std::cout<<"additional_costs_vector.size() = "<<additional_costs_vector.size()<<std::endl;
  // Create a matrix of sample costs
  Eigen::MatrixXd additional_costs_datapoints = 
      Eigen::MatrixXd::Zero(1, additional_costs_vector.size());
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(additional_costs_vector.size());

  for (int i = 0; i < additional_costs_vector.size(); i++) {
    additional_costs_datapoints(0, i) = additional_costs_vector[i];
    timestamps(i) = context.get_time();
  }
  std::cout<<"additional_costs_datapoints rows = "<<additional_costs_datapoints.rows()<<std::endl;
  std::cout<<"additional_costs_datapoints cols = "<<additional_costs_datapoints.cols()<<std::endl;

  LcmTrajectory::Trajectory additional_costs;
  additional_costs.traj_name = "additional_costs";
  additional_costs.datatypes = std::vector<std::string>(1, "double");
  additional_costs.datapoints = additional_costs_datapoints;
  additional_costs.time_vector = timestamps.cast<double>();

  LcmTrajectory cost_traj({additional_costs}, {"additional_costs"},
                          "additional_costs",
                          "additional_costs", false);

	// Output the sample costs
  output_costs->saved_traj = cost_traj.GenerateLcmObject();
  output_costs->utime = context.get_time() * 1e6;
}

}  // namespace systems
}  // namespace dairlib