#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

#include "drake/systems/framework/leaf_system.h"

#include "examples/jacktoy/systems/sample_location_sender.h"
#include <iostream>

namespace dairlib {
namespace systems {

SampleLocationSender::SampleLocationSender() {
  this->set_name("sample_location_sender");

  std::vector<Eigen::Vector3d> sample_locations;

  sample_locations_input_port_ = this->DeclareAbstractInputPort(
          "sample_locations_input",
          drake::Value<std::vector<Eigen::Vector3d>>{sample_locations})
      .get_index();

  sample_locations_output_port_ = this->DeclareAbstractOutputPort(
          "sample_locations_output",
          dairlib::lcmt_timestamped_saved_traj(),
          &SampleLocationSender::OutputSampleLocations)
      .get_index();
}

void SampleLocationSender::OutputSampleLocations(
        const drake::systems::Context<double>& context,
        dairlib::lcmt_timestamped_saved_traj* output_traj) const {
  
	// Evaluate input port to get the sample locations
  const auto sample_locations =
      *this->EvalInputValue<std::vector<Eigen::Vector3d>>(
                                context, sample_locations_input_port_
                                );

  // Create a matrix of sample locations
	Eigen::MatrixXd sample_datapoints = Eigen::MatrixXd::Zero(3, sample_locations.size());
	Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(sample_locations.size());

	for (int i = 0; i < sample_locations.size(); i++) {
		sample_datapoints.col(i) = sample_locations[i];
		timestamps(i) = context.get_time();
	}
	
	LcmTrajectory::Trajectory sample_positions;
	sample_positions.traj_name = "sample_locations";
	sample_positions.datatypes = std::vector<std::string>(3, "double");
	sample_positions.datapoints = sample_datapoints;
	sample_positions.time_vector = timestamps.cast<double>();

	LcmTrajectory sample_traj({sample_positions}, {"sample_locations"},
													"sample_locations",
													"sample_locations", false);
	
	// Output the sample locations
  output_traj->saved_traj = sample_traj.GenerateLcmObject();
  output_traj->utime = context.get_time() * 1e6;
}

}  // namespace systems
}  // namespace dairlib