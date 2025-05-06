#include <iostream>
#include "systems/sender_systems/build_functions.h"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

namespace dairlib {
namespace systems {

// Builds the c3 mode sender message.
void BuildC3ModeSender(const Eigen::VectorXd& is_c3_mode, dairlib::lcmt_timestamped_saved_traj* output) {
  // Create a matrix of sample costs
  Eigen::MatrixXd c3_mode_data = Eigen::MatrixXd::Zero(1, 1);
  Eigen::VectorXd timestamp = Eigen::VectorXd::Zero(1);

  // Reading the boolean value into the matrix.
  c3_mode_data(0, 0) = is_c3_mode(0); 
  // This timestamp corresponds to the trajectory object.
  timestamp(0) = is_c3_mode(1);

  LcmTrajectory::Trajectory c3_mode;
  c3_mode.traj_name = "is_c3_mode";
  c3_mode.datatypes = std::vector<std::string>(1, "bool");
  c3_mode.datapoints = c3_mode_data;
  c3_mode.time_vector = timestamp.cast<double>();

  LcmTrajectory c3_mode_traj({c3_mode}, {"is_c3_mode"},
                            "is_c3_mode",
                            "is_c3_mode", false);

  // Output the mode as an lcm message
  output->saved_traj = c3_mode_traj.GenerateLcmObject();
}

// Builds the sample cost sender message.
void BuildSampleCostSender(const std::vector<double>& sample_costs, dairlib::lcmt_timestamped_saved_traj* output) {
  // Create a matrix of sample costs
  Eigen::MatrixXd cost_datapoints = Eigen::MatrixXd::Zero(1, sample_costs.size());
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(sample_costs.size());

  for (int i = 0; i < sample_costs.size(); i++) {
    cost_datapoints(0, i) = sample_costs[i];
    timestamps(i) = sample_costs[i];
  }

  LcmTrajectory::Trajectory sample_costs_traj;
  sample_costs_traj.traj_name = "sample_costs";
  sample_costs_traj.datatypes = std::vector<std::string>(1, "double");
  sample_costs_traj.datapoints = cost_datapoints;
  sample_costs_traj.time_vector = timestamps.cast<double>();

  LcmTrajectory cost_traj({sample_costs_traj}, {"sample_costs"},
                          "sample_costs",
                          "sample_costs", false);

  // Output the sample costs
  output->saved_traj = cost_traj.GenerateLcmObject();
}

// Builds the sample location sender message.
void BuildSampleLocationSender(const std::vector<Eigen::Vector3d>& sample_locations, 
                               dairlib::lcmt_timestamped_saved_traj* output) {
  // Create a matrix of sample locations
  Eigen::MatrixXd sample_datapoints = Eigen::MatrixXd::Zero(3, sample_locations.size());
  Eigen::VectorXd timestamps = Eigen::VectorXd::Zero(sample_locations.size());

  for (int i = 0; i < sample_locations.size(); i++) {
    sample_datapoints.col(i) = sample_locations[i];
    timestamps(i) = i;
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
  output->saved_traj = sample_traj.GenerateLcmObject();
}

} // namespace systems
}  // namespace dairlib