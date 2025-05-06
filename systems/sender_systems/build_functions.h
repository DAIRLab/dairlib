#pragma once
#include <vector>
#include <Eigen/Core>
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

namespace dairlib {
namespace systems {

void BuildC3ModeSender(const Eigen::VectorXd& is_c3_mode, dairlib::lcmt_timestamped_saved_traj* output);
void BuildSampleCostSender(const std::vector<double>& sample_costs, dairlib::lcmt_timestamped_saved_traj* output);
void BuildSampleLocationSender(const std::vector<Eigen::Vector3d>& sample_locations, 
                               dairlib::lcmt_timestamped_saved_traj* output);

}  // namespace systems
}  // namespace dairlib