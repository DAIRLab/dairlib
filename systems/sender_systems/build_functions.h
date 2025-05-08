#pragma once
#include <vector>
#include <Eigen/Core>
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"
#include "solvers/c3_output.h"

namespace dairlib {
namespace systems {

void BuildDynamicallyFeasiblePlanSenderActor(const std::vector<Eigen::VectorXd>& dynamically_feasible_plan, 
    dairlib::lcmt_timestamped_saved_traj* output);
void BuildDynamicallyFeasiblePlanSenderObject(const std::vector<Eigen::VectorXd>& dynamically_feasible_plan, 
    dairlib::lcmt_timestamped_saved_traj* output_traj);
void BuildTrackingTrajectoryActorSender(const LcmTrajectory& tracking_trajectory, 
    dairlib::lcmt_timestamped_saved_traj* output);
void BuildTrackingTrajectoryObjectSender(const LcmTrajectory& tracking_trajectory, 
         dairlib::lcmt_timestamped_saved_traj* output);
         void BuildC3TrackingTrajectoryActorSender(const C3Output::C3Solution& c3_solution, 
            dairlib::lcmt_timestamped_saved_traj* output);
void BuildC3TrackingTrajectoryObjectSender(const C3Output::C3Solution& c3_solution, 
              dairlib::lcmt_timestamped_saved_traj* output);

}  // namespace systems
}  // namespace dairlib