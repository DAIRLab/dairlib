#include <iostream>
#include "examples/cube_flip/trajectory_lcm_parser.h"

#include "common/find_resource.h"

namespace dairlib {

TrajectoryLcmParser::TrajectoryLcmParser(int num_trajectories, int N)
: num_trajectories_(num_trajectories),
  N_(N) {

    this->set_name("trajectory_lcm_parser");

    trajectory_input_port_ =
        this->DeclareAbstractInputPort(
            "all_trajectories",
              drake::Value<dairlib::lcmt_timestamped_saved_traj>{})
        .get_index();

    for (int i = 0; i < num_trajectories_; ++i) {
      std::string name = "trajectory_" + std::to_string(i);
      trajectory_output_ports_.push_back(
        this->DeclareAbstractOutputPort(
            name,
            []() {
              return drake::AbstractValue::Make(
                  dairlib::lcmt_timestamped_saved_traj());
            },
            [this, i](const drake::systems::Context<double>& context,
                      drake::AbstractValue* output) {
              auto& msg =
                  output->get_mutable_value<dairlib::lcmt_timestamped_saved_traj>();
              this->GetTrajectory(context, &msg, i);
            }
        ).get_index()
      );
    }
  }

  void TrajectoryLcmParser::GetTrajectory(
    const drake::systems::Context<double>& context, 
    lcmt_timestamped_saved_traj* traj, int i) {

    const lcmt_timestamped_saved_traj* lcm_all_trajectories =
      (lcmt_timestamped_saved_traj*)this->EvalVectorInput(context, trajectory_input_port_);

    LcmTrajectory trajectory = LcmTrajectory(lcm_all_trajectories->saved_traj);
    
    // Pull out trajectory corresponding to ith iteration
    const std::string trajectory_i_name = "iteration_" + std::to_string(i);
    LcmTrajectory::Trajectory trajectory_i = trajectory.GetTrajectory(trajectory_i_name);

    LcmTrajectory lcm_trajectory_i({trajectory_i}, {"trajectory_i"},
                          "trajectory_i", "trajectory_i", false);

    traj->saved_traj = lcm_trajectory_i.GenerateLcmObject();
    traj->utime = context.get_time() * 1e6;
  }




}