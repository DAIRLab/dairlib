
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include "drake/lcm/drake_lcm.h"
#include <gflags/gflags.h>
#include <chrono>
#include <thread>

#include "examples/iC3/trifinger/parameter_headers/trifinger_osc_controller_params.h"
#include "examples/iC3/trifinger/parameter_headers/trifinger_lcm_channels.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "lcm/lcm_trajectory.h"

namespace dairlib {

using Eigen::MatrixXd;
using Eigen::VectorXd;
using dairlib::lcmt_timestamped_saved_traj;

DEFINE_string(lcm_channels,
              "examples/iC3/trifinger/parameters/trifinger_lcm_channels_simulation.yaml",
              "Filepath containing lcm channels");

DEFINE_string(osc_params,
              "examples/iC3/trifinger/parameters/trifinger_osc_controller_params.yaml",
              "Filepath containing lcm channels");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  TrifingerOSCControllerParams controller_params =
      drake::yaml::LoadYamlFile<TrifingerOSCControllerParams>(
          FLAGS_osc_params);

  TrifingerLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<TrifingerLcmChannels>(FLAGS_lcm_channels);

  
  /* [[0, 0.07, 0.05],  
      [ 0.07, -0.055, 0.05],  
      [-0.07, -0.055, 0.05]]  */
  VectorXd init_pose(9);
  init_pose.segment(0, 3) = controller_params.neutral_position.at(0);
  init_pose.segment(3, 3) = controller_params.neutral_position.at(1);
  init_pose.segment(6, 3) = controller_params.neutral_position.at(2);

  int num_knots = 120;
  double dt = 0.05;

  VectorXd point_1(9);
  VectorXd point_2(9);
  VectorXd point_3(9);
  VectorXd point_4(9);
  VectorXd point_5(9);
  VectorXd point_6(9);

  
  point_1 << 0.07, -0.055, 0.05, -0.07, -0.055, 0.05, 0, 0.07, 0.05;
  point_2 = init_pose;
  point_3 << -0.07, -0.055, 0.05, 0, 0.07, 0.05, 0.07, -0.055, 0.05;
  point_4 = init_pose;
  point_5 << 0, -0.07, 0.05, 0.07, 0.055, 0.05, -0.07, 0.055, 0.05;
  point_6 << init_pose;

  MatrixXd positions(9, num_knots);
  MatrixXd forces(MatrixXd::Zero(9, num_knots));


  for (int i = 0; i < num_knots / 6; i++) {
    double lambda = static_cast<double>(i) / (num_knots / 6);
    positions.col(i) = (1-lambda) * init_pose + lambda * point_1;
    positions.col(i + num_knots / 6) = (1-lambda) * point_1 + lambda * point_2;

    positions.col(i + 2 * num_knots / 6) = (1-lambda) * point_2 + lambda * point_3;
    positions.col(i + 3 * num_knots / 6) = (1-lambda) * point_3 + lambda * point_4;

    positions.col(i + 4 * num_knots / 6) = (1-lambda) * point_4 + lambda * point_5;
    positions.col(i + 5 * num_knots / 6) = (1-lambda) * point_5 + lambda * point_6;
  }

  int traj_size = 8;
  for (int i = 0; i < num_knots - traj_size; i++) {

    VectorXd time_vector(traj_size);
    for (int i = 0; i < traj_size; i++) {
      time_vector(i) = dt * i;
    }
    
    LcmTrajectory::Trajectory end_effector_traj;
    end_effector_traj.traj_name = "end_effector_position_target";
    end_effector_traj.datatypes =
        std::vector<std::string>(positions.rows(), "double");
    end_effector_traj.datapoints = positions.middleCols(i, traj_size);
    end_effector_traj.time_vector = time_vector;
    LcmTrajectory lcm_traj({end_effector_traj}, {"end_effector_position_target"},
                          "end_effector_position_target",
                          "end_effector_position_target", false);


    LcmTrajectory::Trajectory end_effector_force_traj;
    end_effector_force_traj.traj_name = "end_effector_force_target";
    end_effector_force_traj.datatypes =
        std::vector<std::string>(forces.rows(), "double");
    end_effector_force_traj.datapoints = forces.middleCols(i, traj_size);
    end_effector_force_traj.time_vector = time_vector;
    lcm_traj.AddTrajectory(end_effector_force_traj.traj_name, end_effector_force_traj);                   


    lcmt_timestamped_saved_traj msg;
    msg.saved_traj = lcm_traj.GenerateLcmObject();
    msg.utime = i * dt * 1e6;
    std::cout << "u time " << msg.utime << std::endl;
    
    drake::lcm::Publish(&lcm, lcm_channel_params.c3_actor_channel, msg);
    std::cout << "knot " << i << std::endl;
    std::this_thread::sleep_for(std::chrono::duration<double>(dt));

  }



  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }