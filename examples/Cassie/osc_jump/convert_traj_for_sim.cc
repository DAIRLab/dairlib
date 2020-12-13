#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "examples/Cassie/cassie_utils.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"

#include "drake/multibody/plant/multibody_plant.h"

using drake::geometry::SceneGraph;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::string;

DEFINE_string(trajectory_name, "",
              "File name where the optimal trajectory is stored.");
DEFINE_string(folder_path,
              "/home/yangwill/Documents/research/projects/cassie"
              "/jumping/saved_trajs/",
              "Folder path for where the trajectory names are stored");

namespace dairlib {

/// This program converts the trajectory computed using the fixed spring
/// cassie model to the cassie model with springs. This is necessary to
/// initialize the simulator at a particular state along the trajectory
int DoMain() {
  // Drake system initialization stuff
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");
  MultibodyPlant<double> plant_wo_spr(
      1e-5);  // non-zero timestep to avoid continuous
  MultibodyPlant<double> plant_w_spr(1e-5);  // non-zero timestep to avoid

  Parser parser_wo_spr(&plant_wo_spr, &scene_graph);
  Parser parser_w_spr(&plant_w_spr, &scene_graph);
  parser_wo_spr.AddModelFromFile(
      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"));
  parser_w_spr.AddModelFromFile(
      FindResourceOrThrow("examples/Cassie/urdf/cassie_v2.urdf"));
  plant_wo_spr.mutable_gravity_field().set_gravity_vector(
      -9.81 * Eigen::Vector3d::UnitZ());
  plant_w_spr.mutable_gravity_field().set_gravity_vector(
      -9.81 * Eigen::Vector3d::UnitZ());
  plant_wo_spr.Finalize();
  plant_w_spr.Finalize();

  int nq_wo_spr = plant_wo_spr.num_positions();
  int nv_wo_spr = plant_wo_spr.num_velocities();
  int nq_w_spr = plant_w_spr.num_positions();
  int nv_w_spr = plant_w_spr.num_velocities();

  int nx_wo_spr = nq_wo_spr + nv_wo_spr;
  int nx_w_spr = nq_w_spr + nv_w_spr;

  int nu = plant_w_spr.num_actuators();

  const std::map<string, int>& pos_map_w_spr =
      multibody::makeNameToPositionsMap(plant_w_spr);
  const std::map<string, int>& vel_map_w_spr =
      multibody::makeNameToVelocitiesMap(plant_w_spr);
  const std::map<string, int>& pos_map_wo_spr =
      multibody::makeNameToPositionsMap(plant_wo_spr);
  const std::map<string, int>& vel_map_wo_spr =
      multibody::makeNameToVelocitiesMap(plant_wo_spr);

  // Initialize the mapping from states for the plant without springs to the
  // plant with springs. Note: this is a tall matrix
  MatrixXd map_position_from_no_spring_to_spring =
      MatrixXd::Zero(nq_w_spr, nq_wo_spr);
  MatrixXd map_velocity_from_no_spring_to_spring =
      MatrixXd::Zero(nv_w_spr, nv_wo_spr);
  MatrixXd map_state_from_no_spring_to_spring =
      MatrixXd::Zero(nx_w_spr, nx_wo_spr);

  for (const auto& pos_pair_wo_spr : pos_map_wo_spr) {
    bool successfully_added = false;
    for (const auto& pos_pair_w_spr : pos_map_w_spr) {
      if (pos_pair_wo_spr.first == pos_pair_w_spr.first) {
        map_position_from_no_spring_to_spring(pos_pair_w_spr.second,
                                              pos_pair_wo_spr.second) = 1;
        successfully_added = true;
      }
    }
    DRAKE_DEMAND(successfully_added);
  }

  for (const auto& vel_pair_wo_spr : vel_map_wo_spr) {
    bool successfully_added = false;
    for (const auto& vel_pair_w_spr : vel_map_w_spr) {
      if (vel_pair_wo_spr.first == vel_pair_w_spr.first) {
        map_velocity_from_no_spring_to_spring(vel_pair_w_spr.second,
                                              vel_pair_wo_spr.second) = 1;
        successfully_added = true;
      }
    }
    DRAKE_DEMAND(successfully_added);
  }

  map_state_from_no_spring_to_spring.block(0, 0, nq_w_spr, nq_wo_spr) =
      map_position_from_no_spring_to_spring;
  map_state_from_no_spring_to_spring.block(nq_w_spr, nq_wo_spr, nv_w_spr,
                                           nv_wo_spr) =
      map_velocity_from_no_spring_to_spring;

  LcmTrajectory loadedTrajs =
      LcmTrajectory(FLAGS_folder_path + FLAGS_trajectory_name);
  auto traj_mode0 = loadedTrajs.GetTrajectory("cassie_jumping_trajectory_x_u0");
  auto traj_mode1 = loadedTrajs.GetTrajectory("cassie_jumping_trajectory_x_u1");
  auto traj_mode2 = loadedTrajs.GetTrajectory("cassie_jumping_trajectory_x_u2");

  int n_points = traj_mode0.datapoints.cols() + traj_mode1.datapoints.cols() +
                 traj_mode2.datapoints.cols();

  MatrixXd xu(2 * nx_wo_spr + nu, n_points);
  VectorXd times(n_points);

  xu << traj_mode0.datapoints, traj_mode1.datapoints, traj_mode2.datapoints;
  times << traj_mode0.time_vector, traj_mode1.time_vector,
      traj_mode2.time_vector;
  MatrixXd x_w_spr(2 * nx_w_spr, n_points);
  x_w_spr.topRows(nx_w_spr) =
      map_state_from_no_spring_to_spring * xu.topRows(nx_wo_spr);
  x_w_spr.bottomRows(nx_w_spr) =
      map_state_from_no_spring_to_spring *
      xu.topRows(2 * nx_wo_spr).bottomRows(nx_wo_spr);

  auto state_traj_w_spr = LcmTrajectory::Trajectory();
  state_traj_w_spr.traj_name = "cassie_jumping_trajectory_x";
  state_traj_w_spr.datapoints = x_w_spr;
  state_traj_w_spr.time_vector = times;
  const std::vector<string> state_names =
      multibody::createStateNameVectorFromMap(plant_w_spr);
  const std::vector<string> state_dot_names =
      multibody::createStateNameVectorFromMap(plant_w_spr);

  state_traj_w_spr.datatypes.reserve(2 * nx_w_spr);
  state_traj_w_spr.datatypes.insert(state_traj_w_spr.datatypes.end(),
                                    state_names.begin(), state_names.end());
  state_traj_w_spr.datatypes.insert(state_traj_w_spr.datatypes.end(),
                                    state_dot_names.begin(),
                                    state_dot_names.end());

  std::vector<LcmTrajectory::Trajectory> trajectories = {state_traj_w_spr};
  std::vector<std::string> trajectory_names = {state_traj_w_spr.traj_name};

  auto processed_traj =
      LcmTrajectory(trajectories, trajectory_names, "jumping_trajectory",
                    "State trajectory for cassie jumping adjusted to include "
                    "states of the plant with springs");

  processed_traj.WriteToFile(FLAGS_folder_path + FLAGS_trajectory_name +
                             "_for_sim");
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain();
}