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
using std::vector;

DEFINE_string(trajectory_name, "",
              "File name where the optimal trajectory is stored.");
DEFINE_string(folder_path,
              "/home/yangwill/Documents/research/projects/cassie"
              "/jumping/saved_trajs/",
              "Folder path for where the trajectory names are stored");
DEFINE_int32(num_modes, 0, "Number of contact modes in the trajectory");
DEFINE_string(mode_name, "state_input_trajectory",
              "Base name of each trajectory");

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
  const std::map<string, int>& act_map =
      multibody::makeNameToActuatorsMap(plant_w_spr);

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

  const LcmTrajectory& loadedTrajs =
      LcmTrajectory(FLAGS_folder_path + FLAGS_trajectory_name);

  int n_points = 0;
  std::vector<int> knot_points;
  std::vector<LcmTrajectory::Trajectory> trajectories;
  for (int mode = 0; mode < FLAGS_num_modes; ++mode) {
    trajectories.push_back(
        loadedTrajs.getTrajectory(FLAGS_mode_name + std::to_string(mode)));
    knot_points.push_back(trajectories[mode].time_vector.size());

    n_points += knot_points[mode];
  }

  MatrixXd xu(nx_wo_spr + nx_wo_spr + nu, n_points);
  VectorXd times(n_points);

  int start_idx = 0;
  for (int mode = 0; mode < FLAGS_num_modes; ++mode) {
    if (mode != 0) start_idx += knot_points[mode - 1];
    xu.block(0, start_idx, nx_wo_spr + nx_wo_spr + nu, knot_points[mode]) =
        trajectories[mode].datapoints;
    times.segment(start_idx, knot_points[mode]) =
        trajectories[mode].time_vector;
  }

  MatrixXd x_w_spr(2 * nx_w_spr, n_points);
  MatrixXd u_w_spr(nu, n_points);
  x_w_spr.topRows(nx_w_spr) =
      map_state_from_no_spring_to_spring * xu.topRows(nx_wo_spr);
  x_w_spr.bottomRows(nx_w_spr) =
      map_state_from_no_spring_to_spring *
      xu.topRows(2 * nx_wo_spr).bottomRows(nx_wo_spr);
  u_w_spr = xu.bottomRows(nu);

  /*** Reflecting the trajectory ***/
  MatrixXd x_w_spr_reflected = x_w_spr;
  MatrixXd u_w_spr_reflected = u_w_spr;
  std::pair<string, string> l_r_pair =
      std::pair<string, string>("_left", "_right");
  vector<string> asy_joint_names{
      "hip_roll",
      "hip_yaw",
  };
  vector<string> sym_joint_names{"hip_pitch", "knee", "ankle_joint", "toe"};

  vector<std::pair<int, int>> asy_pos_indices;
  vector<std::pair<int, int>> asy_vel_indices;
  vector<std::pair<int, int>> sym_pos_indices;
  vector<std::pair<int, int>> sym_vel_indices;
  vector<std::pair<int, int>> asy_u_indices;
  vector<std::pair<int, int>> sym_u_indices;

  for (const auto& asy_joint_name : asy_joint_names) {
    std::cout << "\nasymmetric joint name: " << asy_joint_name + l_r_pair.first;
    std::cout << "\n pos index: "
              << pos_map_w_spr.at(asy_joint_name + l_r_pair.first)
              << "\n vel index: "
              << vel_map_w_spr.at(asy_joint_name + l_r_pair.first + "dot");
    asy_pos_indices.emplace_back(
        pos_map_w_spr.at(asy_joint_name + l_r_pair.first),
        pos_map_w_spr.at(asy_joint_name + l_r_pair.second));
    asy_vel_indices.emplace_back(
        vel_map_w_spr.at(asy_joint_name + l_r_pair.first + "dot"),
        vel_map_w_spr.at(asy_joint_name + l_r_pair.second + "dot"));
    asy_u_indices.emplace_back(
        act_map.at(asy_joint_name + l_r_pair.first + "_motor"),
        act_map.at(asy_joint_name + l_r_pair.second + "_motor"));
  }
  for (const auto& sym_joint_name : sym_joint_names) {
    std::cout << "\nsymmetric joint name: " << sym_joint_name + l_r_pair.first;
    std::cout << "\n pos index: "
              << pos_map_w_spr.at(sym_joint_name + l_r_pair.first)
              << "\n vel index: "
              << vel_map_w_spr.at(sym_joint_name + l_r_pair.first + "dot");
    sym_pos_indices.emplace_back(
        pos_map_w_spr.at(sym_joint_name + l_r_pair.first),
        pos_map_w_spr.at(sym_joint_name + l_r_pair.second));
    sym_vel_indices.emplace_back(
        vel_map_w_spr.at(sym_joint_name + l_r_pair.first + "dot"),
        vel_map_w_spr.at(sym_joint_name + l_r_pair.second + "dot"));
    if (sym_joint_name != "ankle_joint") {
      sym_u_indices.emplace_back(
          act_map.at(sym_joint_name + l_r_pair.first + "_motor"),
          act_map.at(sym_joint_name + l_r_pair.second + "_motor"));
    }
  }

  vector<int> offset_indices = {0, nx_w_spr};
  for (const auto& offset_idx : offset_indices) {
    for (const auto& asy_pos_index : asy_pos_indices) {
      VectorXd l_joint =
          x_w_spr_reflected.row(offset_idx + asy_pos_index.first);
      x_w_spr_reflected.row(offset_idx + asy_pos_index.first) =
          -x_w_spr_reflected.row(offset_idx + asy_pos_index.second);
      x_w_spr_reflected.row(offset_idx + asy_pos_index.second) = -l_joint;
    }
    for (const auto& asy_vel_index : asy_vel_indices) {
      int l_index = offset_idx + nq_w_spr + asy_vel_index.first;
      int r_index = offset_idx + nq_w_spr + asy_vel_index.second;
      VectorXd l_joint = x_w_spr_reflected.row(l_index);
      x_w_spr_reflected.row(l_index) = -x_w_spr_reflected.row(r_index);
      x_w_spr_reflected.row(r_index) = -l_joint;
    }
    for (const auto& sym_pos_index : sym_pos_indices) {
      VectorXd l_joint =
          x_w_spr_reflected.row(offset_idx + sym_pos_index.first);
      x_w_spr_reflected.row(offset_idx + sym_pos_index.first) =
          x_w_spr_reflected.row(offset_idx + sym_pos_index.second);
      x_w_spr_reflected.row(offset_idx + sym_pos_index.second) = l_joint;
    }
    for (const auto& sym_vel_index : sym_vel_indices) {
      int l_index = offset_idx + nq_w_spr + sym_vel_index.first;
      int r_index = offset_idx + nq_w_spr + sym_vel_index.second;
      VectorXd l_joint = x_w_spr_reflected.row(l_index);
      x_w_spr_reflected.row(l_index) = x_w_spr_reflected.row(r_index);
      x_w_spr_reflected.row(r_index) = l_joint;
    }
    for (const auto& sym_u_index : sym_u_indices) {
      u_w_spr_reflected.row(sym_u_index.first) =
          u_w_spr.row(sym_u_index.second);
      u_w_spr_reflected.row(sym_u_index.second) =
          u_w_spr.row(sym_u_index.first);
    }
    for (const auto& asy_u_index : asy_u_indices) {
      u_w_spr_reflected.row(asy_u_index.first) =
          -u_w_spr.row(asy_u_index.second);
      u_w_spr_reflected.row(asy_u_index.second) =
          -u_w_spr.row(asy_u_index.first);
    }
  }
  //  std::cout << "x_w_spr" << x_w_spr << std::endl;
  //  std::cout << "x_w_spr reflected" << x_w_spr_reflected << std::endl;

  MatrixXd x_w_spr_full(2 * nx_w_spr, 2 * n_points - 1);
  x_w_spr_full << x_w_spr, x_w_spr_reflected.rightCols(n_points - 1);
  MatrixXd u_w_spr_full(nu, 2 * n_points - 1);
  u_w_spr_full << u_w_spr, u_w_spr_reflected.rightCols(n_points - 1);
  VectorXd times_full(2 * n_points - 1);
  VectorXd time_offset = VectorXd::Ones(n_points) * times.tail(1);
  VectorXd times_copy = times + time_offset;
  times_full << times, times_copy.tail(n_points - 1);

  /*** End of reflecting the trajctory ***/

  auto state_traj_w_spr = LcmTrajectory::Trajectory();
  state_traj_w_spr.traj_name = "state_trajectory";
  state_traj_w_spr.datapoints = x_w_spr_full;
  state_traj_w_spr.time_vector = times_full;
  auto input_traj_w_spr = LcmTrajectory::Trajectory();
  input_traj_w_spr.traj_name = "input_trajectory";
  input_traj_w_spr.datapoints = u_w_spr_full;
  input_traj_w_spr.time_vector = times_full;
  const std::vector<string> state_names =
      multibody::createStateNameVectorFromMap(plant_w_spr);
  const std::vector<string> state_dot_names =
      multibody::createStateNameVectorFromMap(plant_w_spr);
  const std::vector<string> input_names =
      multibody::createActuatorNameVectorFromMap(plant_w_spr);

  state_traj_w_spr.datatypes.reserve(2 * nx_w_spr);
  state_traj_w_spr.datatypes.insert(state_traj_w_spr.datatypes.end(),
                                    state_names.begin(), state_names.end());
  state_traj_w_spr.datatypes.insert(state_traj_w_spr.datatypes.end(),
                                    state_dot_names.begin(),
                                    state_dot_names.end());
  input_traj_w_spr.datatypes = input_names;

  std::vector<LcmTrajectory::Trajectory> converted_trajectories = {
      state_traj_w_spr, input_traj_w_spr};
  std::vector<std::string> trajectory_names = {state_traj_w_spr.traj_name,
                                               input_traj_w_spr.traj_name};

  auto processed_traj =
      LcmTrajectory(converted_trajectories, trajectory_names,
                    "trajectory for cassie model with springs",
                    "State trajectory for cassie adjusted to include "
                    "states of the plant with springs");

  processed_traj.writeToFile(FLAGS_folder_path + FLAGS_trajectory_name +
                             "_for_sim");
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain();
}