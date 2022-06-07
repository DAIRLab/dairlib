#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "examples/Cassie/cassie_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"

#include "drake/multibody/plant/multibody_plant.h"

using std::map;
using std::pair;
using std::string;
using std::vector;

using drake::geometry::SceneGraph;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::trajectories::PiecewisePolynomial;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

DEFINE_bool(relative_feet, true,
            "Set to false if feet positions should "
            "be measured relative to the world "
            "instead of as an offset from the hips");
DEFINE_string(trajectory_name, "",
              "File name where the optimal trajectory is stored.");
DEFINE_string(folder_path, "",
              "Folder path for where the trajectory names are stored");

namespace dairlib {

/// This program pre-computes the output trajectories (center of mass, pelvis
/// orientation, feet trajectories) for the OSC controller.
///

int DoMain() {
  // Drake system initialization
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");
  MultibodyPlant<double> plant(1e-5);  // non-zero timestep to avoid continuous
  // model warnings
  Parser parser(&plant, &scene_graph);
  parser.AddModelFromFile(
      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"));
  plant.mutable_gravity_field().set_gravity_vector(-9.81 *
                                                   Eigen::Vector3d::UnitZ());
  plant.Finalize();

  // Create maps for joints
  map<string, int> pos_map = multibody::MakeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::MakeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::MakeNameToActuatorsMap(plant);

  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();

  int nv = plant.num_velocities();
  int nx = plant.num_positions() + plant.num_velocities();

  auto l_toe_frame = &plant.GetBodyByName("toe_left").body_frame();
  auto r_toe_frame = &plant.GetBodyByName("toe_right").body_frame();
  auto hip_left_frame = &plant.GetBodyByName("hip_left").body_frame();
  auto hip_right_frame = &plant.GetBodyByName("hip_right").body_frame();
  auto pelvis_frame = &plant.GetBodyByName("pelvis").body_frame();
  auto world = &plant.world_frame();

  DirconTrajectory dircon_traj(plant,
                               FLAGS_folder_path + FLAGS_trajectory_name);
  PiecewisePolynomial<double> state_traj =
      dircon_traj.ReconstructStateTrajectory();

  std::vector<MatrixXd> all_l_foot_points;
  std::vector<MatrixXd> all_r_foot_points;
  std::vector<MatrixXd> all_l_toe_points;
  std::vector<MatrixXd> all_r_toe_points;
  std::vector<MatrixXd> all_l_hip_points;
  std::vector<MatrixXd> all_r_hip_points;
  std::vector<MatrixXd> all_pelvis_points;
  std::vector<MatrixXd> all_pelvis_orientation;
  std::vector<VectorXd> all_times;
  Vector3d zero_offset = Vector3d::Zero();

  for (int mode = 0; mode < dircon_traj.GetNumModes(); ++mode) {
    if (dircon_traj.GetStateBreaks(mode).size() <= 1) {
      continue;
    }
    VectorXd times = dircon_traj.GetStateBreaks(mode);
    MatrixXd state_samples = dircon_traj.GetStateSamples(mode);
    MatrixXd state_derivative_samples =
        dircon_traj.GetStateDerivativeSamples(mode);
    int n_points = times.size();
    MatrixXd l_foot_points(9, n_points);
    MatrixXd r_foot_points(9, n_points);
    MatrixXd l_toe_points(2, n_points);
    MatrixXd r_toe_points(2, n_points);
    MatrixXd l_hip_points(9, n_points);
    MatrixXd r_hip_points(9, n_points);
    MatrixXd pelvis_points(6, n_points);
    MatrixXd pelvis_orientation(8, n_points);

    for (unsigned int i = 0; i < times.size(); ++i) {
      VectorXd x_i = state_samples.col(i).head(nx);
      VectorXd xdot_i = state_derivative_samples.col(i);
      plant.SetPositionsAndVelocities(context.get(), x_i);
      Eigen::Ref<Eigen::MatrixXd> pelvis_pos_block =
          pelvis_points.block(0, i, 3, 1);
      Eigen::Ref<Eigen::MatrixXd> l_foot_pos_block =
          l_foot_points.block(0, i, 3, 1);
      Eigen::Ref<Eigen::MatrixXd> r_foot_pos_block =
          r_foot_points.block(0, i, 3, 1);
      Eigen::Ref<Eigen::MatrixXd> l_hip_pos_block =
          l_hip_points.block(0, i, 3, 1);
      Eigen::Ref<Eigen::MatrixXd> r_hip_pos_block =
          r_hip_points.block(0, i, 3, 1);
      plant.CalcPointsPositions(*context, *pelvis_frame, zero_offset, *world,
                                &pelvis_pos_block);
      plant.CalcPointsPositions(*context, *l_toe_frame, zero_offset, *world,
                                &l_foot_pos_block);
      plant.CalcPointsPositions(*context, *r_toe_frame, zero_offset, *world,
                                &r_foot_pos_block);
      plant.CalcPointsPositions(*context, *hip_left_frame, zero_offset, *world,
                                &l_hip_pos_block);
      plant.CalcPointsPositions(*context, *hip_right_frame, zero_offset, *world,
                                &r_hip_pos_block);

      l_toe_points(0, i) = x_i(pos_map["toe_left"]);
      r_toe_points(0, i) = x_i(pos_map["toe_right"]);

      pelvis_orientation.block(0, i, 4, 1) = x_i.head(4);
      pelvis_orientation.block(4, i, 4, 1) = xdot_i.head(4);

      MatrixXd J_pelvis(3, nv);
      MatrixXd J_l_foot(3, nv);
      MatrixXd J_r_foot(3, nv);
      MatrixXd J_l_hip(3, nv);
      MatrixXd J_r_hip(3, nv);

      plant.CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV,
                                              *pelvis_frame, zero_offset,
                                              *world, *world, &J_pelvis);
      plant.CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV,
                                              *l_toe_frame, zero_offset, *world,
                                              *world, &J_l_foot);
      plant.CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV,
                                              *r_toe_frame, zero_offset, *world,
                                              *world, &J_r_foot);
      plant.CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV,
                                              *hip_left_frame, zero_offset,
                                              *world, *world, &J_l_hip);
      plant.CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV,
                                              *hip_right_frame, zero_offset,
                                              *world, *world, &J_r_hip);

      VectorXd v_i = x_i.tail(nv);
      pelvis_points.block(3, i, 3, 1) = J_pelvis * v_i;
      l_foot_points.block(3, i, 3, 1) = J_l_foot * v_i;
      r_foot_points.block(3, i, 3, 1) = J_r_foot * v_i;
      l_hip_points.block(3, i, 3, 1) = J_l_hip * v_i;
      r_hip_points.block(3, i, 3, 1) = J_r_hip * v_i;
      l_toe_points(1, i) = v_i(vel_map["toe_leftdot"]);
      r_toe_points(1, i) = v_i(vel_map["toe_rightdot"]);

      VectorXd JdotV_l_foot = plant.CalcBiasTranslationalAcceleration(
          *context, JacobianWrtVariable::kV, *l_toe_frame, zero_offset, *world,
          *world);
      VectorXd JdotV_r_foot = plant.CalcBiasTranslationalAcceleration(
          *context, JacobianWrtVariable::kV, *r_toe_frame, zero_offset, *world,
          *world);
      VectorXd JdotV_l_hip = plant.CalcBiasTranslationalAcceleration(
          *context, JacobianWrtVariable::kV, *hip_left_frame, zero_offset,
          *world, *world);
      VectorXd JdotV_r_hip = plant.CalcBiasTranslationalAcceleration(
          *context, JacobianWrtVariable::kV, *hip_right_frame, zero_offset,
          *world, *world);
      l_foot_points.block(6, i, 3, 1) =
          JdotV_l_foot + J_l_foot * xdot_i.tail(nv);
      r_foot_points.block(6, i, 3, 1) =
          JdotV_r_foot + J_r_foot * xdot_i.tail(nv);
      l_hip_points.block(6, i, 3, 1) = JdotV_l_hip + J_l_hip * xdot_i.tail(nv);
      r_hip_points.block(6, i, 3, 1) = JdotV_r_hip + J_r_hip * xdot_i.tail(nv);
    }
    pelvis_points = pelvis_points -
                    0.5 * (l_foot_points.topRows(6) + r_foot_points.topRows(6));

    all_times.push_back(times);
    all_l_foot_points.push_back(l_foot_points);
    all_r_foot_points.push_back(r_foot_points);
    all_l_hip_points.push_back(l_hip_points);
    all_r_hip_points.push_back(r_hip_points);
    all_l_toe_points.push_back(l_toe_points);
    all_r_toe_points.push_back(r_toe_points);
    all_pelvis_points.push_back(pelvis_points);
    all_pelvis_orientation.push_back(pelvis_orientation);
  }
  std::vector<LcmTrajectory::Trajectory> converted_trajectories;
  std::vector<std::string> trajectory_names;

  for (int mode = 0; mode < all_times.size(); ++mode) {
    auto lfoot_traj_block = LcmTrajectory::Trajectory();
    lfoot_traj_block.traj_name = "left_foot_trajectory" + std::to_string(mode);
    lfoot_traj_block.datapoints = all_l_foot_points[mode];
    lfoot_traj_block.time_vector = all_times[mode];
    lfoot_traj_block.datatypes = {"lfoot_x",     "lfoot_y",     "lfoot_z",
                                  "lfoot_xdot",  "lfoot_ydot",  "lfoot_zdot",
                                  "lfoot_xddot", "lfoot_yddot", "lfoot_zddot"};

    auto rfoot_traj_block = LcmTrajectory::Trajectory();
    rfoot_traj_block.traj_name = "right_foot_trajectory" + std::to_string(mode);
    rfoot_traj_block.datapoints = all_r_foot_points[mode];
    rfoot_traj_block.time_vector = all_times[mode];
    rfoot_traj_block.datatypes = {"rfoot_x",     "rfoot_y",     "rfoot_z",
                                  "rfoot_xdot",  "rfoot_ydot",  "rfoot_zdot",
                                  "rfoot_xddot", "rfoot_yddot", "rfoot_zddot"};

    auto lhip_traj_block = LcmTrajectory::Trajectory();
    lhip_traj_block.traj_name = "left_hip_trajectory" + std::to_string(mode);
    lhip_traj_block.datapoints = all_l_hip_points[mode];
    lhip_traj_block.time_vector = all_times[mode];
    lhip_traj_block.datatypes = {"lhip_x",     "lhip_y",     "lhip_z",
                                 "lhip_xdot",  "lhip_ydot",  "lhip_zdot",
                                 "lhip_xddot", "lhip_yddot", "lhip_zddot"};

    auto rhip_traj_block = LcmTrajectory::Trajectory();
    rhip_traj_block.traj_name = "right_hip_trajectory" + std::to_string(mode);
    rhip_traj_block.datapoints = all_r_hip_points[mode];
    rhip_traj_block.time_vector = all_times[mode];
    rhip_traj_block.datatypes = {"rhip_x",     "rhip_y",     "rhip_z",
                                 "rhip_xdot",  "rhip_ydot",  "rhip_zdot",
                                 "rhip_xddot", "rhip_yddot", "rhip_zddot"};

    auto ltoe_traj_block = LcmTrajectory::Trajectory();
    ltoe_traj_block.traj_name = "left_toe_trajectory" + std::to_string(mode);
    ltoe_traj_block.datapoints = all_l_toe_points[mode];
    ltoe_traj_block.time_vector = all_times[mode];
    ltoe_traj_block.datatypes = {"ltoe", "ltoe_dot"};

    auto rtoe_traj_block = LcmTrajectory::Trajectory();
    rtoe_traj_block.traj_name = "right_toe_trajectory" + std::to_string(mode);
    rtoe_traj_block.datapoints = all_r_toe_points[mode];
    rtoe_traj_block.time_vector = all_times[mode];
    rtoe_traj_block.datatypes = {"rtoe", "rtoe_dot"};

    auto com_traj_block = LcmTrajectory::Trajectory();
    com_traj_block.traj_name = "pelvis_trans_trajectory" + std::to_string(mode);
    com_traj_block.datapoints = all_pelvis_points[mode];
    com_traj_block.time_vector = all_times[mode];
    com_traj_block.datatypes = {"com_x",    "com_y",    "com_z",
                                "com_xdot", "com_ydot", "com_zdot"};

    auto pelvis_orientation_block = LcmTrajectory::Trajectory();
    pelvis_orientation_block.traj_name =
        "pelvis_rot_trajectory" + std::to_string(mode);
    pelvis_orientation_block.datapoints = all_pelvis_orientation[mode];
    pelvis_orientation_block.time_vector = all_times[mode];
    pelvis_orientation_block.datatypes = {
        "pelvis_rotw",    "pelvis_rotx",    "pelvis_roty",    "pelvis_rotz",
        "pelvis_rotwdot", "pelvis_rotxdot", "pelvis_rotydot", "pelvis_rotzdot"};

    converted_trajectories.push_back(lfoot_traj_block);
    converted_trajectories.push_back(rfoot_traj_block);
    converted_trajectories.push_back(lhip_traj_block);
    converted_trajectories.push_back(rhip_traj_block);
    converted_trajectories.push_back(ltoe_traj_block);
    converted_trajectories.push_back(rtoe_traj_block);
    converted_trajectories.push_back(com_traj_block);
    converted_trajectories.push_back(pelvis_orientation_block);
    trajectory_names.push_back(lfoot_traj_block.traj_name);
    trajectory_names.push_back(rfoot_traj_block.traj_name);
    trajectory_names.push_back(lhip_traj_block.traj_name);
    trajectory_names.push_back(rhip_traj_block.traj_name);
    trajectory_names.push_back(ltoe_traj_block.traj_name);
    trajectory_names.push_back(rtoe_traj_block.traj_name);
    trajectory_names.push_back(com_traj_block.traj_name);
    trajectory_names.push_back(pelvis_orientation_block.traj_name);
  }

  auto processed_traj = LcmTrajectory(converted_trajectories, trajectory_names,
                                      "jumping_trajectory",
                                      "Output trajectories "
                                      "for Cassie jumping");

  if (FLAGS_relative_feet) {
    processed_traj.WriteToFile(FLAGS_folder_path + FLAGS_trajectory_name +
                               "_processed" + "_rel");
  } else {
    processed_traj.WriteToFile(FLAGS_folder_path + FLAGS_trajectory_name +
                               "_processed");
  }
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain();
}