#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>

#include "examples/Cassie/cassie_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"

#include "drake/multibody/plant/multibody_plant.h"

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

DEFINE_string(trajectory_name, "",
              "File name where the optimal trajectory is stored.");
DEFINE_string(folder_path, "",
              "Folder path for where the trajectory names are stored");

namespace dairlib {

int ComputeCoeffMatrix(const std::vector<double>& breaks,
                       const std::vector<MatrixXd>& samples, int row, int col,
                       MatrixXd* A, VectorXd* b) {


}

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

  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();

  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int nu = plant.num_actuators();
  int nx = plant.num_positions() + plant.num_velocities();

  auto l_toe_frame = &plant.GetBodyByName("toe_left").body_frame();
  auto r_toe_frame = &plant.GetBodyByName("toe_right").body_frame();
  auto hip_left_frame = &plant.GetBodyByName("hip_left").body_frame();
  auto hip_right_frame = &plant.GetBodyByName("hip_right").body_frame();
  auto pelvis_frame = &plant.GetBodyByName("pelvis").body_frame();
  auto world = &plant.world_frame();

  DirconTrajectory dircon_traj(FLAGS_folder_path + FLAGS_trajectory_name);
  PiecewisePolynomial<double> state_traj =
      dircon_traj.ReconstructStateTrajectory();

  std::vector<MatrixXd> all_l_foot_points;
  std::vector<MatrixXd> all_r_foot_points;
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
    std::cout << "times: " << times << std::endl;
    MatrixXd state_samples = dircon_traj.GetStateSamples(mode);
    int n_points = times.size();
    MatrixXd l_foot_points(9, n_points);
    MatrixXd r_foot_points(9, n_points);
    MatrixXd l_hip_points(9, n_points);
    MatrixXd r_hip_points(9, n_points);
    MatrixXd pelvis_points(9, n_points);
    MatrixXd pelvis_orientation(12, n_points);
    for (unsigned int i = 0; i < times.size(); ++i) {
      VectorXd x_i = state_samples.col(i).head(nx);
      VectorXd xdot_i = state_samples.col(i).tail(nx);

      plant.SetPositionsAndVelocities(context.get(), x_i);
      Eigen::Ref<Eigen::MatrixXd> pelvis_pos_block =
          pelvis_points.block(0, i, 3, 1);
      Eigen::Ref<Eigen::MatrixXd> l_foot_pos_block =
          l_foot_points.block(0, i, 3, 1);
      Eigen::Ref<Eigen::MatrixXd> r_foot_pos_block =
          r_foot_points.block(0, i, 3, 1);
      plant.CalcPointsPositions(*context, *pelvis_frame, zero_offset, *world,
                                &pelvis_pos_block);
      plant.CalcPointsPositions(*context, *l_toe_frame, zero_offset, *world,
                                &l_foot_pos_block);
      plant.CalcPointsPositions(*context, *r_toe_frame, zero_offset, *world,
                                &r_foot_pos_block);

      pelvis_orientation.block(0, i, 4, 1) = x_i.head(4);
      pelvis_orientation.block(4, i, 4, 1) = xdot_i.head(4);

      MatrixXd J_CoM(3, nv);
      MatrixXd J_l_foot(3, nv);
      MatrixXd J_r_foot(3, nv);
      plant.CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV,
                                              *pelvis_frame, zero_offset,
                                              *world, *world, &J_CoM);
      plant.CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV,
                                              *l_toe_frame, zero_offset, *world,
                                              *world, &J_l_foot);
      plant.CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV,
                                              *r_toe_frame, zero_offset, *world,
                                              *world, &J_r_foot);
      VectorXd v_i = x_i.tail(nv);
      pelvis_points.block(3, i, 3, 1) = J_CoM * v_i;
      l_foot_points.block(3, i, 3, 1) = J_l_foot * v_i;
      r_foot_points.block(3, i, 3, 1) = J_r_foot * v_i;

      VectorXd JdotV_CoM = plant.CalcBiasTranslationalAcceleration(
          *context, JacobianWrtVariable::kV, *pelvis_frame, zero_offset, *world,
          *world);
      VectorXd JdotV_l_foot = plant.CalcBiasTranslationalAcceleration(
          *context, JacobianWrtVariable::kV, *l_toe_frame, zero_offset, *world,
          *world);
      VectorXd JdotV_r_foot = plant.CalcBiasTranslationalAcceleration(
          *context, JacobianWrtVariable::kV, *r_toe_frame, zero_offset, *world,
          *world);

      pelvis_points.block(6, i, 3, 1) = JdotV_CoM + J_CoM * xdot_i.tail(nv);
      l_foot_points.block(6, i, 3, 1) =
          JdotV_l_foot + J_l_foot * xdot_i.tail(nv);
      r_foot_points.block(6, i, 3, 1) =
          JdotV_r_foot + J_r_foot * xdot_i.tail(nv);
    }
    all_times.push_back(times);
    all_l_foot_points.push_back(l_foot_points);
    all_r_foot_points.push_back(r_foot_points);
    all_l_hip_points.push_back(l_hip_points);
    all_r_hip_points.push_back(r_hip_points);
    all_pelvis_points.push_back(pelvis_points);
    all_pelvis_orientation.push_back(pelvis_orientation);
  }

  //  l_foot_points.row(2) = VectorXd::Zero(n_points);
  //  l_foot_points.row(5) = VectorXd::Zero(n_points);
  //  r_foot_points = r_foot_points - l_foot_points;
  //  l_foot_points = r_foot_points;
  //  l_foot_points.row(1) = -r_foot_points.row(1);
  //  l_foot_points.row(4) = -r_foot_points.row(4);

  //  std::vector<LcmTrajectory::Trajectory> converted_trajectories = {
  //      lfoot_traj_block, rfoot_traj_block, com_traj_block,
  //      pelvis_orientation_block};
  //  std::vector<std::string> trajectory_names = {
  //      lfoot_traj_block.traj_name, rfoot_traj_block.traj_name,
  //      com_traj_block.traj_name, pelvis_orientation_block.traj_name};

  std::vector<LcmTrajectory::Trajectory> converted_trajectories;
  std::vector<std::string> trajectory_names;

  for (int mode = 0; mode < all_times.size(); ++mode) {
    auto lfoot_traj_block = LcmTrajectory::Trajectory();
    lfoot_traj_block.traj_name = "left_foot_trajectory" + std::to_string(mode);
    lfoot_traj_block.datapoints = all_l_foot_points[mode];
    lfoot_traj_block.time_vector = all_times[mode];
    lfoot_traj_block.datatypes = {"lfoot_x",    "lfoot_y",    "lfoot_z",
                                  "lfoot_xdot", "lfoot_ydot", "lfoot_zdot"};

    auto rfoot_traj_block = LcmTrajectory::Trajectory();
    rfoot_traj_block.traj_name = "right_foot_trajectory" + std::to_string(mode);
    rfoot_traj_block.datapoints = all_r_foot_points[mode];
    rfoot_traj_block.time_vector = all_times[mode];
    rfoot_traj_block.datatypes = {"rfoot_x",    "rfoot_y",    "rfoot_z",
                                  "rfoot_xdot", "rfoot_ydot", "rfoot_zdot"};

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
    converted_trajectories.push_back(com_traj_block);
    converted_trajectories.push_back(pelvis_orientation_block);
    trajectory_names.push_back(lfoot_traj_block.traj_name);
    trajectory_names.push_back(rfoot_traj_block.traj_name);
    trajectory_names.push_back(com_traj_block.traj_name);
    trajectory_names.push_back(pelvis_orientation_block.traj_name);
  }

  auto processed_traj = LcmTrajectory(converted_trajectories, trajectory_names,
                                      "walking_trajectory",
                                      "Output trajectories "
                                      "for Cassie walking");

  processed_traj.WriteToFile(FLAGS_folder_path + FLAGS_trajectory_name +
                             "_processed");
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain();
}