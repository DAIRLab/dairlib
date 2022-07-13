#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/analysis/simulator.h>
#include <gflags/gflags.h>

#include "examples/Cassie/cassie_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "multibody/multipose_visualizer.h"
#include "multibody/visualization_utils.h"

#include "drake/multibody/plant/multibody_plant.h"

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::trajectories::PiecewisePolynomial;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

DEFINE_double(realtime_rate, 1.0, "Relative speed to animate the trajectory");
DEFINE_string(folder_path, "", "Folder path of the trajectory");
DEFINE_string(trajectory_name, "", "File path to load the trajectory from");
DEFINE_int32(num_poses, 1, "Number of poses per mode to draw");
DEFINE_int32(visualize_mode, 0,
             "0 - Single animation"
             "1 - Looped animation"
             "2 - Multipose visualizer");
DEFINE_bool(use_transparency, false,
            "Transparency setting for the Multipose visualizer");
DEFINE_bool(use_springs, false,
            "Set to true if the trajectory is for the model with springs");
DEFINE_bool(
    mirror_traj, false,
    "Whether or not to extend the trajectory by mirroring the trajectory. Only "
    "use for periodic trajectories that are symmetric.");

namespace dairlib {

int DoMain() {
  // Drake system initialization stuff
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");
  MultibodyPlant<double> plant(1e-5);
  Parser parser(&plant, &scene_graph);
  parser.AddModelFromFile(
      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"));
  plant.Finalize();

  int nq = plant.num_positions();
  int nv = plant.num_positions();
  int nx = nq + nv;

  auto pos_map = multibody::MakeNameToPositionsMap(plant);
  auto vel_map = multibody::MakeNameToVelocitiesMap(plant);

  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();

  DirconTrajectory saved_traj(plant, FLAGS_folder_path + FLAGS_trajectory_name);
  //  VectorXd time_vector = saved_traj.GetBreaks();

  PiecewisePolynomial<double> optimal_traj =
      saved_traj.ReconstructStateTrajectory();
  std::vector<double> time_vector = optimal_traj.get_segment_times();

  if (FLAGS_mirror_traj) {
    auto mirrored_traj =
        saved_traj.ReconstructMirrorStateTrajectory(optimal_traj.end_time());
    VectorXd x_offset = VectorXd::Zero(nx);
    x_offset(pos_map["base_x"]) =
        optimal_traj.value(optimal_traj.end_time())(pos_map["base_x"]) -
        optimal_traj.value(optimal_traj.start_time())(pos_map["base_x"]);
    std::vector<MatrixXd> x_offset_rep(mirrored_traj.get_segment_times().size(),
                                       x_offset);
    PiecewisePolynomial<double> x_offset_traj =
        PiecewisePolynomial<double>::ZeroOrderHold(
            mirrored_traj.get_segment_times(), x_offset_rep);
    optimal_traj.ConcatenateInTime(mirrored_traj + x_offset_traj);

    x_offset(pos_map["base_x"]) =
        optimal_traj.value(optimal_traj.end_time())(pos_map["base_x"]) -
        optimal_traj.value(optimal_traj.start_time())(pos_map["base_x"]);
    x_offset_rep = std::vector<MatrixXd>(
        optimal_traj.get_segment_times().size(), x_offset);

    PiecewisePolynomial<double> copy = optimal_traj;
    copy.shiftRight(optimal_traj.end_time());
    x_offset_traj = PiecewisePolynomial<double>::ZeroOrderHold(
        copy.get_segment_times(), x_offset_rep);
    optimal_traj.ConcatenateInTime(copy + x_offset_traj);
  }

  if (FLAGS_visualize_mode == 0 || FLAGS_visualize_mode == 1) {
    multibody::ConnectTrajectoryVisualizer(&plant, &builder, &scene_graph,
                                           optimal_traj);

    auto diagram = builder.Build();

    do {
      drake::systems::Simulator<double> simulator(*diagram);

      simulator.set_target_realtime_rate(FLAGS_realtime_rate);
      simulator.Initialize();
      simulator.AdvanceTo(optimal_traj.end_time());
    } while (FLAGS_visualize_mode == 1);
  } else if (FLAGS_visualize_mode == 2) {
    MatrixXd poses = MatrixXd::Zero(nq, FLAGS_num_poses);
    for (int i = 0; i < FLAGS_num_poses; ++i) {
      poses.col(i) = optimal_traj.value(
          time_vector[i * time_vector.size() / FLAGS_num_poses]);
    }
    if (FLAGS_use_transparency) {
      VectorXd alpha_scale = VectorXd::LinSpaced(FLAGS_num_poses, 0.2, 1.0);
      multibody::MultiposeVisualizer visualizer =
          multibody::MultiposeVisualizer(
              FindResourceOrThrow(
                  "examples/Cassie/urdf/cassie_fixed_springs.urdf"),
              FLAGS_num_poses, alpha_scale.array().square());
      visualizer.DrawPoses(poses);
    } else {
      multibody::MultiposeVisualizer visualizer =
          multibody::MultiposeVisualizer(
              FindResourceOrThrow(
                  "examples/Cassie/urdf/cassie_fixed_springs.urdf"),
              FLAGS_num_poses);
      visualizer.DrawPoses(poses);
    }
  }

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return dairlib::DoMain();
}
