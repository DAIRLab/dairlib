#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/analysis/simulator.h>
#include <gflags/gflags.h>

#include "examples/Cassie/cassie_utils.h"
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

  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();

  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int nu = plant.num_actuators();
  int nx = plant.num_positions() + plant.num_velocities();

  const LcmTrajectory& loadedTrajs =
      LcmTrajectory(FLAGS_folder_path + FLAGS_trajectory_name);

  auto traj_mode0 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u0");
  auto traj_mode1 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u1");
  auto traj_mode2 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u2");

  int n_points = traj_mode0.datapoints.cols() + traj_mode1.datapoints.cols() +
                 traj_mode2.datapoints.cols();

  MatrixXd xu(nx + nx + nu, n_points);
  VectorXd times(n_points);

  xu << traj_mode0.datapoints, traj_mode1.datapoints, traj_mode2.datapoints;

  times << traj_mode0.time_vector, traj_mode1.time_vector,
      traj_mode2.time_vector;

  MatrixXd x = xu.topRows(nx);
  MatrixXd xdot = xu.topRows(2 * nx).bottomRows(nx);

  PiecewisePolynomial<double> optimal_traj =
      PiecewisePolynomial<double>::CubicHermite(times, x, xdot);

  if (FLAGS_visualize_mode == 0 || FLAGS_visualize_mode == 1) {
    multibody::connectTrajectoryVisualizer(&plant, &builder, &scene_graph,
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
    std::vector<double> indices = {0, 12, 20};
    for (int i = 0; i < FLAGS_num_poses; ++i) {
      poses.col(i) = optimal_traj.value(times[i * n_points / FLAGS_num_poses]);
    }
    multibody::MultiposeVisualizer visualizer = multibody::MultiposeVisualizer(
        FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"),
        FLAGS_num_poses);
    visualizer.DrawPoses(poses);
  }

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return dairlib::DoMain();
}
