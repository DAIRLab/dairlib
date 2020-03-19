#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/analysis/simulator.h>
#include <gflags/gflags.h>
#include "examples/Cassie/cassie_utils.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
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

DEFINE_double(realtime_rate, 1.0, "Relative speed to animate the trajectory");
DEFINE_string(trajectory_name, "", "File path to load the trajectory from");

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
  plant.mutable_gravity_field().set_gravity_vector(-9.81 *
                                                   Eigen::Vector3d::UnitZ());
  plant.Finalize();

  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();

  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int nu = plant.num_actuators();
  int nx = plant.num_positions() + plant.num_velocities();

  const LcmTrajectory& loadedTrajs = LcmTrajectory(
      "/home/yangwill/Documents/research/projects/cassie/jumping/saved_trajs/" +
      FLAGS_trajectory_name);
  //      "/home/yangwill/Documents/research/projects/cassie/jumping/saved_trajs/"
  //      "March_19_jumping_0.2");
  auto traj_mode0 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u0");
  auto traj_mode1 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u1");
  auto traj_mode2 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u2");

  std::cout << traj_mode0.datapoints.rows() << std::endl;
  //  int knot_points = traj_mode0.datapoints.cols();
  int n_points = traj_mode0.datapoints.cols() + traj_mode1.datapoints.cols() +
                 traj_mode2.datapoints.cols();

  MatrixXd xu(nx + nu, n_points);
  VectorXd times(n_points);
  std::cout << "Mode transition 0 to 1: " << traj_mode0.time_vector.tail(1);
  std::cout << "\nMode transition 1 to 2: " << traj_mode1.time_vector.tail(1);

  xu << traj_mode0.datapoints, traj_mode1.datapoints, traj_mode2.datapoints;

  times << traj_mode0.time_vector, traj_mode1.time_vector,
      traj_mode2.time_vector;

  MatrixXd x = xu.topRows(nx);

  PiecewisePolynomial<double> optimal_traj =
      PiecewisePolynomial<double>::FirstOrderHold(times, x);

  multibody::connectTrajectoryVisualizer(&plant, &builder, &scene_graph,
                                         optimal_traj);

  auto diagram = builder.Build();

  while (true) {
    //    std::this_thread::sleep_for(std::chrono::seconds(2));
    drake::systems::Simulator<double> simulator(*diagram);

    simulator.set_target_realtime_rate(FLAGS_realtime_rate);
    simulator.Initialize();
    simulator.AdvanceTo(optimal_traj.end_time());
  }
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return dairlib::DoMain();
}