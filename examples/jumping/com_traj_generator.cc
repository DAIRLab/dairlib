//
// Created by yangwill on 11/13/19.
//
#include <chrono>
#include <fstream>
#include <gflags/gflags.h>

#include "attic/multibody/rigidbody_utils.h"

#include "drake/multibody/rigid_body_tree_construction.h"

#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solution_result.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"

#include "common/find_resource.h"

#include "examples/jumping/traj_logger.h"
#include "lcm/lcm_trajectory.h"

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::trajectories::PiecewisePolynomial;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::AngleAxisd;
using std::cout;
using std::endl;
using std::vector;

DEFINE_double(time_offset, 0.0,
              "Length of time (s) to remain in neutral state");
DEFINE_string(filename, "", "Name of file to write COM traj to");
DEFINE_int32(resolution, 200, "Number of timesteps per states");

namespace dairlib {

namespace examples {
namespace jumping {

int doMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string filename = "examples/jumping/five_link_biped.urdf";
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(filename), drake::multibody::joints::kFixed, &tree);

  KinematicsCache<double> cache = tree.CreateKinematicsCache();
  const LcmTrajectory& loaded_traj = LcmTrajectory(LcmTrajectory::loadFromFile(
      "examples/jumping/saved_trajs/jumping_1_4"));
  const LcmTrajectory::Trajectory& jumping_traj =
      loaded_traj.getTrajectory("jumping_trajectory_x_u");
  int num_positions = tree.get_num_positions();
  int num_states = num_positions + tree.get_num_velocities();
  //  cout << jumping_traj.datapoints.topRows(num_states).cols() << endl;
  //  cout << jumping_traj.time_vector.size() << endl;
  const PiecewisePolynomial<double>& datapoints =
      PiecewisePolynomial<double>::Pchip(
          jumping_traj.time_vector,
          jumping_traj.datapoints.topRows(num_states));

  //  for (auto const& element : multibody::makeNameToPositionsMap(tree)) {
  //    cout << element.first << " = " << element.second << endl;
  //  }

  VectorXd q(num_positions);
  Vector3d center_of_mass;
  Vector3d l_foot;
  Vector3d r_foot;
  Vector3d pt_on_foot = VectorXd::Zero(3);
  int l_foot_idx = multibody::GetBodyIndexFromName(tree, "left_foot");
  int r_foot_idx = multibody::GetBodyIndexFromName(tree, "right_foot");

  std::cout << "l_foot_idx: " << l_foot_idx << std::endl;
  std::cout << "r_foot_idx: " << r_foot_idx << std::endl;

  std::vector<double> com_points;
  std::vector<double> l_foot_points;
  std::vector<double> r_foot_points;
  std::vector<double> torso_angle;
  std::vector<double> times;
  com_points.reserve(10000);
  times.reserve(10000);

  for (int i = 0; i < FLAGS_resolution; ++i) {
    q << datapoints.value(0).topRows(num_positions);
    //     Order of states are not the same for multibody and rigid bodies
    dairlib::multibody::SetZeroQuaternionToIdentity(&q);
    cache.initialize(q);
    tree.doKinematics(cache);
    center_of_mass = tree.centerOfMass(cache);
    l_foot = tree.CalcBodyPoseInWorldFrame(cache, tree.get_body(l_foot_idx))
                 .linear()
                 .col(0);
    r_foot = tree.CalcBodyPoseInWorldFrame(cache, tree.get_body(r_foot_idx))
                 .linear()
                 .col(0);
    times.push_back(i * FLAGS_time_offset / FLAGS_resolution);
    //    VectorXd torso_quat(4);
    //    torso_quat << 1, 0, q(2), 0;
    Quaterniond quat;
    quat = AngleAxisd(0, Vector3d::UnitX())
        * AngleAxisd(q(2), Vector3d::UnitY())
        * AngleAxisd(0, Vector3d::UnitZ());
    torso_angle.push_back(quat.w());
    torso_angle.push_back(quat.x());
    torso_angle.push_back(quat.y());
    torso_angle.push_back(quat.z());
    for (int j = 0; j < 3; ++j) {
      com_points.push_back(center_of_mass(j));
      l_foot_points.push_back(l_foot(j) - center_of_mass(j));
      r_foot_points.push_back(r_foot(j) - center_of_mass(j));
    }
  }
  double time_offset =
      times.empty() ? 0
                    : times.back() + datapoints.end_time() / FLAGS_resolution;
  double end_time = datapoints.end_time();

  for (int i = 0; i < FLAGS_resolution; ++i) {
    q << datapoints.value(i * end_time / FLAGS_resolution)
             .topRows(num_positions);
    dairlib::multibody::SetZeroQuaternionToIdentity(&q);
    cache.initialize(q);
    tree.doKinematics(cache);
    center_of_mass = tree.centerOfMass(cache);

    l_foot = tree.transformPoints(cache, pt_on_foot, l_foot_idx, 0);
    r_foot = tree.transformPoints(cache, pt_on_foot, r_foot_idx, 0);

    times.push_back(i * end_time / FLAGS_resolution + time_offset);
    Quaterniond quat;
    quat = AngleAxisd(0, Vector3d::UnitX())
        * AngleAxisd(q(2), Vector3d::UnitY())
        * AngleAxisd(0, Vector3d::UnitZ());
    torso_angle.push_back(quat.w());
    torso_angle.push_back(quat.x());
    torso_angle.push_back(quat.y());
    torso_angle.push_back(quat.z());
    for (int j = 0; j < 3; ++j) {
      com_points.push_back(center_of_mass(j));
      l_foot_points.push_back(l_foot(j) - center_of_mass(j));
      r_foot_points.push_back(r_foot(j) - center_of_mass(j));
    }
  }

  std::cout << "Creating matrix " << std::endl;

  MatrixXd com_pos_matrix = Eigen::Map<const Matrix<double, Dynamic, Dynamic>>(
      com_points.data(), 3, com_points.size() / 3);
  MatrixXd l_foot_matrix = Eigen::Map<const Matrix<double, Dynamic, Dynamic>>(
      l_foot_points.data(), 3, l_foot_points.size() / 3);
  MatrixXd r_foot_matrix = Eigen::Map<const Matrix<double, Dynamic, Dynamic>>(
      r_foot_points.data(), 3, r_foot_points.size() / 3);
  MatrixXd torso_angle_mat = Eigen::Map<const Matrix<double, Dynamic, Dynamic>>(
      torso_angle.data(), 4, torso_angle.size() / 4);
  VectorXd time_matrix = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
      times.data(), times.size());

  std::cout << "l_foot_matrix size: " << l_foot_matrix.size() << std::endl;
  std::cout << "torso_angle_mat size: " << torso_angle_mat.size() << std::endl;
  std::cout << "time_matrix size: " << time_matrix.size() << std::endl;

  LcmTrajectory::Trajectory com_traj_block;
  com_traj_block.traj_name = "center_of_mass_trajectory";
  com_traj_block.datapoints = com_pos_matrix;
  com_traj_block.time_vector = time_matrix;
  com_traj_block.datatypes = {"com_x", "com_y", "com_z"};
  LcmTrajectory::Trajectory l_foot_traj_block;
  l_foot_traj_block.traj_name = "left_foot_trajectory";
  l_foot_traj_block.datapoints = l_foot_matrix;
  l_foot_traj_block.time_vector = time_matrix;
  l_foot_traj_block.datatypes = {"l_foot_x", "l_foot_y", "l_foot_z"};
  LcmTrajectory::Trajectory r_foot_traj_block;
  r_foot_traj_block.traj_name = "right_foot_trajectory";
  r_foot_traj_block.datapoints = r_foot_matrix;
  r_foot_traj_block.time_vector = time_matrix;
  r_foot_traj_block.datatypes = {"r_foot_x", "r_foot_y", "r_foot_z"};
  LcmTrajectory::Trajectory torso_angle_traj_block;
  torso_angle_traj_block.traj_name = "torso_trajectory";
  torso_angle_traj_block.datapoints = torso_angle_mat;
  torso_angle_traj_block.time_vector = time_matrix;
  torso_angle_traj_block.datatypes = {"quat_w", "quat_x", "quat_y", "quat_z"};

  std::cout << "torso_angle_mat rows: "
            << torso_angle_traj_block.datapoints.rows() << std::endl;
  std::cout << "torso_angle_mat cols: "
            << torso_angle_traj_block.datapoints.cols() << std::endl;

  std::vector<LcmTrajectory::Trajectory> trajectories;
  trajectories.push_back(com_traj_block);
  trajectories.push_back(l_foot_traj_block);
  trajectories.push_back(r_foot_traj_block);
  trajectories.push_back(torso_angle_traj_block);
  std::vector<std::string> trajectory_names;
  trajectory_names.push_back("center_of_mass_trajectory");
  trajectory_names.push_back("left_foot_trajectory");
  trajectory_names.push_back("right_foot_trajectory");
  trajectory_names.push_back("torso_trajectory");
  LcmTrajectory saved_traj(trajectories, trajectory_names, "jumping_trajectory",
                           "Center of mass, and feet trajectory for "
                           "jumping");
  saved_traj.writeToFile("examples/jumping/saved_trajs/" + FLAGS_filename);
  return 0;
}

}  // namespace jumping
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::jumping::doMain(argc, argv);
}
