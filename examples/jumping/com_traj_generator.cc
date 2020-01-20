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
using Eigen::AngleAxisd;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
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
  const LcmTrajectory& loaded_traj = LcmTrajectory(
      LcmTrajectory::loadFromFile("examples/jumping/saved_trajs/jumping_1_14"));
  const LcmTrajectory::Trajectory& jumping_traj =
      loaded_traj.getTrajectory("jumping_trajectory_x_u");
  int num_positions = tree.get_num_positions();
  int num_velocities = tree.get_num_velocities();
  int num_states = num_positions + num_velocities;
  //  cout << jumping_traj.datapoints.topRows(num_states).cols() << endl;
  //  cout << jumping_traj.time_vector.size() << endl;
  std::cout << jumping_traj.datapoints.rows() << ","
            << jumping_traj.datapoints.cols() << std::endl;
  const PiecewisePolynomial<double>& datapoints =
      PiecewisePolynomial<double>::Pchip(
          jumping_traj.time_vector,
          jumping_traj.datapoints.topRows(num_states));

  //  for (auto const& element : multibody::makeNameToPositionsMap(tree)) {
  //    cout << element.first << " = " << element.second << endl;
  //  }

  VectorXd q(num_positions);
  VectorXd v(num_velocities);
  Vector3d center_of_mass;
  Vector3d com_vel;
  Vector3d l_foot;
  Vector3d r_foot;
  Vector3d torso_center;
  Vector3d hip;
  Vector3d l_foot_vel;
  Vector3d r_foot_vel;
  Vector3d hip_vel;
  Vector3d pt_on_foot = VectorXd::Zero(3);
  int l_foot_idx = multibody::GetBodyIndexFromName(tree, "left_foot");
  int r_foot_idx = multibody::GetBodyIndexFromName(tree, "right_foot");
  int torso_idx = multibody::GetBodyIndexFromName(tree, "torso");
  Vector3d hip_offset;
  hip_offset << 0, 0, -0.2;

  std::cout << "l_foot_idx: " << l_foot_idx << std::endl;
  std::cout << "r_foot_idx: " << r_foot_idx << std::endl;
  std::cout << "torso_idx: " << torso_idx << std::endl;

  std::vector<double> com_points;
  std::vector<double> com_vel_points;
  std::vector<double> l_foot_points;
  std::vector<double> r_foot_points;
  std::vector<double> l_foot_vel_points;
  std::vector<double> r_foot_vel_points;
  std::vector<double> torso_angle;
  std::vector<double> times;

  com_points.reserve(10000);
  times.reserve(10000);

  for (int i = 0; i < FLAGS_resolution; ++i) {
    //    std::cout << "Datapoints shape : " << datapoints.value(0).rows() <<
    //    ","
    //              << datapoints.value(0).cols() << endl;
    q << datapoints.value(0).topRows(num_positions);
    v << datapoints.value(0).bottomRows(num_velocities);
    //     Order of states are not the same for multibody and rigid bodies
    dairlib::multibody::SetZeroQuaternionToIdentity(&q);
    cache.initialize(q, v);
    tree.doKinematics(cache);

    center_of_mass = tree.centerOfMass(cache);
    com_vel = tree.centerOfMassJacobian(cache) * v;
    l_foot = tree.transformPoints(cache, pt_on_foot, l_foot_idx, 0);
    r_foot = tree.transformPoints(cache, pt_on_foot, r_foot_idx, 0);

    hip = tree.transformPoints(cache, hip_offset, torso_idx, 0);

    times.push_back(i * FLAGS_time_offset / FLAGS_resolution);
    Eigen::Matrix3d rot_mat =
        tree.CalcBodyPoseInWorldFrame(cache, tree.get_body(torso_idx)).linear();
    Quaterniond quat(rot_mat);
    //    quat = AngleAxisd(0, Vector3d::UnitX()) *
    //           AngleAxisd(q(2), Vector3d::UnitY()) *
    //           AngleAxisd(0, Vector3d::UnitZ());
    torso_angle.push_back(quat.w());
    torso_angle.push_back(quat.x());
    torso_angle.push_back(quat.y());
    torso_angle.push_back(quat.z());
    for (int j = 0; j < 3; ++j) {
      com_points.push_back(center_of_mass(j));
      //      l_foot_points.push_back(l_foot(j) - center_of_mass(j));
      //      r_foot_points.push_back(l_foot(j) - center_of_mass(j));
      l_foot_points.push_back(l_foot(j) - hip(j));
      r_foot_points.push_back(r_foot(j) - hip(j));
      com_vel_points.push_back(com_vel(j));
      l_foot_vel_points.push_back(0);
      r_foot_vel_points.push_back(0);
    }
  }
  double time_offset =
      times.empty() ? 0
                    : times.back() + datapoints.end_time() / FLAGS_resolution;
  double end_time = datapoints.end_time();

  for (int i = 0; i < FLAGS_resolution; ++i) {
    q << datapoints.value(i * end_time / FLAGS_resolution)
             .topRows(num_positions);
    v << datapoints.value(i * end_time / FLAGS_resolution)
             .bottomRows(num_velocities);
    dairlib::multibody::SetZeroQuaternionToIdentity(&q);
    cache.initialize(q, v);
    tree.doKinematics(cache);

    center_of_mass = tree.centerOfMass(cache);
    com_vel = tree.centerOfMassJacobian(cache) * v;
    l_foot = tree.transformPoints(cache, pt_on_foot, l_foot_idx, 0);
    r_foot = tree.transformPoints(cache, pt_on_foot, r_foot_idx, 0);
    l_foot_vel =
        tree.transformPointsJacobian(cache, pt_on_foot, l_foot_idx, 0, false) *
        v;
    r_foot_vel =
        tree.transformPointsJacobian(cache, pt_on_foot, r_foot_idx, 0, false) *
        v;

    hip = tree.transformPoints(cache, hip_offset, torso_idx, 0);
    hip_vel =
        tree.transformPointsJacobian(cache, hip_offset, torso_idx, 0, false) *
        v;

    times.push_back(i * end_time / FLAGS_resolution + time_offset);
    //    Vector4d quat = tree.relativeQuaternion(cache, torso_idx, 0);
    Eigen::Matrix3d rot_mat =
        tree.CalcBodyPoseInWorldFrame(cache, tree.get_body(torso_idx)).linear();
    Quaterniond quat(rot_mat);
    //    quat = AngleAxisd(0, Vector3d::UnitX()) *
    //           AngleAxisd(q(2), Vector3d::UnitY()) *
    //           AngleAxisd(0, Vector3d::UnitZ());
    torso_angle.push_back(quat.w());
    torso_angle.push_back(quat.x());
    torso_angle.push_back(quat.y());
    torso_angle.push_back(quat.z());
    for (int j = 0; j < 3; ++j) {
      com_points.push_back(center_of_mass(j));
      //      l_foot_points.push_back(l_foot(j) - center_of_mass(j));
      //      r_foot_points.push_back(r_foot(j) - center_of_mass(j));
      l_foot_points.push_back(l_foot(j) - hip(j));
      r_foot_points.push_back(r_foot(j) - hip(j));

      com_vel_points.push_back(com_vel(j));
      //      l_foot_vel_points.push_back(l_foot_vel(j));
      l_foot_vel_points.push_back(l_foot_vel(j) - hip_vel(j));
      //      r_foot_vel_points.push_back(r_foot_vel(j));
      r_foot_vel_points.push_back(r_foot_vel(j) - hip_vel(j));
    }
  }

  std::cout << "Creating matrix " << std::endl;

  MatrixXd com_pos_matrix = Eigen::Map<const Matrix<double, Dynamic, Dynamic>>(
      com_points.data(), 3, com_points.size() / 3);
  MatrixXd l_foot_matrix = Eigen::Map<const Matrix<double, Dynamic, Dynamic>>(
      l_foot_points.data(), 3, l_foot_points.size() / 3);
  MatrixXd r_foot_matrix = Eigen::Map<const Matrix<double, Dynamic, Dynamic>>(
      r_foot_points.data(), 3, r_foot_points.size() / 3);
  MatrixXd com_pos_vel_matrix =
      Eigen::Map<const Matrix<double, Dynamic, Dynamic>>(
          com_vel_points.data(), 3, com_vel_points.size() / 3);
  MatrixXd l_foot_vel_matrix =
      Eigen::Map<const Matrix<double, Dynamic, Dynamic>>(
          l_foot_vel_points.data(), 3, l_foot_vel_points.size() / 3);
  MatrixXd r_foot_vel_matrix =
      Eigen::Map<const Matrix<double, Dynamic, Dynamic>>(
          r_foot_vel_points.data(), 3, r_foot_vel_points.size() / 3);
  MatrixXd torso_angle_mat = Eigen::Map<const Matrix<double, Dynamic, Dynamic>>(
      torso_angle.data(), 4, torso_angle.size() / 4);
  VectorXd time_matrix = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
      times.data(), times.size());

  std::cout << "l_foot_matrix size: " << l_foot_matrix.size() << std::endl;
  std::cout << "l_foot_vel_matrix size: " << l_foot_vel_matrix.size()
            << std::endl;
  std::cout << "com_vel_matrix size: " << com_pos_vel_matrix.size()
            << std::endl;
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
  LcmTrajectory::Trajectory com_vel_traj_block;
  com_vel_traj_block.traj_name = "center_of_mass_vel_trajectory";
  com_vel_traj_block.datapoints = com_pos_vel_matrix;
  com_vel_traj_block.time_vector = time_matrix;
  com_vel_traj_block.datatypes = {"com_xdot", "com_ydot", "com_zdot"};
  LcmTrajectory::Trajectory l_foot_vel_traj_block;
  l_foot_vel_traj_block.traj_name = "left_foot_vel_trajectory";
  l_foot_vel_traj_block.datapoints = l_foot_vel_matrix;
  l_foot_vel_traj_block.time_vector = time_matrix;
  l_foot_vel_traj_block.datatypes = {"l_foot_xdot", "l_foot_ydot",
                                     "l_foot_zdot"};
  LcmTrajectory::Trajectory r_foot_vel_traj_block;
  r_foot_vel_traj_block.traj_name = "right_foot_vel_trajectory";
  r_foot_vel_traj_block.datapoints = r_foot_vel_matrix;
  r_foot_vel_traj_block.time_vector = time_matrix;
  r_foot_vel_traj_block.datatypes = {"r_foot_xdot", "r_foot_ydot",
                                     "r_foot_zdot"};
  LcmTrajectory::Trajectory torso_angle_traj_block;
  torso_angle_traj_block.traj_name = "torso_trajectory";
  torso_angle_traj_block.datapoints = torso_angle_mat;
  torso_angle_traj_block.time_vector = time_matrix;
  torso_angle_traj_block.datatypes = {"quat_w", "quat_x", "quat_y", "quat_z"};

  std::vector<LcmTrajectory::Trajectory> trajectories;
  trajectories.push_back(com_traj_block);
  trajectories.push_back(l_foot_traj_block);
  trajectories.push_back(r_foot_traj_block);
  trajectories.push_back(com_vel_traj_block);
  trajectories.push_back(l_foot_vel_traj_block);
  trajectories.push_back(r_foot_vel_traj_block);
  trajectories.push_back(torso_angle_traj_block);
  std::vector<std::string> trajectory_names;
  trajectory_names.push_back("center_of_mass_trajectory");
  trajectory_names.push_back("left_foot_trajectory");
  trajectory_names.push_back("right_foot_trajectory");
  trajectory_names.push_back("center_of_mass_vel_trajectory");
  trajectory_names.push_back("left_foot_vel_trajectory");
  trajectory_names.push_back("right_foot_vel_trajectory");
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
