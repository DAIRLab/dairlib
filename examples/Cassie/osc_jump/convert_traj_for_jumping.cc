#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>
#include "examples/Cassie/cassie_utils.h"
#include "lcm/lcm_trajectory.h"
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

//TEMP
using drake::multibody::JointActuatorIndex;


namespace dairlib {

int DoMain() {
  // Drake system initialization stuff
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");
  MultibodyPlant<double> plant(1e-5);
  //  addCassieMultibody(&plant, &scene_graph, true,
  //                     "examples/Cassie/urdf/cassie_fixed_springs.urdf");
  Parser parser(&plant, &scene_graph);
  parser.AddModelFromFile(
      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"));
  plant.mutable_gravity_field().set_gravity_vector(-9.81 *
                                                   Eigen::Vector3d::UnitZ());
  plant.Finalize();

  std::cout << plant.get_joint_actuator(JointActuatorIndex(0)).effort_limit()
  <<std::endl;

  std::unique_ptr<Context<double>> context = plant.CreateDefaultContext();

  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int nu = plant.num_actuators();
  int nx = plant.num_positions() + plant.num_velocities();

  auto l_toe_frame = &plant.GetBodyByName("toe_left").body_frame();
  auto r_toe_frame = &plant.GetBodyByName("toe_right").body_frame();
  //  auto pelvis_frame = &plant.GetBodyByName("pelvis").body_frame();
  auto world = &plant.world_frame();

  const LcmTrajectory& loadedTrajs = LcmTrajectory(
      "/home/yangwill/Documents/research/projects/cassie/jumping/saved_trajs/"
      "March_3_jumping");
  auto traj_mode0 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u0");
  auto traj_mode1 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u1");
  auto traj_mode2 = loadedTrajs.getTrajectory("cassie_jumping_trajectory_x_u2");

  std::cout << traj_mode0.datapoints.rows() << std::endl;
  DRAKE_ASSERT(nx == traj_mode0.datapoints.rows());
  int n_points = traj_mode0.datapoints.cols() + traj_mode1.datapoints.cols() +
                 traj_mode2.datapoints.cols();

  MatrixXd xu(nx + nu, n_points);
  VectorXd times(n_points);

  xu << traj_mode0.datapoints, traj_mode1.datapoints, traj_mode2.datapoints;
  times << traj_mode0.time_vector, traj_mode1.time_vector,
      traj_mode2.time_vector;

  MatrixXd l_foot_points(6, n_points);
  MatrixXd r_foot_points(6, n_points);
  MatrixXd center_of_mass_points(6, n_points);
  Vector3d zero_offset = Vector3d::Zero();

  std::cout << xu.block(0, 2, nx, 1).size() << std::endl;
  for (unsigned int i = 0; i < times.size(); ++i) {
    plant.SetPositionsAndVelocities(context.get(), xu.block(0, i, nx, 1));
    center_of_mass_points.block(0, i, 3, 1) =
        plant.CalcCenterOfMassPosition(*context);
    Eigen::Ref<Eigen::MatrixXd> l_pos_block = l_foot_points.block(0, i, 3, 1);
    Eigen::Ref<Eigen::MatrixXd> r_pos_block = r_foot_points.block(0, i, 3, 1);
    plant.CalcPointsPositions(*context, *l_toe_frame, zero_offset, *world,
                              &l_pos_block);
    plant.CalcPointsPositions(*context, *r_toe_frame, zero_offset, *world,
                              &r_pos_block);

    MatrixXd J_CoM(3, nv);
    MatrixXd J_l_foot(3, nv);
    MatrixXd J_r_foot(3, nv);
    plant.CalcJacobianCenterOfMassVelocity(
        *context, JacobianWrtVariable::kV, *world, *world, &J_CoM);
    plant.CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV,
                                            *l_toe_frame, zero_offset, *world,
                                            *world, &J_l_foot);
    plant.CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV,
                                            *r_toe_frame, zero_offset, *world,
                                            *world, &J_r_foot);
    center_of_mass_points.block(3, i, 3, 1) = J_CoM * xu.block(nq, i, nv, 1);
    l_foot_points.block(3, i, 3, 1) = J_l_foot * xu.block(nq, i, nv, 1);
    r_foot_points.block(3, i, 3, 1) = J_r_foot * xu.block(nq, i, nv, 1);
  }

  auto lfoot_traj_block = LcmTrajectory::Trajectory();
  lfoot_traj_block.traj_name = "left_foot_trajectory";
  lfoot_traj_block.datapoints = l_foot_points;
  lfoot_traj_block.time_vector = times;
  lfoot_traj_block.datatypes = {"lfoot_x",    "lfoot_y",    "lfoot_z",
                                "lfoot_xdot", "lfoot_ydot", "lfoot_zdot"};

  auto rfoot_traj_block = LcmTrajectory::Trajectory();
  rfoot_traj_block.traj_name = "right_foot_trajectory";
  rfoot_traj_block.datapoints = r_foot_points;
  rfoot_traj_block.time_vector = times;
  rfoot_traj_block.datatypes = {"rfoot_x",    "rfoot_y",    "rfoot_z",
                                "rfoot_xdot", "rfoot_ydot", "rfoot_zdot"};

  auto com_traj_block = LcmTrajectory::Trajectory();
  com_traj_block.traj_name = "center_of_mass_trajectory";
  com_traj_block.datapoints = center_of_mass_points;
  com_traj_block.time_vector = times;
  com_traj_block.datatypes = {"com_x",    "com_y",    "com_z",
                              "com_xdot", "com_ydot", "com_zdot"};

  std::cout << center_of_mass_points.col(0) << std::endl;

  std::vector<LcmTrajectory::Trajectory> trajectories = {
      lfoot_traj_block, rfoot_traj_block, com_traj_block};
  std::vector<std::string> trajectory_names = {
      lfoot_traj_block.traj_name, rfoot_traj_block.traj_name,
      com_traj_block.traj_name};

  auto processed_traj =
      LcmTrajectory(trajectories, trajectory_names, "jumping_trajectory",
                    "Feet trajectories "
                    "for Cassie jumping");

  processed_traj.writeToFile(
      "/home/yangwill/Documents/research/projects/cassie/jumping"
      "/saved_trajs/March_3_jumping_processed");
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  //  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return dairlib::DoMain();
}