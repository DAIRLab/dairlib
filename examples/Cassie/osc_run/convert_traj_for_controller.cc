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
DEFINE_bool(mirror_traj, false,
            "Whether or not to extend the trajectory by mirroring");

namespace dairlib {

/// This program pre-pelvisputes the output trajectories (center of mass, pelvis
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
  int nx = plant.num_positions() + plant.num_velocities();

  auto l_toe_frame = &plant.GetBodyByName("toe_left").body_frame();
  auto r_toe_frame = &plant.GetBodyByName("toe_right").body_frame();
  auto hip_left_frame = &plant.GetBodyByName("hip_left").body_frame();
  auto hip_right_frame = &plant.GetBodyByName("hip_right").body_frame();
  auto pelvis_frame = &plant.GetBodyByName("pelvis").body_frame();
  auto world = &plant.world_frame();

  DirconTrajectory dircon_traj(plant, FLAGS_folder_path + FLAGS_trajectory_name);
  PiecewisePolynomial<double> state_traj =
      dircon_traj.ReconstructStateTrajectory();

  MatrixXd M = dircon_traj.GetTrajectory("mirror_matrix").datapoints;

  double end_time = state_traj.end_time();

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

      pelvis_orientation.block(0, i, 4, 1) = x_i.head(4);
      pelvis_orientation.block(4, i, 4, 1) = xdot_i.head(4);

      MatrixXd J_CoM(3, nv);
      MatrixXd J_l_foot(3, nv);
      MatrixXd J_r_foot(3, nv);
      MatrixXd J_l_hip(3, nv);
      MatrixXd J_r_hip(3, nv);

      plant.CalcJacobianTranslationalVelocity(*context, JacobianWrtVariable::kV,
                                              *pelvis_frame, zero_offset,
                                              *world, *world, &J_CoM);
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
      pelvis_points.block(3, i, 3, 1) = J_CoM * v_i;
      l_foot_points.block(3, i, 3, 1) = J_l_foot * v_i;
      r_foot_points.block(3, i, 3, 1) = J_r_foot * v_i;
      l_hip_points.block(3, i, 3, 1) = J_l_hip * v_i;
      r_hip_points.block(3, i, 3, 1) = J_r_hip * v_i;

      VectorXd JdotV_CoM = plant.CalcBiasTranslationalAcceleration(
          *context, JacobianWrtVariable::kV, *pelvis_frame, zero_offset, *world,
          *world);
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

      pelvis_points.block(6, i, 3, 1) = JdotV_CoM + J_CoM * xdot_i.tail(nv);
      l_foot_points.block(6, i, 3, 1) =
          JdotV_l_foot + J_l_foot * xdot_i.tail(nv);
      r_foot_points.block(6, i, 3, 1) =
          JdotV_r_foot + J_r_foot * xdot_i.tail(nv);
      l_hip_points.block(6, i, 3, 1) = JdotV_l_hip + J_l_hip * xdot_i.tail(nv);
      r_hip_points.block(6, i, 3, 1) = JdotV_r_hip + J_r_hip * xdot_i.tail(nv);
    }

    all_times.push_back(times);
    all_l_foot_points.push_back(l_foot_points);
    all_r_foot_points.push_back(r_foot_points);
    all_l_hip_points.push_back(l_hip_points);
    all_r_hip_points.push_back(r_hip_points);
    all_pelvis_points.push_back(pelvis_points);
    all_pelvis_orientation.push_back(pelvis_orientation);
  }

  if (FLAGS_mirror_traj) {
    double x_offset = state_traj.value(state_traj.end_time())(4) -
                      state_traj.value(state_traj.start_time())(4);
    std::cout << "x_offset: " << x_offset << std::endl;
    // Extended trajectory
    for (int mode = 0; mode < dircon_traj.GetNumModes(); ++mode) {
      if (dircon_traj.GetStateBreaks(mode).size() <= 1) {
        continue;
      }
      VectorXd times =
          dircon_traj.GetStateBreaks(mode) +
          end_time * VectorXd::Ones(dircon_traj.GetStateBreaks(mode).size());
      MatrixXd x_samples = M * dircon_traj.GetStateSamples(mode).topRows(nx);
      MatrixXd xdot_samples =
          M * dircon_traj.GetStateSamples(mode).bottomRows(nx);

      int n_points = times.size();
      MatrixXd l_foot_points(9, n_points);
      MatrixXd r_foot_points(9, n_points);
      MatrixXd l_hip_points(9, n_points);
      MatrixXd r_hip_points(9, n_points);
      MatrixXd pelvis_points(9, n_points);
      MatrixXd pelvis_orientation(12, n_points);

      for (unsigned int i = 0; i < times.size(); ++i) {
        //      VectorXd x_i = M * state_samples.col(i).head(nx);
        //      VectorXd xdot_i = M * state_samples.col(i).tail(nx);
        VectorXd x_i = x_samples.col(i);
        VectorXd xdot_i = xdot_samples.col(i);
        x_i(4) = x_i(4) + x_offset;

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
        plant.CalcPointsPositions(*context, *hip_left_frame, zero_offset,
                                  *world, &l_hip_pos_block);
        plant.CalcPointsPositions(*context, *hip_right_frame, zero_offset,
                                  *world, &r_hip_pos_block);

        pelvis_orientation.block(0, i, 4, 1) = x_i.head(4);
        pelvis_orientation.block(4, i, 4, 1) = xdot_i.head(4);

        MatrixXd J_CoM(3, nv);
        MatrixXd J_l_foot(3, nv);
        MatrixXd J_r_foot(3, nv);
        MatrixXd J_l_hip(3, nv);
        MatrixXd J_r_hip(3, nv);
        plant.CalcJacobianTranslationalVelocity(
            *context, JacobianWrtVariable::kV, *pelvis_frame, zero_offset,
            *world, *world, &J_CoM);
        plant.CalcJacobianTranslationalVelocity(
            *context, JacobianWrtVariable::kV, *l_toe_frame, zero_offset,
            *world, *world, &J_l_foot);
        plant.CalcJacobianTranslationalVelocity(
            *context, JacobianWrtVariable::kV, *r_toe_frame, zero_offset,
            *world, *world, &J_r_foot);
        plant.CalcJacobianTranslationalVelocity(
            *context, JacobianWrtVariable::kV, *hip_left_frame, zero_offset,
            *world, *world, &J_l_hip);
        plant.CalcJacobianTranslationalVelocity(
            *context, JacobianWrtVariable::kV, *hip_right_frame, zero_offset,
            *world, *world, &J_r_hip);

        VectorXd v_i = x_i.tail(nv);
        pelvis_points.block(3, i, 3, 1) = J_CoM * v_i;
        l_foot_points.block(3, i, 3, 1) = J_l_foot * v_i;
        r_foot_points.block(3, i, 3, 1) = J_r_foot * v_i;
        l_hip_points.block(3, i, 3, 1) = J_l_hip * v_i;
        r_hip_points.block(3, i, 3, 1) = J_r_hip * v_i;

        VectorXd JdotV_CoM = plant.CalcBiasTranslationalAcceleration(
            *context, JacobianWrtVariable::kV, *pelvis_frame, zero_offset,
            *world, *world);
        VectorXd JdotV_l_foot = plant.CalcBiasTranslationalAcceleration(
            *context, JacobianWrtVariable::kV, *l_toe_frame, zero_offset,
            *world, *world);
        VectorXd JdotV_r_foot = plant.CalcBiasTranslationalAcceleration(
            *context, JacobianWrtVariable::kV, *r_toe_frame, zero_offset,
            *world, *world);
        VectorXd JdotV_l_hip = plant.CalcBiasTranslationalAcceleration(
            *context, JacobianWrtVariable::kV, *hip_left_frame, zero_offset,
            *world, *world);
        VectorXd JdotV_r_hip = plant.CalcBiasTranslationalAcceleration(
            *context, JacobianWrtVariable::kV, *hip_right_frame, zero_offset,
            *world, *world);

        pelvis_points.block(6, i, 3, 1) = JdotV_CoM + J_CoM * xdot_i.tail(nv);
        l_foot_points.block(6, i, 3, 1) =
            JdotV_l_foot + J_l_foot * xdot_i.tail(nv);
        r_foot_points.block(6, i, 3, 1) =
            JdotV_r_foot + J_r_foot * xdot_i.tail(nv);
        l_hip_points.block(6, i, 3, 1) =
            JdotV_l_hip + J_l_hip * xdot_i.tail(nv);
        r_hip_points.block(6, i, 3, 1) =
            JdotV_r_hip + J_r_hip * xdot_i.tail(nv);
      }

      all_times.push_back(times);
      all_l_foot_points.push_back(l_foot_points);
      all_r_foot_points.push_back(r_foot_points);
      all_l_hip_points.push_back(l_hip_points);
      all_r_hip_points.push_back(r_hip_points);
      all_pelvis_points.push_back(pelvis_points);
      all_pelvis_orientation.push_back(pelvis_orientation);
    }
  }

  std::vector<LcmTrajectory::Trajectory> converted_trajectories;
  std::vector<std::string> trajectory_names;

  std::cout << "num_modes: " << all_times.size() << std::endl;
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
    lhip_traj_block.datatypes = {"lhip_x",    "lhip_y",    "lhip_z",
                                 "lhip_xdot", "lhip_ydot", "lhip_zdot",
                                 "lhip_xddot", "lhip_yddot", "lhip_zddot"};

    auto rhip_traj_block = LcmTrajectory::Trajectory();
    rhip_traj_block.traj_name = "right_hip_trajectory" + std::to_string(mode);
    rhip_traj_block.datapoints = all_r_hip_points[mode];
    rhip_traj_block.time_vector = all_times[mode];
    rhip_traj_block.datatypes = {"rhip_x",    "rhip_y",    "rhip_z",
                                 "rhip_xdot", "rhip_ydot", "rhip_zdot",
                                 "rhip_xddot", "rhip_yddot", "rhip_zddot"};

    auto pelvis_traj_block = LcmTrajectory::Trajectory();
    pelvis_traj_block.traj_name =
        "pelvis_trans_trajectory" + std::to_string(mode);
    pelvis_traj_block.datapoints = all_pelvis_points[mode];
    pelvis_traj_block.time_vector = all_times[mode];
    pelvis_traj_block.datatypes = {
        "pelvis_x",     "pelvis_y",     "pelvis_z",
        "pelvis_xdot",  "pelvis_ydot",  "pelvis_zdot",
        "pelvis_xddot", "pelvis_yddot", "pelvis_zddot"};

    auto pelvis_orientation_block = LcmTrajectory::Trajectory();
    pelvis_orientation_block.traj_name =
        "pelvis_rot_trajectory" + std::to_string(mode);
    pelvis_orientation_block.datapoints = all_pelvis_orientation[mode];
    pelvis_orientation_block.time_vector = all_times[mode];
    pelvis_orientation_block.datatypes = {
        "pelvis_rotw",     "pelvis_rotx",     "pelvis_roty",
        "pelvis_rotz",     "pelvis_rotwdot",  "pelvis_rotxdot",
        "pelvis_rotydot",  "pelvis_rotzdot",  "pelvis_rotwddot",
        "pelvis_rotxddot", "pelvis_rotyddot", "pelvis_rotzddot"};

    converted_trajectories.push_back(lfoot_traj_block);
    converted_trajectories.push_back(rfoot_traj_block);
    converted_trajectories.push_back(lhip_traj_block);
    converted_trajectories.push_back(rhip_traj_block);
    converted_trajectories.push_back(pelvis_traj_block);
    converted_trajectories.push_back(pelvis_orientation_block);
    trajectory_names.push_back(lfoot_traj_block.traj_name);
    trajectory_names.push_back(rfoot_traj_block.traj_name);
    trajectory_names.push_back(lhip_traj_block.traj_name);
    trajectory_names.push_back(rhip_traj_block.traj_name);
    trajectory_names.push_back(pelvis_traj_block.traj_name);
    trajectory_names.push_back(pelvis_orientation_block.traj_name);
  }

  auto processed_traj = LcmTrajectory(converted_trajectories, trajectory_names,
                                      "walking_trajectory",
                                      "Output trajectories "
                                      "for Cassie walking");

  processed_traj.WriteToFile(FLAGS_folder_path + FLAGS_trajectory_name +
                             "_processed_rel");
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::DoMain();
}