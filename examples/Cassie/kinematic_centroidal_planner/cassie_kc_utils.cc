#include "cassie_kc_utils.h"

#include <iostream>
#include <regex>

#include <drake/geometry/scene_graph.h>
#include <drake/multibody/inverse_kinematics/com_position_constraint.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/solvers/solve.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>

#include "examples/Cassie/cassie_utils.h"
#include "multibody/visualization_utils.h"

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

Eigen::VectorXd GenerateNominalStand(
    const drake::multibody::MultibodyPlant<double>& plant, double com_height,
    double stance_width, bool visualize) {
  using Eigen::Vector3d;
  using Eigen::VectorXd;
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_x = n_q + n_v;
  std::map<std::string, int> positions_map =
      dairlib::multibody::MakeNameToPositionsMap(plant);

  Eigen::VectorXd q_ik_guess = Eigen::VectorXd::Zero(n_q);

  std::map<std::string, double> pos_value_map;
  Eigen::Vector4d quat(2000.06, -0.339462, -0.609533, -0.760854);
  quat.normalize();
  pos_value_map["base_qw"] = quat(0);
  pos_value_map["base_qx"] = quat(1);
  pos_value_map["base_qy"] = quat(2);
  pos_value_map["base_qz"] = quat(3);
  pos_value_map["base_x"] = 0.000889849;
  pos_value_map["base_y"] = 0.000626865;
  pos_value_map["base_z"] = 1.0009;
  pos_value_map["hip_roll_left"] = 0.00927845;
  pos_value_map["hip_roll_right"] = 0.00927845;
  pos_value_map["hip_yaw_left"] = -0.000895805;
  pos_value_map["hip_yaw_right"] = 0.000895805;
  pos_value_map["hip_pitch_left"] = 0.610808;
  pos_value_map["hip_pitch_right"] = 0.610808;
  pos_value_map["knee_left"] = -1.35926;
  pos_value_map["knee_right"] = -1.35926;
  pos_value_map["ankle_joint_left"] = 1.00716;
  pos_value_map["ankle_joint_right"] = 1.00716;
  pos_value_map["toe_left"] = -M_PI / 2;
  pos_value_map["toe_right"] = -M_PI / 2;

  for (auto pair : pos_value_map) {
    q_ik_guess(positions_map.at(pair.first)) = pair.second;
  }

  Eigen::Vector3d heel_rt_toe = {.122, 0, 0};

  Eigen::Vector3d pelvis_pos = {0, 0, com_height};
  Eigen::Vector3d l_toe_pos = {0.06, stance_width / 2, 0};
  Eigen::Vector3d r_toe_pos = {0.06, -stance_width / 2, 0};

  Eigen::Vector3d l_heel_pos = l_toe_pos - heel_rt_toe;
  Eigen::Vector3d r_heel_pos = r_toe_pos - heel_rt_toe;

  const auto& world_frame = plant.world_frame();
  const auto& pelvis_frame = plant.GetFrameByName("pelvis");
  const auto& toe_left_frame = plant.GetFrameByName("toe_left");
  const auto& toe_right_frame = plant.GetFrameByName("toe_right");

  auto context = plant.CreateDefaultContext();
  drake::multibody::InverseKinematics ik(plant, context.get());
  double eps = 1e-3;
  Vector3d eps_vec = eps * VectorXd::Ones(3);
  ik.AddOrientationConstraint(
      pelvis_frame, drake::math::RotationMatrix<double>(), world_frame,
      drake::math::RotationMatrix<double>(), eps);
  ik.AddPositionConstraint(toe_left_frame, dairlib::LeftToeFront(plant).first,
                           world_frame, l_toe_pos - eps_vec,
                           l_toe_pos + eps_vec);
  ik.AddPositionConstraint(toe_left_frame, dairlib::LeftToeRear(plant).first,
                           world_frame, l_heel_pos - eps_vec,
                           l_heel_pos + eps_vec);

  ik.AddPositionConstraint(toe_right_frame, dairlib::RightToeFront(plant).first,
                           world_frame, r_toe_pos - eps_vec,
                           r_toe_pos + eps_vec);
  ik.AddPositionConstraint(toe_right_frame, dairlib::RightToeRear(plant).first,
                           world_frame, r_heel_pos - eps_vec,
                           r_heel_pos + eps_vec);

  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("hip_yaw_left")) == 0);
  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("hip_yaw_right")) == 0);
  // Four bar linkage constraint (without spring)
  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("knee_left")) +
          (ik.q())(positions_map.at("ankle_joint_left")) ==
      M_PI * 13 / 180.0);
  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("knee_right")) +
          (ik.q())(positions_map.at("ankle_joint_right")) ==
      M_PI * 13 / 180.0);

  auto constraint = std::make_shared<drake::multibody::ComPositionConstraint>(
      &plant, std::nullopt, plant.world_frame(), context.get());
  auto r = ik.get_mutable_prog()->NewContinuousVariables(3);
  ik.get_mutable_prog()->AddConstraint(constraint, {ik.q(), r});
  Eigen::Vector3d rdes = {0, 0, com_height};
  ik.get_mutable_prog()->AddBoundingBoxConstraint(rdes, rdes, r);

  ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_ik_guess);
  ik.get_mutable_prog()->SetInitialGuess(r, rdes);

  const auto result = drake::solvers::Solve(ik.prog());
  const auto q_sol = result.GetSolution(ik.q());
  VectorXd q_sol_normd(n_q);
  q_sol_normd << q_sol.head(4).normalized(), q_sol.tail(n_q - 4);
  q_ik_guess = q_sol_normd;

  if (visualize) {
    // Build temporary diagram for visualization
    drake::systems::DiagramBuilder<double> builder_ik;
    drake::geometry::SceneGraph<double>& scene_graph_ik =
        *builder_ik.AddSystem<drake::geometry::SceneGraph>();
    scene_graph_ik.set_name("scene_graph_ik");
    MultibodyPlant<double> plant_ik(0.0);
    Parser parser(&plant_ik, &scene_graph_ik);
    std::string full_name = dairlib::FindResourceOrThrow(
        "examples/Cassie/urdf/cassie_fixed_springs.urdf");
    parser.AddModelFromFile(full_name);
    plant_ik.Finalize();

    // Visualize
    VectorXd x_const = VectorXd::Zero(n_x);
    x_const.head(n_q) = q_sol;
    drake::trajectories::PiecewisePolynomial<double> pp_xtraj(x_const);

    dairlib::multibody::ConnectTrajectoryVisualizer(&plant_ik, &builder_ik,
                                                    &scene_graph_ik, pp_xtraj);
    auto diagram = builder_ik.Build();
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(.1);
    simulator.Initialize();
    simulator.AdvanceTo(1.0);
  }

  Eigen::VectorXd rv = Eigen::VectorXd::Zero(n_x);
  rv.head(n_q) = q_ik_guess;
  return rv;
}

std::vector<Complexity> GenerateComplexitySchedule(
    int n_knot_points, const std::vector<std::string>& complexity_string_list) {
  std::vector<Complexity> complexity_schedule;
  std::regex number_regex("(^|\\s)([0-9]+)($|\\s)");
  std::smatch match;
  for (const auto& complexity_string : complexity_string_list) {
    DRAKE_DEMAND(complexity_string.at(0) == 'c' or
                 complexity_string.at(0) == 's');
    if (complexity_string.at(0) == 'c') {
      std::regex_search(complexity_string, match, number_regex);
      std::cout << std::stoi(match[0]) << std::endl;
      for (int i = 0; i < std::stoi(match[0]); i++) {
        complexity_schedule.push_back(Complexity::KINEMATIC_CENTROIDAL);
      }
    } else if (complexity_string.at(0) == 's') {
      std::regex_search(complexity_string, match, number_regex);
      std::cout << std::stoi(match[0]) << std::endl;
      for (int i = 0; i < std::stoi(match[0]); i++) {
        complexity_schedule.push_back(Complexity::SLIP);
      }
    }
  }
  DRAKE_DEMAND(n_knot_points == complexity_schedule.size());
  return complexity_schedule;
}