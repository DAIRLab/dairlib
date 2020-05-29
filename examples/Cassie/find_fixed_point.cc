#include <gflags/gflags.h>

#include "examples/Cassie/cassie_utils.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_solvers.h"
#include "multibody/multibody_utils.h"
#include "multibody/multipose_visualizer.h"

#include "drake/common/text_logging.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/snopt_solver.h"

DEFINE_double(height, 1, "Fixed height");

namespace dairlib {

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  srand((unsigned int) time(0));
  drake::logging::set_log_level("err");

  drake::multibody::MultibodyPlant<double> plant(0);
  drake::multibody::Parser parser(&plant);

  std::string full_name =
      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
  parser.AddModelFromFile(full_name);
  plant.mutable_gravity_field().set_gravity_vector(-9.81 *
                                                   Eigen::Vector3d::UnitZ());
  plant.Finalize();

  std::vector<multibody::KinematicEvaluator<double>*> evaluators;

  // Add loop closures
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  evaluators.push_back(&left_loop);
  evaluators.push_back(&right_loop);

  // Add contact points
  auto left_toe = LeftToe(plant);
  auto left_toe_evaluator = multibody::WorldPointEvaluator(plant,
      left_toe.first, left_toe.second, Eigen::Vector3d(0,0,1));
  evaluators.push_back(&left_toe_evaluator);

  auto right_toe = RightToe(plant);
  auto right_toe_evaluator = multibody::WorldPointEvaluator(plant,
      right_toe.first, right_toe.second, Eigen::Vector3d(0,0,1));
  evaluators.push_back(&right_toe_evaluator);

  auto left_heel = LeftHeel(plant);
  auto left_heel_evaluator = multibody::WorldPointEvaluator(plant,
      left_heel.first, left_heel.second, Eigen::Vector3d(0,0,1));
  evaluators.push_back(&left_heel_evaluator);

  auto right_heel = RightHeel(plant);
  auto right_heel_evaluator = multibody::WorldPointEvaluator(plant,
      right_heel.first, right_heel.second, Eigen::Vector3d(0,0,1));
  evaluators.push_back(&right_heel_evaluator);

  auto program = multibody::MultibodyProgram(plant, evaluators);

  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto q = program.AddPositionVariables();
  auto u = program.AddInputVariables();
  auto lambda = program.AddConstraintForceVariables();
  auto kinematic_constraint = program.AddKinematicConstraint(q);
  auto fp_constraint = program.AddFixedPointConstraint(q, u, lambda);
  program.AddJointLimitConstraints(q);

  // Fix floating base
  program.AddConstraint(q(positions_map.at("base_qw")) == 1);
  program.AddConstraint(q(positions_map.at("base_qx")) == 0);
  program.AddConstraint(q(positions_map.at("base_qy")) == 0);
  program.AddConstraint(q(positions_map.at("base_qz")) == 0);

  program.AddConstraint(q(positions_map.at("base_x")) == 0);
  program.AddConstraint(q(positions_map.at("base_y")) == 0);
  program.AddConstraint(q(positions_map.at("base_z")) == FLAGS_height);

  // Add symmetry constraints, and zero roll/pitch on the hip
  program.AddConstraint(q(positions_map.at("knee_left")) ==
      q(positions_map.at("knee_right")));
  program.AddConstraint(q(positions_map.at("hip_pitch_left")) ==
      q(positions_map.at("hip_pitch_right")));
  program.AddConstraint(q(positions_map.at("hip_roll_left")) == 0);
  program.AddConstraint(q(positions_map.at("hip_roll_right")) == 0);
  program.AddConstraint(q(positions_map.at("hip_yaw_right")) == 0);
  program.AddConstraint(q(positions_map.at("hip_yaw_left")) == 0);

  // Add some contact force constraints
  // TODO: move these into a class. Linear constraints much faster than Lorentz
  program.AddConstraint(lambda(4) >= .5*(lambda(2) + lambda(3)));
  program.AddConstraint(lambda(4) >= .5*(-lambda(2) + lambda(3)));
  program.AddConstraint(lambda(4) >= .5*(-lambda(2) + -lambda(3)));
  program.AddConstraint(lambda(4) >= .5*(lambda(2) + -lambda(3)));

  program.AddConstraint(lambda(7) >= .5*(lambda(5) + lambda(6)));
  program.AddConstraint(lambda(7) >= .5*(-lambda(5) + lambda(6)));
  program.AddConstraint(lambda(7) >= .5*(-lambda(5) + -lambda(6)));
  program.AddConstraint(lambda(7) >= .5*(lambda(5) + -lambda(6)));

  program.AddConstraint(lambda(10) >= .5*(lambda(8) + lambda(9)));
  program.AddConstraint(lambda(10) >= .5*(-lambda(8) + lambda(9)));
  program.AddConstraint(lambda(10) >= .5*(-lambda(8) + -lambda(9)));
  program.AddConstraint(lambda(10) >= .5*(lambda(8) + -lambda(9)));

  program.AddConstraint(lambda(13) >= .5*(lambda(11) + lambda(12)));
  program.AddConstraint(lambda(13) >= .5*(-lambda(11) + lambda(12)));
  program.AddConstraint(lambda(13) >= .5*(-lambda(11) + -lambda(12)));
  program.AddConstraint(lambda(13) >= .5*(lambda(11) + -lambda(12)));

  program.AddConstraint(lambda(4) >= 0);
  program.AddConstraint(lambda(7) >= 0);
  program.AddConstraint(lambda(10) >= 0);
  program.AddConstraint(lambda(13) >= 0);

  // Set initial guess/cost for q using a vaguely neutral position
  Eigen::VectorXd q_guess = Eigen::VectorXd::Zero(plant.num_positions());
  q_guess(0) = 1; //quaternion
  q_guess(positions_map.at("base_z")) = FLAGS_height;
  q_guess(positions_map.at("hip_pitch_left")) = 1;
  q_guess(positions_map.at("knee_left")) = -2;
  q_guess(positions_map.at("ankle_joint_left")) = 2;
  q_guess(positions_map.at("toe_left")) = -2;
  q_guess(positions_map.at("hip_pitch_right")) = 1;
  q_guess(positions_map.at("knee_right")) = -2;
  q_guess(positions_map.at("ankle_joint_right")) = 2;
  q_guess(positions_map.at("toe_right")) = -2;
  q_guess += .1*Eigen::VectorXd::Random(plant.num_positions());

  program.AddQuadraticCost(u.dot(1.0 * u));

  Eigen::VectorXd guess = Eigen::VectorXd::Random(program.num_vars());
  guess.head(plant.num_positions()) = q_guess;

  auto visualizer = multibody::MultiposeVisualizer(
      "examples/Cassie/urdf/cassie_fixed_springs.urdf", 1);
  visualizer.DrawPoses(q_guess);

  const auto result = drake::solvers::Solve(program, guess);
      
  std::cout << to_string(result.get_solution_result()) << std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() << std::endl;

  visualizer.DrawPoses(result.GetSolution(q));

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
