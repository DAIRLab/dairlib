#include "examples/Cassie/cassie_fixed_point_solver.h"
#include <iostream>

#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_solvers.h"
#include "multibody/multibody_utils.h"
#include "multibody/multipose_visualizer.h"
#include "solvers/constraint_factory.h"
#include "solvers/optimization_utils.h"

#include "drake/common/text_logging.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"

namespace dairlib {

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::multibody::MultibodyPlant;
using multibody::KinematicEvaluatorSet;
using solvers::NonlinearConstraint;

void CassieFixedPointSolver(
    const drake::multibody::MultibodyPlant<double>& plant, double height,
    double mu, double min_normal_force, bool linear_friction_cone,
    double toe_spread, VectorXd* q_result, VectorXd* u_result,
    VectorXd* lambda_result, std::string visualize_model_urdf,
    double ground_incline, VectorXd* all_sol) {
  // Get the rotational matrix
  Eigen::AngleAxisd yaw_angle(0, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitch_angle(ground_incline, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd roll_angle(0, Eigen::Vector3d::UnitX());
  Eigen::Quaternion<double> quat = yaw_angle * pitch_angle * roll_angle;
  Eigen::Matrix3d rotation_mat = quat.matrix();
  rotation_mat = rotation_mat.transpose();

  multibody::KinematicEvaluatorSet<double> evaluators(plant);

  // Add loop closures
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  // Add contact points
  std::vector<int> yz_active_idx = {1, 2};
  std::vector<int> z_active_idx = {2};
  auto left_toe = LeftToeFront(plant);
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant, left_toe.first, left_toe.second, rotation_mat,
      Eigen::Vector3d(0, toe_spread, 0), yz_active_idx);
  evaluators.add_evaluator(&left_toe_evaluator);

  auto left_heel = LeftToeRear(plant);
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant, left_heel.first, left_heel.second, rotation_mat,
      Eigen::Vector3d::Zero(), z_active_idx);
  evaluators.add_evaluator(&left_heel_evaluator);

  auto right_toe = RightToeFront(plant);
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant, right_toe.first, right_toe.second, rotation_mat,
      Eigen::Vector3d(0, -toe_spread, 0), yz_active_idx);
  evaluators.add_evaluator(&right_toe_evaluator);

  auto right_heel = RightToeRear(plant);
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant, right_heel.first, right_heel.second, rotation_mat,
      Eigen::Vector3d::Zero(), z_active_idx);
  evaluators.add_evaluator(&right_heel_evaluator);

  auto program = multibody::MultibodyProgram(plant);


  auto positions_map = multibody::MakeNameToPositionsMap(plant);
  auto q = program.AddPositionVariables();
  auto u = program.AddInputVariables();
  auto lambda = program.AddConstraintForceVariables(evaluators);
  auto kinematic_constraint =
      program.AddKinematicPositionConstraint(evaluators, q);
  auto fp_constraint =
      program.AddFixedPointConstraint(evaluators, q, u, lambda);
  program.AddJointLimitConstraints(q);

  // Fix floating base
  program.AddConstraint(q(positions_map.at("base_qw")) == 1);
  program.AddConstraint(q(positions_map.at("base_qx")) == 0);
  program.AddConstraint(q(positions_map.at("base_qy")) == 0);
  program.AddConstraint(q(positions_map.at("base_qz")) == 0);

  program.AddConstraint(q(positions_map.at("base_x")) == 0);
  program.AddConstraint(q(positions_map.at("base_y")) == 0);
  program.AddConstraint(q(positions_map.at("base_z")) == height);

  // Add symmetry constraints, and zero roll/pitch on the hip
  program.AddConstraint(q(positions_map.at("knee_left")) ==
                        q(positions_map.at("knee_right")));
  program.AddConstraint(q(positions_map.at("hip_pitch_left")) ==
                        q(positions_map.at("hip_pitch_right")));
  program.AddConstraint(q(positions_map.at("hip_roll_left")) ==
                        -q(positions_map.at("hip_roll_right")));
  program.AddConstraint(q(positions_map.at("hip_yaw_right")) == 0);
  program.AddConstraint(q(positions_map.at("hip_yaw_left")) == 0);

  // Add some contact force constraints: linear version
  if (linear_friction_cone) {
    int num_linear_faces = 40;  // try lots of faces!
    program.AddConstraint(
        solvers::CreateLinearFrictionConstraint(mu, num_linear_faces),
        lambda.segment(2, 3));
    program.AddConstraint(
        solvers::CreateLinearFrictionConstraint(mu, num_linear_faces),
        lambda.segment(5, 3));
    program.AddConstraint(
        solvers::CreateLinearFrictionConstraint(mu, num_linear_faces),
        lambda.segment(8, 3));
    program.AddConstraint(
        solvers::CreateLinearFrictionConstraint(mu, num_linear_faces),
        lambda.segment(11, 3));
  } else {
    // Add some contact force constraints: Lorentz version
    program.AddConstraint(solvers::CreateConicFrictionConstraint(mu),
                          lambda.segment(2, 3));
    program.AddConstraint(solvers::CreateConicFrictionConstraint(mu),
                          lambda.segment(5, 3));
    program.AddConstraint(solvers::CreateConicFrictionConstraint(mu),
                          lambda.segment(8, 3));
    program.AddConstraint(solvers::CreateConicFrictionConstraint(mu),
                          lambda.segment(11, 3));
  }

  // Add minimum normal forces on all contact points
  program.AddConstraint(lambda(4) >= min_normal_force);
  program.AddConstraint(lambda(7) >= min_normal_force);
  program.AddConstraint(lambda(10) >= min_normal_force);
  program.AddConstraint(lambda(13) >= min_normal_force);

  // Set initial guess/cost for q using a vaguely neutral position
  Eigen::VectorXd q_guess = Eigen::VectorXd::Zero(plant.num_positions());
  q_guess(0) = 1;  // quaternion
  q_guess(positions_map.at("base_z")) = height;
  q_guess(positions_map.at("hip_pitch_left")) = 1;
  q_guess(positions_map.at("knee_left")) = -2;
  q_guess(positions_map.at("ankle_joint_left")) = 2;
  q_guess(positions_map.at("toe_left")) = -2;
  q_guess(positions_map.at("hip_pitch_right")) = 1;
  q_guess(positions_map.at("knee_right")) = -2;
  q_guess(positions_map.at("ankle_joint_right")) = 2;
  q_guess(positions_map.at("toe_right")) = -2;

  q_guess += .05 * Eigen::VectorXd::Random(plant.num_positions());

  // Only cost in this program: u^T u
  program.AddQuadraticCost(u.dot(1.0 * u));
  // Added contact forces so that the COM is at the center of support polygon
  program.AddQuadraticCost(lambda(4) * lambda(4));
  program.AddQuadraticCost(lambda(7) * lambda(7));
  program.AddQuadraticCost(lambda(10) * lambda(10));
  program.AddQuadraticCost(lambda(13) * lambda(13));

  // Random guess, except for the positions
  Eigen::VectorXd guess = Eigen::VectorXd::Random(program.num_vars());
  guess.head(plant.num_positions()) = q_guess;
  if (all_sol) {
    std::cout << "set initial guess from all_sol\n";
    guess = *all_sol;
  }

  double tol = 1e-6;
  program.SetSolverOption(drake::solvers::SnoptSolver::id(),
                          "Major optimality tolerance", tol);
  program.SetSolverOption(drake::solvers::SnoptSolver::id(),
                          "Major feasibility tolerance", tol);

  auto start = std::chrono::high_resolution_clock::now();
  const auto result = drake::solvers::Solve(program, guess);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time:" << elapsed.count() << std::endl;

  std::cout << to_string(result.get_solution_result()) << std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() << std::endl;

  // Draw final pose
  if (visualize_model_urdf != "") {
    auto visualizer = multibody::MultiposeVisualizer(visualize_model_urdf, 1);
    visualizer.DrawPoses(result.GetSolution(q));
  }

  //  solvers::CheckGenericConstraints(program, result, tol);

  *q_result = result.GetSolution(q);
  *u_result = result.GetSolution(u);
  *lambda_result = result.GetSolution(lambda);
  if (all_sol) *all_sol = result.GetSolution();
}

void CassieFixedBaseFixedPointSolver(
    const drake::multibody::MultibodyPlant<double>& plant, VectorXd* q_result,
    VectorXd* u_result, VectorXd* lambda_result,
    std::string visualize_model_urdf) {
  multibody::KinematicEvaluatorSet<double> evaluators(plant);

  // Add loop closures
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  auto program = multibody::MultibodyProgram(plant);

  auto positions_map = multibody::MakeNameToPositionsMap(plant);
  auto q = program.AddPositionVariables();
  auto u = program.AddInputVariables();
  auto lambda = program.AddConstraintForceVariables(evaluators);
  auto kinematic_constraint =
      program.AddKinematicPositionConstraint(evaluators, q);
  auto fp_constraint =
      program.AddFixedPointConstraint(evaluators, q, u, lambda);
  program.AddJointLimitConstraints(q);

  // Add symmetry constraints, and zero roll/pitch on the hip
  program.AddConstraint(q(positions_map.at("knee_left")) ==
                        q(positions_map.at("knee_right")));
  program.AddConstraint(q(positions_map.at("hip_pitch_left")) ==
                        q(positions_map.at("hip_pitch_right")));
  program.AddConstraint(q(positions_map.at("hip_roll_left")) == 0);
  program.AddConstraint(q(positions_map.at("hip_roll_right")) == 0);
  program.AddConstraint(q(positions_map.at("hip_yaw_right")) == 0);
  program.AddConstraint(q(positions_map.at("hip_yaw_left")) == 0);

  // Set initial guess/cost for q using a vaguely neutral position
  Eigen::VectorXd q_guess = Eigen::VectorXd::Zero(plant.num_positions());
  q_guess(0) = 1;  // quaternion
  q_guess(positions_map.at("hip_pitch_left")) = 1;
  q_guess(positions_map.at("knee_left")) = -2;
  q_guess(positions_map.at("ankle_joint_left")) = 2;
  q_guess(positions_map.at("toe_left")) = -2;
  q_guess(positions_map.at("hip_pitch_right")) = 1;
  q_guess(positions_map.at("knee_right")) = -2;
  q_guess(positions_map.at("ankle_joint_right")) = 2;
  q_guess(positions_map.at("toe_right")) = -2;

  // Only cost in this program: u^T u
  program.AddQuadraticCost(u.dot(1.0 * u));

  // Random guess, except for the positions
  Eigen::VectorXd guess = Eigen::VectorXd::Random(program.num_vars());
  guess.head(plant.num_positions()) = q_guess;

  auto start = std::chrono::high_resolution_clock::now();
  const auto result = drake::solvers::Solve(program, guess);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;

  // Draw final pose
  if (visualize_model_urdf != "") {
    auto visualizer = multibody::MultiposeVisualizer(visualize_model_urdf, 1);
    visualizer.DrawPoses(result.GetSolution(q));
  }

  *q_result = result.GetSolution(q);
  *u_result = result.GetSolution(u);
  *lambda_result = result.GetSolution(lambda);
}

void CassieInitStateSolver(
    const drake::multibody::MultibodyPlant<double>& plant,
    const VectorXd& pelvis_xy_vel, double height, double mu,
    double min_normal_force, bool linear_friction_cone, double toe_spread,
    double ground_incline, const VectorXd& q_desired, const VectorXd& u_desired,
    const VectorXd& lambda_desired, VectorXd* q_result, VectorXd* v_result,
    VectorXd* u_result, VectorXd* lambda_result) {
  // Get the rotational matrix
  Eigen::AngleAxisd rollAngle(ground_incline, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd yawAngle(0, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitX());
  Eigen::Quaternion<double> quat = rollAngle * yawAngle * pitchAngle;
  Eigen::Matrix3d rotationMatrix = quat.matrix();
  // Get normal direction
  Vector3d ground_normal(sin(ground_incline), 0, cos(ground_incline));

  multibody::KinematicEvaluatorSet<double> evaluators(plant);

  // Add loop closures
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  // Add contact points
  auto left_toe = LeftToeFront(plant);
  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant, left_toe.first, left_toe.second, rotationMatrix,
      Eigen::Vector3d(0, toe_spread, 1e-4), {1, 2});
  evaluators.add_evaluator(&left_toe_evaluator);

  auto left_heel = LeftToeRear(plant);
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant, left_heel.first, left_heel.second, ground_normal,
      Eigen::Vector3d(0, 0, 1e-4), false);
  evaluators.add_evaluator(&left_heel_evaluator);

  auto right_toe = RightToeFront(plant);
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant, right_toe.first, right_toe.second, rotationMatrix,
      Eigen::Vector3d(0, -toe_spread, 0), {1, 2});
  evaluators.add_evaluator(&right_toe_evaluator);

  auto right_heel = RightToeRear(plant);
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant, right_heel.first, right_heel.second, ground_normal,
      Eigen::Vector3d(0, 0, 1e-4), false);
  evaluators.add_evaluator(&right_heel_evaluator);

  auto program = multibody::MultibodyProgram(plant);

  auto positions_map = multibody::MakeNameToPositionsMap(plant);
  auto q = program.AddPositionVariables();
  auto v = program.AddVelocityVariables();
  auto u = program.AddInputVariables();
  auto lambda = program.AddConstraintForceVariables(evaluators);
  auto kinematic_constraints =
      program.AddHolonomicConstraint(evaluators, q, v, u, lambda);
  program.AddJointLimitConstraints(q);


  auto vel_map = multibody::MakeNameToVelocitiesMap(plant);
  int n_v = plant.num_velocities();

  // Fix floating base
  program.AddBoundingBoxConstraint(1, 1, q(positions_map.at("base_qw")));
  program.AddBoundingBoxConstraint(0, 0, q(positions_map.at("base_qx")));
  program.AddBoundingBoxConstraint(0, 0, q(positions_map.at("base_qy")));
  program.AddBoundingBoxConstraint(0, 0, q(positions_map.at("base_qz")));

  program.AddBoundingBoxConstraint(0, 0, q(positions_map.at("base_x")));
  program.AddBoundingBoxConstraint(0, 0, q(positions_map.at("base_y")));
  program.AddBoundingBoxConstraint(height, height,
                                   q(positions_map.at("base_z")));

  program.AddBoundingBoxConstraint(-10, 10, v);

  program.AddBoundingBoxConstraint(0, 0, v(vel_map.at("base_wx")));
  program.AddBoundingBoxConstraint(0, 0, v(vel_map.at("base_wy")));
  program.AddBoundingBoxConstraint(0, 0, v(vel_map.at("base_wz")));

  program.AddBoundingBoxConstraint(pelvis_xy_vel(0), pelvis_xy_vel(0),
                                   v(vel_map.at("base_vx")));
  program.AddBoundingBoxConstraint(pelvis_xy_vel(1), pelvis_xy_vel(1),
                                   v(vel_map.at("base_vy")));
  program.AddBoundingBoxConstraint(0, 0, v(vel_map.at("base_vz")));

  // Add symmetry constraints, and zero roll/pitch on the hip
  program.AddConstraint(q(positions_map.at("knee_left")) ==
                        q(positions_map.at("knee_right")));
  program.AddConstraint(q(positions_map.at("hip_pitch_left")) ==
                        q(positions_map.at("hip_pitch_right")));
  program.AddConstraint(q(positions_map.at("hip_roll_left")) ==
                        -q(positions_map.at("hip_roll_right")));
  program.AddBoundingBoxConstraint(0, 0, q(positions_map.at("hip_yaw_right")));
  program.AddBoundingBoxConstraint(0, 0, q(positions_map.at("hip_yaw_left")));

  // Add some contact force constraints: linear version
  if (linear_friction_cone) {
    int num_linear_faces = 40;  // try lots of faces!
    program.AddConstraint(
        solvers::CreateLinearFrictionConstraint(mu, num_linear_faces),
        lambda.segment(2, 3));
    program.AddConstraint(
        solvers::CreateLinearFrictionConstraint(mu, num_linear_faces),
        lambda.segment(5, 3));
    program.AddConstraint(
        solvers::CreateLinearFrictionConstraint(mu, num_linear_faces),
        lambda.segment(8, 3));
    program.AddConstraint(
        solvers::CreateLinearFrictionConstraint(mu, num_linear_faces),
        lambda.segment(11, 3));
  } else {
    // Add some contact force constraints: Lorentz version
    program.AddConstraint(solvers::CreateConicFrictionConstraint(mu),
                          lambda.segment(2, 3));
    program.AddConstraint(solvers::CreateConicFrictionConstraint(mu),
                          lambda.segment(5, 3));
    program.AddConstraint(solvers::CreateConicFrictionConstraint(mu),
                          lambda.segment(8, 3));
    program.AddConstraint(solvers::CreateConicFrictionConstraint(mu),
                          lambda.segment(11, 3));
  }

  // Add minimum normal forces on all contact points
  program.AddLinearConstraint(lambda(4) >= min_normal_force);
  program.AddLinearConstraint(lambda(7) >= min_normal_force);
  program.AddLinearConstraint(lambda(10) >= min_normal_force);
  program.AddLinearConstraint(lambda(13) >= min_normal_force);

  // Add costs
  double s = 100;
  auto q_cost_binding = program.AddQuadraticErrorCost(
      s * MatrixXd::Identity(q.size(), q.size()), q_desired, q);
  auto u_cost_binding = program.AddQuadraticErrorCost(
      s * 0.001 * MatrixXd::Identity(u.size(), u.size()), u_desired, u);
  auto lambda_cost_binding = program.AddQuadraticErrorCost(
      s * 0.000001 * MatrixXd::Identity(lambda.size(), lambda.size()),
      lambda_desired, lambda);
  auto v_cost_binding = program.AddQuadraticCost(
      v.tail(n_v - 6).dot(s * 0.001 * v.tail(n_v - 6)));
  // auto vdot_cost_binding = program.AddQuadraticCost(vdot.dot(s * 0.001 *
  // vdot));

  // Initial guesses
  program.SetInitialGuessForAllVariables(
      0.01 * Eigen::VectorXd::Random(program.num_vars()));
  program.SetInitialGuess(q, q_desired);
  program.SetInitialGuess(u, u_desired);
  program.SetInitialGuess(lambda, lambda_desired);

  // Snopt settings
  // program.SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
  //                         "../snopt_test.out");
  program.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level", 0);
  program.SetSolverOption(drake::solvers::SnoptSolver::id(),
                          "Major optimality tolerance", 1e-2);
  program.SetSolverOption(drake::solvers::SnoptSolver::id(),
                          "Major feasibility tolerance", 1e-4);

  std::cout << "Start solving" << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  //  drake::solvers::IpoptSolver ipopt_solver;
  //  const auto result = ipopt_solver.Solve(program, guess);
  const auto result = drake::solvers::Solve(program, program.initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time:" << elapsed.count() << std::endl;

  std::cout << to_string(result.get_solution_result()) << std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() << std::endl;

  *q_result = result.GetSolution(q);
  *v_result = result.GetSolution(v);
  *u_result = result.GetSolution(u);
  *lambda_result = result.GetSolution(lambda);

  //  std::cout << "q = " << *q_result << std::endl;
  //  std::cout << "v = " << *v_result << std::endl;
  //  std::cout << "u = " << *u_result << std::endl;
  //  std::cout << "lambda = " << *lambda_result << std::endl;
  //  std::cout << "vdot = " << result.GetSolution(vdot) << std::endl;
  //
  //  std::cout << "q_cost_binding = "
  //            << solvers::EvalCostGivenSolution(result, q_cost_binding)
  //            << std::endl;
  //  std::cout << "u_cost_binding = "
  //            << solvers::EvalCostGivenSolution(result, u_cost_binding)
  //            << std::endl;
  //  std::cout << "lambda_cost_binding = "
  //            << solvers::EvalCostGivenSolution(result, lambda_cost_binding)
  //            << std::endl;
  //  std::cout << "v_cost_binding = "
  //            << solvers::EvalCostGivenSolution(result, v_cost_binding)
  //            << std::endl;
  // //  std::cout << "vdot_cost_binding = "
  // //            << solvers::EvalCostGivenSolution(result, vdot_cost_binding)
  // //            << std::endl;
}

std::pair<VectorXd, VectorXd> GetInitialCassieState(
    const std::string& urdf, bool use_springs, const Vector3d& pelvis_vel,
    double toe_spread, double height) {

  drake::multibody::MultibodyPlant<double> plant(0.0);
  AddCassieMultibody(&plant, nullptr, true, urdf, use_springs, false);
  plant.Finalize();

  multibody::KinematicEvaluatorSet<double> evaluators(plant);

  // Add loop closures
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  // Add contact points
  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  auto right_toe = RightToeFront(plant);
  auto right_heel = RightToeRear(plant);

  auto left_toe_evaluator = multibody::WorldPointEvaluator(
      plant, left_toe.first, left_toe.second, Matrix3d::Identity(),
      Eigen::Vector3d(0, toe_spread, 1e-4), {1, 2});
  auto left_heel_evaluator = multibody::WorldPointEvaluator(
      plant, left_heel.first, left_heel.second, Vector3d::UnitZ(),
      Eigen::Vector3d(0, 0, 1e-4), false);
  auto right_toe_evaluator = multibody::WorldPointEvaluator(
      plant, right_toe.first, right_toe.second, Matrix3d::Identity(),
      Eigen::Vector3d(0, -toe_spread, 0), {1, 2});
  auto right_heel_evaluator = multibody::WorldPointEvaluator(
      plant, right_heel.first, right_heel.second, Vector3d::UnitZ(),
      Eigen::Vector3d(0, 0, 1e-4), false);

  evaluators.add_evaluator(&left_toe_evaluator);
  evaluators.add_evaluator(&left_heel_evaluator);
  evaluators.add_evaluator(&right_toe_evaluator);
  evaluators.add_evaluator(&right_heel_evaluator);

  auto program = multibody::MultibodyProgram(plant);

  auto positions_map = multibody::MakeNameToPositionsMap(plant);
  auto q = program.AddPositionVariables();
  auto v = program.AddVelocityVariables();
  auto u = program.AddInputVariables();
  auto lambda = program.AddConstraintForceVariables(evaluators);
  auto kinematic_constraints =
      program.AddHolonomicConstraint(evaluators, q, v, u, lambda);
  auto fixed_point_constraint = program.AddFixedPointConstraint(
      evaluators, q, u, lambda);
  program.AddJointLimitConstraints(q);

  auto vel_map = multibody::MakeNameToVelocitiesMap(plant);

  // Fix floating base
  program.AddBoundingBoxConstraint(1, 1, q(positions_map.at("base_qw")));
  program.AddBoundingBoxConstraint(0, 0, q(positions_map.at("base_qx")));
  program.AddBoundingBoxConstraint(0, 0, q(positions_map.at("base_qy")));
  program.AddBoundingBoxConstraint(0, 0, q(positions_map.at("base_qz")));
  program.AddBoundingBoxConstraint(0, 0, q(positions_map.at("base_x")));
  program.AddBoundingBoxConstraint(0, 0, q(positions_map.at("base_y")));
  program.AddBoundingBoxConstraint(height, height, q(positions_map.at("base_z")));
  program.AddBoundingBoxConstraint(-10, 10, v);
  program.AddBoundingBoxConstraint(0, 0, v(vel_map.at("base_wx")));
  program.AddBoundingBoxConstraint(0, 0, v(vel_map.at("base_wy")));
  program.AddBoundingBoxConstraint(0, 0, v(vel_map.at("base_wz")));

  program.AddBoundingBoxConstraint(pelvis_vel(0), pelvis_vel(0), v(vel_map.at("base_vx")));
  program.AddBoundingBoxConstraint(pelvis_vel(1), pelvis_vel(1), v(vel_map.at("base_vy")));
  program.AddBoundingBoxConstraint(pelvis_vel(2), pelvis_vel(2), v(vel_map.at("base_vz")));

  // Add symmetry constraints, and zero roll/pitch on the hip
  program.AddConstraint(q(positions_map.at("knee_left")) == q(positions_map.at("knee_right")));
  program.AddConstraint(q(positions_map.at("hip_pitch_left")) == q(positions_map.at("hip_pitch_right")));
  program.AddConstraint(q(positions_map.at("hip_roll_left")) == -q(positions_map.at("hip_roll_right")));
  program.AddBoundingBoxConstraint(0, 0, q(positions_map.at("hip_yaw_right")));
  program.AddBoundingBoxConstraint(0, 0, q(positions_map.at("hip_yaw_left")));


    int nfaces = 20;  // try lots of faces!
    program.AddConstraint(
        solvers::CreateLinearFrictionConstraint(1, nfaces),
        lambda.segment(2, 3));
    program.AddConstraint(
        solvers::CreateLinearFrictionConstraint(1, nfaces),
        lambda.segment(5, 3));
    program.AddConstraint(
        solvers::CreateLinearFrictionConstraint(1, nfaces),
        lambda.segment(8, 3));
    program.AddConstraint(
        solvers::CreateLinearFrictionConstraint(1, nfaces),
        lambda.segment(11, 3));

  program.AddQuadraticCost(0.01 * u.dot(1.0 * u));
  program.AddQuadraticCost(v.dot(1.0 * v));
  // Added contact forces so that the COM is at the center of support polygon
  program.AddQuadraticCost(lambda(4) * lambda(4));
  program.AddQuadraticCost(lambda(7) * lambda(7));
  program.AddQuadraticCost(lambda(10) * lambda(10));
  program.AddQuadraticCost(lambda(13) * lambda(13));

  Eigen::VectorXd q_guess = Eigen::VectorXd::Zero(plant.num_positions());
  q_guess(positions_map.at("base_z")) = height;
  q_guess(positions_map.at("hip_pitch_left")) = 1;
  q_guess(positions_map.at("knee_left")) = -2;
  q_guess(positions_map.at("ankle_joint_left")) = 2;
  q_guess(positions_map.at("toe_left")) = -2;
  q_guess(positions_map.at("hip_pitch_right")) = 1;
  q_guess(positions_map.at("knee_right")) = -2;
  q_guess(positions_map.at("ankle_joint_right")) = 2;
  q_guess(positions_map.at("toe_right")) = -2;

  program.SetInitialGuess(q, q_guess);

  const auto result = drake::solvers::Solve(program, program.initial_guess());
  return { result.GetSolution(q), result.GetSolution(v) };
}

}  // namespace dairlib