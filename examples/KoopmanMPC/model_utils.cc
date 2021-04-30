#include "model_utils.h"
#include "multibody/multibody_solvers.h"
#include "multibody/multibody_utils.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "solvers/constraint_factory.h"

#include "drake/solvers/solve.h"

namespace dairlib {

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using multibody::MultibodyProgram;

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::geometry::SceneGraph;

void LoadPlanarWalkerFromFile(MultibodyPlant<double>& plant) {
  Parser parser(&plant);

  std::string plant_file_name = FindResourceOrThrow("examples/PlanarWalker/PlanarWalkerWithTorso.urdf");
  parser.AddModelFromFile(plant_file_name);

  plant.WeldFrames(
      plant.world_frame(), plant.GetFrameByName("base"), drake::math::RigidTransform<double>());
}

void LoadPlanarWalkerFromFile(MultibodyPlant<double>& plant,
    SceneGraph<double>* scene_graph ) {

  Parser parser (&plant, scene_graph);
  std::string full_name = FindResourceOrThrow(
      "examples/PlanarWalker/PlanarWalkerWithTorso.urdf");
  parser.AddModelFromFile(full_name);

  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"),
                   drake::math::RigidTransform<double>());

}

void PlanarWalkerFixedPointSolver(const MultibodyPlant<double>& plant,
                                  const double height, const double foot_spread,
                                  const double mu, VectorXd* q_result,
                                  VectorXd* u_result) {

  multibody::KinematicEvaluatorSet<double> evaluators(plant);

  auto left_pt = std::pair<const drake::multibody::BodyFrame<double> &, Eigen::Vector3d>(
      plant.GetBodyByName("left_lower_leg").body_frame(), Vector3d(0, 0, -0.5));

  auto right_pt = std::pair<const drake::multibody::BodyFrame<double> &, Eigen::Vector3d>(
      plant.GetBodyByName("right_lower_leg").body_frame(), Vector3d(0, 0, -0.5));

  Vector3d foot_contact_disp(0, 0, 0);

  auto left_foot_evaluator = multibody::WorldPointEvaluator(plant,
      left_pt.second, left_pt.first, Matrix3d::Identity(),
      foot_contact_disp,{0, 2});
  evaluators.add_evaluator(&left_foot_evaluator);

  auto right_foot_evaluator = multibody::WorldPointEvaluator(plant,
      right_pt.second, right_pt.first,  Matrix3d::Identity(),
      foot_contact_disp, {0, 2});
  evaluators.add_evaluator(&right_foot_evaluator);

  auto prog = MultibodyProgram(plant);

  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto q = prog.AddPositionVariables();
  auto u = prog.AddInputVariables();
  auto lambda = prog.AddConstraintForceVariables(evaluators);
  auto kinematic_constraints = prog.AddKinematicConstraint(evaluators, q);
  auto fp_contraint = prog.AddFixedPointConstraint(evaluators, q, u, lambda);
  prog.AddJointLimitConstraints(q);

  prog.AddConstraint(q(positions_map.at("planar_z")) == height);
  prog.AddConstraint(q(positions_map.at("left_knee_pin")) >= .1);
  prog.AddConstraint(q(positions_map.at("right_knee_pin")) >= .1);
  prog.AddConstraint(q(positions_map.at("left_hip_pin")) <= -.1);
  prog.AddConstraint(q(positions_map.at("right_hip_pin")) <= -.1);
  prog.AddConstraint(q(positions_map.at("planar_roty")) == 0);

  int n_lin_faces = 10;
  prog.AddConstraint(solvers::CreateLinearFrictionConstraint(mu, n_lin_faces), lambda.segment(0,3));
  prog.AddConstraint(solvers::CreateLinearFrictionConstraint(mu, n_lin_faces), lambda.segment(3,3));

  prog.AddQuadraticCost(u.dot(1.0 * u));

  VectorXd q_guess = Eigen::VectorXd::Zero(plant.num_positions());
  q_guess(positions_map.at("planar_z")) = height;
  q_guess(positions_map.at("left_hip_pin")) = 0.01;
  q_guess(positions_map.at("right_hip_pin")) = -0.01;
  q_guess(positions_map.at("left_knee_pin")) = 0.01;
  q_guess(positions_map.at("right_knee_pin")) = 0.01;

  VectorXd guess = VectorXd::Random(prog.num_vars());
  guess.head(plant.num_positions()) = q_guess;
  const auto result = drake::solvers::Solve(prog, guess);

  *q_result = result.GetSolution(q);
  *u_result = result.GetSolution(u);
  std::cout << "lambda: \n" << result.GetSolution(lambda) << std::endl;
}

}