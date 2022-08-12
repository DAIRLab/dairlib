#include <cmath>
#include "examples/Cassie/cassie_fixed_point_solver.h"

#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_solvers.h"
#include "solvers/constraint_factory.h"
#include "common/file_utils.h"

#include "drake/multibody/inverse_kinematics/unit_quaternion_constraint.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/snopt_solver.h"

namespace dairlib {

using Eigen::Matrix3d;
using Eigen::MatrixXd;

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector2d;
using drake::multibody::MultibodyPlant;
using drake::multibody::Frame;
using drake::systems::Context;

Eigen::VectorXd SolveIK(
    const MultibodyPlant<double> &plant, const VectorXd &q_guess,
    const VectorXd &v_guess, const VectorXd &u_guess,
    Context<double> &context,
    std::pair<const Vector3d &, const Frame<double> &> stance_toe,
    std::pair<const Vector3d &, const Frame<double> &> stance_heel,
    std::pair<const Vector3d &, const Frame<double> &> swing_toe,
    std::pair<const Vector3d &, const Frame<double> &> swing_heel,
    double height,
    const Vector3d &stance_toe_pos, const Vector3d &stance_heel_pos,
    const Vector3d &swing_toe_pos, const Vector3d &swing_heel_pos,
    const Vector2d &pelvis_vel) {

  multibody::KinematicEvaluatorSet<double> evaluators(plant);

  // Add loop closures
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  // Add contact points
  auto stance_toe_evaluator = multibody::WorldPointEvaluator(
      plant, stance_toe.first, stance_toe.second, Matrix3d::Identity(),
      stance_toe_pos, {1, 2});
  evaluators.add_evaluator(&stance_toe_evaluator);

  auto stance_heel_evaluator = multibody::WorldPointEvaluator(
      plant, stance_heel.first, stance_heel.second, Vector3d::UnitZ(),
      stance_heel_pos, false);
  evaluators.add_evaluator(&stance_heel_evaluator);

  auto swing_toe_evaluator = multibody::WorldPointEvaluator(
      plant, swing_toe.first, swing_toe.second, Matrix3d::Identity(),
      swing_toe_pos, {1, 2});
  evaluators.add_evaluator(&swing_toe_evaluator);

  auto swing_heel_evaluator = multibody::WorldPointEvaluator(
      plant, swing_heel.first, swing_heel.second, Vector3d::UnitZ(),
      swing_heel_pos, false);
  evaluators.add_evaluator(&swing_heel_evaluator);

  auto program = multibody::MultibodyProgram(plant);

  auto positions_map = multibody::MakeNameToPositionsMap(plant);
  auto q = program.AddPositionVariables();
  auto u = program.AddInputVariables();
  auto lambda = program.AddConstraintForceVariables(evaluators);
  auto kinematic_constraint = program.AddKinematicConstraint(evaluators, q);
  program.AddJointLimitConstraints(q);

  // Velocity part
  auto vel_map = multibody::MakeNameToVelocitiesMap(plant);
  int n_v = plant.num_velocities();
  auto v = program.NewContinuousVariables(n_v, "v");

  // Equation of motion
//  auto vdot = program.NewContinuousVariables(n_v, "vdot");
//  auto constraint = std::make_shared<VdotConstraint>(plant, evaluators);
//  program.AddConstraint(constraint, {q, v, u, lambda, vdot});

  // Zero velocity on feet
  multibody::KinematicEvaluatorSet<double> contact_evaluators(plant);
  contact_evaluators.add_evaluator(&stance_toe_evaluator);
  contact_evaluators.add_evaluator(&stance_heel_evaluator);
  contact_evaluators.add_evaluator(&swing_toe_evaluator);
  contact_evaluators.add_evaluator(&swing_heel_evaluator);
  auto contact_vel_constraint =
      std::make_shared<BodyPointVelConstraint>(plant, contact_evaluators);
  program.AddConstraint(contact_vel_constraint, {q, v});

  // Fix floating base
  program.AddBoundingBoxConstraint(q_guess.head(4), q_guess.head(4), q.head(4));

  program.AddBoundingBoxConstraint(height, height,
                                   q(positions_map.at("base_z")));

  program.AddBoundingBoxConstraint(-20, 20, v);

  //  program.AddBoundingBoxConstraint(0, 0, v(vel_map.at("base_wx")));
  //  program.AddBoundingBoxConstraint(0, 0, v(vel_map.at("base_wy")));
  //  program.AddBoundingBoxConstraint(0, 0, v(vel_map.at("base_wz")));

  program.AddBoundingBoxConstraint(pelvis_vel(0), pelvis_vel(0),
                                   v(vel_map.at("base_vx")));
  program.AddBoundingBoxConstraint(pelvis_vel(1), pelvis_vel(1),
                                   v(vel_map.at("base_vy")));
  program.AddBoundingBoxConstraint(-0.1, 0.1, v(vel_map.at("base_vz")));


  // Add some contact force constraints: Lorentz version
  double mu = 0.8;
  int num_linear_faces = 8;
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

  // Add minimum normal forces on all contact points
  program.AddLinearConstraint(lambda(4) >= 40);
  program.AddLinearConstraint(lambda(7) >= 40);
  program.AddLinearConstraint(lambda(10) >= 40);
  program.AddLinearConstraint(lambda(13) >= 40);

  // Add costs
  double s = 10;
  auto q_cost_binding = program.AddQuadraticErrorCost(
      s * MatrixXd::Identity(q.size(), q.size()), q_guess, q);
//  auto u_cost_binding = program.AddQuadraticErrorCost(
//      s * 0.001 * MatrixXd::Identity(u.size(), u.size()), u_guess, u);
  auto v_cost_binding = program.AddQuadraticErrorCost(
      s * .1 * MatrixXd::Identity(v.size() - 6, v.size() - 6),
      v_guess.tail(n_v - 6), v.tail(n_v - 6));
  auto w_cost_binding = program.AddQuadraticErrorCost(
      s * 0.1 * Matrix3d::Identity(), v_guess.head(3), v.head(3));


  // Initial guesses
  program.SetInitialGuessForAllVariables(
      0.01 * Eigen::VectorXd::Random(program.num_vars()));
  program.SetInitialGuess(q, q_guess);
  program.SetInitialGuess(v, v_guess);

  // Snopt settings
  program.SetSolverOption(drake::solvers::IpoptSolver::id(),
                          "print_level", 0);
  program.SetSolverOption(drake::solvers::IpoptSolver::id(),
      "tol", 1e-2);
  program.SetSolverOption(drake::solvers::IpoptSolver::id(),
                          "constr_viol_tol", 1e-4);
//  program.SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
//                          "../snopt_test.out");
//  program.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level", 0);
//  program.SetSolverOption(drake::solvers::SnoptSolver::id(),
//                          "Major optimality tolerance",
//                          1e-2);
//  program.SetSolverOption(drake::solvers::SnoptSolver::id(),
//                          "Major feasibility tolerance",
//                          1e-4);

//  std::cout << "Start solving" << std::endl;
//  auto start = std::chrono::high_resolution_clock::now();

//  std::cout << program.initial_guess() << std::endl;
  drake::solvers::IpoptSolver ipopt_solver;
  const auto result = ipopt_solver.Solve(program, program.initial_guess());
//  const auto result = drake::solvers::Solve(program, program.initial_guess());
//  auto finish = std::chrono::high_resolution_clock::now();
//  std::chrono::duration<double> elapsed = finish - start;
//  std::cout << "Solve time:" << elapsed.count() << std::endl;

  std::cout << to_string(result.get_solution_result()) << std::endl;
//  std::cout << "Cost:" << result.get_optimal_cost() << std::endl;

  VectorXd q_result = result.GetSolution(q);
  VectorXd v_result = result.GetSolution(v);
  VectorXd u_result = result.GetSolution(u);
  VectorXd lambda_result = result.GetSolution(lambda);
  VectorXd sol(q_result.size() + v_result.size());
  sol << q_result, v_result;
  return sol;
}

void FindInitialStates(
    const MultibodyPlant<double> &plant, const VectorXd &q_init,
    const VectorXd &v_init, const VectorXd &u_init,
    Context<double> *context,
    std::pair<const Vector3d &, const Frame<double> &> stance_toe,
    std::pair<const Vector3d &, const Frame<double> &> stance_heel,
    std::pair<const Vector3d &, const Frame<double> &> swing_toe,
    std::pair<const Vector3d &, const Frame<double> &> swing_heel,
    double height_offset_bound,
    double swing_foot_offset_bound_radius,
    double pelvis_vel_xy_bound,
    std::string sol_file_path) {

  /*** Calulate the contact point positions at the original state ***/
  plant.SetPositions(context, q_init);
  Vector3d stance_toe_pos;
  plant.CalcPointsPositions(
      *context, stance_toe.second, stance_toe.first,
      plant.world_frame(), &stance_toe_pos);

  VectorXd q = q_init;
  // subtract foot position in case of state estimate drift
  q(6) -= stance_toe_pos(2);

  plant.SetPositions(context, q);
  plant.SetVelocities(context, v_init);

  MatrixXd stance_points(3, 2);
  MatrixXd stance_pos(3, 2);
  Matrix3d swing_points;
  Matrix3d swing_pos;
  stance_points.col(0) = stance_toe.first;
  stance_points.col(1) = stance_heel.first;
  swing_points.col(0) = swing_toe.first;
  swing_points.col(1) = swing_heel.first;
  swing_points.col(2) = 0.5 * (swing_toe.first + swing_heel.first);
  plant.CalcPointsPositions(*context, stance_toe.second, stance_points,
                            plant.world_frame(), &stance_pos);
  plant.CalcPointsPositions(*context, swing_toe.second, swing_points,
                            plant.world_frame(), &swing_pos);
  Vector3d mid_to_toe_w = swing_points.col(0) - swing_points.col(3);
  Vector3d mid_to_heel_w = -mid_to_toe_w;

  int n_heights = 5;
  int n_swing_radii = 5;
  int n_swing_angle = 5;
  int n_pelvis_vel_angle = 5;
  int n_pelvis_vel = 5;

  VectorXd heights = VectorXd::LinSpaced(
      n_heights, q(6) - height_offset_bound, q(6) + height_offset_bound);
  std::cout << "Heights: " << heights.transpose() << "\n";
  VectorXd radii = VectorXd::LinSpaced(
      n_swing_radii, swing_foot_offset_bound_radius / n_swing_radii,
      swing_foot_offset_bound_radius);
  std::cout << "Radii: " << radii.transpose() << "\n";
  VectorXd swing_angles = VectorXd::LinSpaced(
      n_swing_angle, 0,
      M_2_PI * (double) (n_swing_angle + 1) / (double) (n_swing_angle));
  std::cout << "SWA: " << swing_angles.transpose() << "\n";
  VectorXd pelvis_vel_angles = VectorXd::LinSpaced(
      n_pelvis_vel_angle, 0,
      M_2_PI * (double) (n_pelvis_vel_angle + 1)
          / (double) (n_pelvis_vel_angle));
  std::cout << "PVA: " << pelvis_vel_angles.transpose() << "\n";
  VectorXd pelvis_vel = VectorXd::LinSpaced(
      n_pelvis_vel, pelvis_vel_xy_bound / n_pelvis_vel, pelvis_vel_xy_bound);
  std::cout << "V: " << pelvis_vel.transpose() << "\n";

  int sol_idx = 0;
  for (int idx_h = 0; idx_h < n_heights; idx_h++) {
    for (int idx_rsw = 0; idx_rsw < n_swing_angle; idx_rsw++) {
      for (int idx_asw = 0; idx_asw < n_swing_angle; idx_asw++) {
        for (int idx_vp = 0; idx_vp < n_pelvis_vel; idx_vp++) {
          for (int idx_ap = 0; idx_ap < n_pelvis_vel_angle; idx_ap++) {
            double height = heights(idx_h);
            double r_sw = radii(idx_rsw);
            double a_sw = swing_angles(idx_asw);
            double v = pelvis_vel(idx_vp);
            double ap = pelvis_vel_angles(idx_ap);
            Vector3d swing_mid_pos_adjusted =
                swing_pos.col(2) + r_sw * Vector3d(cos(a_sw), sin(a_sw), 0);
            VectorXd solution = SolveIK(
                plant, q, v_init, u_init, *context, stance_toe,
                stance_heel, swing_toe, swing_heel, height,
                stance_pos.col(0), stance_pos.col(1),
                swing_mid_pos_adjusted + mid_to_toe_w,
                swing_mid_pos_adjusted + mid_to_heel_w,
                v * Vector2d(cos(ap), sin(ap)));
            sol_idx++;
            appendCSV(sol_file_path, solution.transpose());
          }
        }
      }
    }
  }
}

int do_main(){
  std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";

  MultibodyPlant<double> plant(0.0);
  AddCassieMultibody(&plant, nullptr, true, urdf, true, false);
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  VectorXd x(45);
  x <<  9.99999994e-01, -9.75672206e-05,  6.00044143e-05,  1.51191670e-05,
      1.64820876e-05, -6.69671293e-06,  9.50003136e-01,  2.07290598e-02,
      -2.65128678e-05,  5.43385750e-01, -1.31539589e+00, -4.01696242e-02,
      1.62053942e+00, -2.84294141e-02, -1.68171234e+00, -2.03198419e-02,
      -3.21608639e-05,  5.43664121e-01, -1.31809820e+00, -3.73206371e-02,
      1.62025623e+00, -2.91660067e-02, -1.68184813e+00, -1.92531741e-01,
      6.27865751e-02,  2.79565936e-02,  7.43566473e-03, -6.64940231e-03,
      7.30699664e-04,  2.01833534e-01, -2.64304714e-02, -1.50072302e-01,
      6.71744632e-01, -5.43207028e-01,  1.34859689e-01,  3.61006843e-01,
      -4.96650921e-02,  2.01542101e-01, -2.86009955e-02,  1.53678093e-01,
      -2.00170497e+00,  2.19799862e+00, -1.57034350e-01, -2.83855002e-01,
      -1.26584468e-01;
  FindInitialStates(plant, x.head(23), x.tail(22), VectorXd::Zero(10),
                    context.get(), LeftToeFront(plant), LeftToeRear(plant),
                    RightToeFront(plant), RightToeRear(plant), 0.05, 0.15, 1.0,
                    ".learning_data/ic_creation_test.csv");
  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::do_main();
}


