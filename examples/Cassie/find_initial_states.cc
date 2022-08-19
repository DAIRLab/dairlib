#include <gflags/gflags.h>
#include <cmath>

#include "examples/Cassie/cassie_fixed_point_solver.h"

#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_solvers.h"
#include "solvers/constraint_factory.h"
#include "common/file_utils.h"

#include "drake/solvers/ipopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/snopt_solver.h"

DEFINE_string(csvloadpath, ".learning_data/hardware_ics.csv",
              "csv to augment initial conditions from");
DEFINE_string(csvsavepath, ".learning_data/augmented_hardware_ics.csv",
              "csv path to save data to");

namespace dairlib {

using Eigen::Matrix3d;
using Eigen::MatrixXd;

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Vector2d;

using drake::multibody::MultibodyPlant;
using drake::solvers::MathematicalProgramResult;
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
    const Vector2d &pelvis_vel, bool* is_success) {

  multibody::KinematicEvaluatorSet<double> evaluators(plant);

  // Add loop closures
  auto left_loop = LeftLoopClosureEvaluator(plant);
  auto right_loop = RightLoopClosureEvaluator(plant);
  evaluators.add_evaluator(&left_loop);
  evaluators.add_evaluator(&right_loop);

  // Add contact points
  auto stance_toe_evaluator = multibody::WorldPointEvaluator(
      plant, stance_toe.first, stance_toe.second,  Vector3d::UnitZ(),
      stance_toe_pos, true);
  evaluators.add_evaluator(&stance_toe_evaluator);

  auto stance_heel_evaluator = multibody::WorldPointEvaluator(
      plant, stance_heel.first, stance_heel.second, Vector3d::UnitZ(),
      stance_heel_pos, true);
  evaluators.add_evaluator(&stance_heel_evaluator);

  auto swing_toe_evaluator = multibody::WorldPointEvaluator(
      plant, swing_toe.first, swing_toe.second,  Vector3d::UnitZ(),
      swing_toe_pos, true);
  evaluators.add_evaluator(&swing_toe_evaluator);

  auto swing_heel_evaluator = multibody::WorldPointEvaluator(
      plant, swing_heel.first, swing_heel.second, Vector3d::UnitZ(),
      swing_heel_pos, true);
  evaluators.add_evaluator(&swing_heel_evaluator);

  auto program = multibody::MultibodyProgram(plant);

  auto positions_map = multibody::MakeNameToPositionsMap(plant);
  auto q = program.AddPositionVariables();
  auto v = program.AddVelocityVariables();
  auto u = program.AddInputVariables();
  auto lambda = program.AddConstraintForceVariables(evaluators);
  auto kinematic_constraints = program.AddHolonomicConstraint(
      evaluators, q, v, u, lambda);
  program.AddJointLimitConstraints(q);

  // Velocity part
  auto vel_map = multibody::MakeNameToVelocitiesMap(plant);
  int n_v = plant.num_velocities();

  // Fix floating base
  program.AddBoundingBoxConstraint(q_guess.head(4), q_guess.head(4), q.head(4));

  program.AddBoundingBoxConstraint(height, height,
                                   q(positions_map.at("base_z")));

  program.AddBoundingBoxConstraint(-20, 20, v);

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
  program.AddLinearConstraint(lambda(4) >= 50);
  program.AddLinearConstraint(lambda(7) >= 50);
  program.AddLinearConstraint(lambda(10) >= 50);
  program.AddLinearConstraint(lambda(13) >= 50);

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

  //solver settings
  bool use_ipopt = false;
  MathematicalProgramResult result;
  if (use_ipopt) {
    program.SetSolverOption(drake::solvers::IpoptSolver::id(),
                            "print_level", 0);
    program.SetSolverOption(drake::solvers::IpoptSolver::id(),
                            "tol", 1e-2);
    program.SetSolverOption(drake::solvers::IpoptSolver::id(),
                            "constr_viol_tol", 1e-3);
    drake::solvers::IpoptSolver ipopt_solver;
    result = ipopt_solver.Solve(program, program.initial_guess());
  } else {
    program.SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file",
                          "../snopt_test.out");
    program.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level", 0);
    program.SetSolverOption(drake::solvers::SnoptSolver::id(),
                          "Major optimality tolerance",
                          1e-2);
    program.SetSolverOption(drake::solvers::SnoptSolver::id(),
                          "Major feasibility tolerance",
                          1e-3);
    result = drake::solvers::Solve(program, program.initial_guess());
  }

  std::cout << to_string(result.get_solution_result());
  *is_success = result.is_success();

  VectorXd q_result = result.GetSolution(q);
  VectorXd v_result = result.GetSolution(v);
  VectorXd u_result = result.GetSolution(u);
  VectorXd lambda_result = result.GetSolution(lambda);
  VectorXd sol(q_result.size() + v_result.size());
  sol << q_result, v_result;

  if (*is_success) {
    plant.SetPositionsAndVelocities(&context, sol);
    drake::multibody::SpatialVelocity<double> foot_vel_right =
        plant.GetBodyByName("toe_right").EvalSpatialVelocityInWorld(context);
    drake::multibody::SpatialVelocity<double> foot_vel_left =
        plant.GetBodyByName("toe_right").EvalSpatialVelocityInWorld(context);
    *is_success =
        (foot_vel_left.translational().norm() < 0.06) &&
            (foot_vel_right.translational().norm() < 0.06);
    std::cout << (*is_success ? ": Validation Successful" : ": Validation failed") << std::endl;
  } else {
    std::cout << std::endl;
  }

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
    double height_offset_bound, double swing_foot_offset_bound_radius,
    double pelvis_vel_xy_bound, std::string sol_file_path,
    int n_heights, int n_swing_radii, int n_swing_angle, int n_pelvis_vel_angle,
    int n_pelvis_vel) {

  /*** Calulate the contact point positions at the original state ***/
  plant.SetPositions(context, q_init);
  Vector3d stance_toe_pos;
  plant.CalcPointsPositions(
      *context, stance_toe.second, stance_toe.first,
      plant.world_frame(), &stance_toe_pos);

  VectorXd q = q_init;
  // subtract foot position in case of state estimate drift
  q.segment(4, 3) -= stance_toe_pos;

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

  double h_nom = 0.95;
  VectorXd heights = VectorXd::LinSpaced(
      n_heights, h_nom - height_offset_bound, h_nom + height_offset_bound);
  std::cout << "Heights: " << heights.transpose() << "\n";
  VectorXd radii = VectorXd::LinSpaced(
      n_swing_radii, swing_foot_offset_bound_radius / n_swing_radii,
      swing_foot_offset_bound_radius);
  std::cout << "Radii: " << radii.transpose() << "\n";
  VectorXd swing_angles = VectorXd::LinSpaced(
      n_swing_angle, 0,
      2 *M_PI * ((double) (n_swing_angle) / (double) (n_swing_angle + 1)));
  std::cout << "SWA: " << swing_angles.transpose() << "\n";
  VectorXd pelvis_vel_angles = VectorXd::Zero(10);
  pelvis_vel_angles << 0, M_PI / 6, M_PI / 3, M_PI / 2, 2* M_PI / 3, 5* M_PI / 6, M_PI, 5 * M_PI / 4, 3 * M_PI / 2, 7 * M_PI / 4;
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
            auto noise = VectorXd::Random(5);
            double height = heights(idx_h) + 0.025 * noise(0);
            double r_sw = radii(idx_rsw) + 0.025 * noise(1);
            double a_sw = swing_angles(idx_asw) + 0.174553 * noise(2);
            double v = pelvis_vel(idx_vp) + 0.2 * noise(3);
            double ap = pelvis_vel_angles(idx_ap) + 0.26 *  noise(4);
            bool is_success;
            Vector3d swing_mid_pos_adjusted =
                swing_pos.col(2) + r_sw * Vector3d(cos(a_sw), sin(a_sw), 0);
            std::cout << sol_idx << ": ";
            VectorXd solution = SolveIK(
                plant, q, v_init, u_init, *context, stance_toe,
                stance_heel, swing_toe, swing_heel, height,
                stance_pos.col(0), stance_pos.col(1),
                swing_mid_pos_adjusted + mid_to_toe_w,
                swing_mid_pos_adjusted + mid_to_heel_w,
                v * Vector2d(cos(ap), sin(ap)), &is_success);
            sol_idx++;
            if (is_success) {
              appendCSV(sol_file_path, solution.transpose());
            }
          }
        }
      }
    }
  }
}

int do_main(int argc, char* argv[]){
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  MultibodyPlant<double> plant(0.0);
  AddCassieMultibody(&plant, nullptr, true, urdf, true, false);
  plant.Finalize();
  auto context = plant.CreateDefaultContext();
  auto initial_conditions_reader = CSVReader(FLAGS_csvloadpath);

  while (initial_conditions_reader.has_next()) {
    VectorXd x = initial_conditions_reader.next();
    FindInitialStates(plant, x.head(23), x.tail(22), VectorXd::Zero(10),
                      context.get(), LeftToeFront(plant), LeftToeRear(plant),
                      RightToeFront(plant), RightToeRear(plant), 0.05, 0.15, 1.0,
                      FLAGS_csvsavepath, 4, 4, 6, 10, 4);
  }

  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}


