#include <unistd.h>  // for pausing the program for a few seconds
#include <chrono>
#include <memory>
#include <string>
#include <gflags/gflags.h>

#include "common/file_utils.h"
#include "common/find_resource.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/planning/visualization/ik/FoM_stance_foot_constraint_given_pos.h"
#include "examples/goldilocks_models/planning/visualization/ik/kinematics_constraint_cost.h"
#include "examples/goldilocks_models/planning/visualization/ik/kinematics_constraint_only_pos.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"

using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::cin;
using std::cout;
using std::endl;
using std::shared_ptr;
using std::string;
using std::to_string;
using std::vector;

using drake::MatrixX;
using drake::geometry::SceneGraph;
using drake::math::autoDiffToGradientMatrix;
using drake::math::autoDiffToValueMatrix;
using drake::math::DiscardGradient;
using drake::math::initializeAutoDiff;
using drake::multibody::Body;
using drake::multibody::MultibodyPlant;
using drake::systems::rendering::MultibodyPositionToGeometryPose;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib {
namespace goldilocks_models {
namespace planning {

DEFINE_int32(robot_option, 0, "0: plannar robot. 1: cassie_fixed_spring");
DEFINE_int32(rom_option, 1, "0: LIPM. 1: LIPM with point-mass swing foot");
DEFINE_int32(iter, 298, "Which iteration");
DEFINE_double(dt, 0.1, "Duration per step * 2");
DEFINE_bool(is_do_inverse_kin, false, "Do inverse kinematics for presentation");

DEFINE_bool(is_soft_constraint, true, "Put IK constraint in cost");
DEFINE_double(soft_constraint_weight, 10, "Cost weight for soft constraint");
DEFINE_bool(assign_des_torso_angle, true, "Assign a desired torso angle");
DEFINE_double(d_clearance, 0, "Max angle for swing foot ground clearance");

DEFINE_double(realtime_factor, 1, "Visualization realtime factor");

// This program visualizes the full order model by doing inverse kinematics.
// This program is used with the planning which contains FoM at pre/post impact.
// Therefore, this program is just for presentation usage. (so that it's easy to
// explain to other what you are doing)
void visualizeFullOrderModelTraj(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  DRAKE_DEMAND(FLAGS_robot_option == 0);  // haven't implemented Cassie yet

  // parameters
  const string dir = "../dairlib_data/goldilocks_models/planning/robot_" +
                     to_string(FLAGS_robot_option) + "/";
  const string data_directory = dir + "data/";
  const string model_directory = dir + "models/";

  // Confirmation
  bool start_with_left_stance = true;
  if (start_with_left_stance) {
    cout << "This program assumes the robot start with left stance. ";
  } else {
    cout << "This program assumes the robot start with right stance. ";
  }
  cout << "iter (used for theta) = " << FLAGS_iter << endl;

  cout << "Are the above numbers correct? (Y/N)\n";
  char answer2[1];
  cin >> answer2;
  if (!((answer2[0] == 'Y') || (answer2[0] == 'y'))) {
    cout << "Ending the program, since the numbers are incorrect.\n";
    return;
  } else {
    cout << "Continue the program...\n";
  }

  // Read in some parameters
  int n_step = readCSV(data_directory + string("n_step.csv"))(0, 0);
  int n_nodes = readCSV(data_directory + string("nodes_per_step.csv"))(0, 0);
  cout << "n_step = " << n_step << endl;
  cout << "n_nodes = " << n_nodes << endl;

  // Setup
  bool left_stance = start_with_left_stance;
  double desired_torso = 0;
  if (FLAGS_assign_des_torso_angle) {
    desired_torso = 0.1;
  }
  double d_clearance = FLAGS_d_clearance;

  // Reduced order model
  MultibodyPlant<double> plant(0.0);
  CreateMBP(&plant, FLAGS_robot_option);
  std::unique_ptr<ReducedOrderModel> rom =
      CreateRom(FLAGS_rom_option, FLAGS_robot_option, plant, false);
  ReadModelParameters(rom.get(), model_directory, FLAGS_iter);
  int n_y = rom->n_y();

  // Read in pose
  MatrixXd x0_each_mode = readCSV(data_directory + string("x0_each_mode.csv"));
  MatrixXd xf_each_mode = readCSV(data_directory + string("xf_each_mode.csv"));
  DRAKE_DEMAND(x0_each_mode.cols() == n_step);
  cout << "x0_each_mode = \n" << x0_each_mode << endl;
  cout << "xf_each_mode = \n" << xf_each_mode << endl;

  // Read in states
  MatrixXd states = readCSV(data_directory + string("x_samples.csv"));
  DRAKE_DEMAND(states.cols() == n_step * n_nodes - (n_step - 1));
  DRAKE_DEMAND(n_y * 2 == states.rows());

  // Get foot contacts
  std::vector<std::pair<const Vector3d, const drake::multibody::Frame<double>&>>
      left_contacts = {FiveLinkRobotLeftContact(plant)};
  std::vector<std::pair<const Vector3d, const drake::multibody::Frame<double>&>>
      right_contacts = {FiveLinkRobotRightContact(plant)};
  // Create context
  auto context = plant.CreateDefaultContext();

  // Get the stance foot positions
  MatrixXd stance_foot_pos_each_mode(2, x0_each_mode.cols());
  for (int i = 0; i < n_step; i++) {
    if (start_with_left_stance)
      left_stance = i % 2 == 0;
    else
      left_stance = i % 2 != 0;

    VectorX<double> q0 = x0_each_mode.col(i).head(7);
    // clang-format off
    if (left_stance) {
      plant.SetPositions(context.get(), q0);
      drake::VectorX<double> pt(3);
      const auto& contact = left_contacts.at(0);
      plant.CalcPointsPositions(*context, contact.second, contact.first,
                                plant.world_frame(), &pt);
      stance_foot_pos_each_mode.col(i) << pt(0), pt(2);
    } else {
      plant.SetPositions(context.get(), q0);
      drake::VectorX<double> pt(3);
      const auto& contact = right_contacts.at(0);
      plant.CalcPointsPositions(*context, contact.second, contact.first,
                                plant.world_frame(), &pt);
      stance_foot_pos_each_mode.col(i) << pt(0), pt(2);
    }
    // clang-format on
  }
  cout << "stance_foot_pos_each_mode = \n" << stance_foot_pos_each_mode << endl;

  // Some setup
  int n_q = 7;
  map<string, int> positions_map = multibody::makeNameToPositionsMap(plant);
  MatrixXd q_at_all_knots(7, states.cols());
  for (int i = 0; i < n_step; i++) {
    if (!FLAGS_assign_des_torso_angle) {
      q_at_all_knots.col((n_nodes - 1) * i) = x0_each_mode.col(i);
      q_at_all_knots.col((n_nodes - 1) * (i + 1)) = xf_each_mode.col(i);
    } else {
      double difference;
      VectorXd modified_x0_each_mode = x0_each_mode.col(i);
      difference = desired_torso - modified_x0_each_mode(2);
      modified_x0_each_mode(2) += difference;
      modified_x0_each_mode(3) -= difference;
      modified_x0_each_mode(4) -= difference;
      q_at_all_knots.col((n_nodes - 1) * i) = modified_x0_each_mode;
      VectorXd modified_xf_each_mode = xf_each_mode.col(i);
      difference = desired_torso - modified_xf_each_mode(2);
      modified_xf_each_mode(2) += difference;
      modified_xf_each_mode(3) -= difference;
      modified_xf_each_mode(4) -= difference;
      q_at_all_knots.col((n_nodes - 1) * (i + 1)) = modified_xf_each_mode;
    }
  }

  // Do inverse kinematics
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < n_step; i++) {
    cout << "\n\n\n";

    if (start_with_left_stance)
      left_stance = i % 2 == 0;
    else
      left_stance = i % 2 != 0;
    cout << "left_stance = " << left_stance << endl;

    for (int j = 1; j < n_nodes - 1; j++) {
      MathematicalProgram math_prog;
      auto r = math_prog.NewContinuousVariables(n_y, "r");
      auto q = math_prog.NewContinuousVariables(7, "q");

      // Full order model joint limits
      // cout << "Adding full-order model joint constraint...\n";
      vector<string> l_or_r{"left_", "right_"};
      vector<string> fom_joint_names{"hip_pin", "knee_pin"};
      vector<double> lb_for_fom_joints{-M_PI / 2.0, 5.0 / 180.0 * M_PI};
      vector<double> ub_for_fom_joints{M_PI / 2.0, M_PI / 2.0};
      for (unsigned int k = 0; k < l_or_r.size(); k++) {
        for (unsigned int l = 0; l < fom_joint_names.size(); l++) {
          math_prog.AddLinearConstraint(
              q(positions_map.at(l_or_r[k] + fom_joint_names[l])),
              lb_for_fom_joints[l], ub_for_fom_joints[l]);
          // cout << "(" << positions_map.at(l_or_r[k] + fom_joint_names[l]) <<
          //      ") " << l_or_r[k] + fom_joint_names[l] << ": lb = " <<
          //      lb_for_fom_joints[l] << ", ub = " << ub_for_fom_joints[l] <<
          //      endl;
        }
      }

      // Add RoM-FoM mapping constraints
      // cout << "Adding RoM-FoM mapping constraint...\n";
      VectorXd r_i = states.col(j + (n_nodes - 1) * i).head(n_y);
      cout << "r_i = " << r_i << endl;
      //////////////////////// Hard Constraint Version//////////////////////////
      if (!FLAGS_is_soft_constraint) {
        auto kin_constraint =
            std::make_shared<planning::KinematicsConstraintOnlyPos>(*rom,
                                                                    plant);
        if (left_stance) {
          // math_prog.AddConstraint(kin_constraint, q);
          math_prog.AddConstraint(kin_constraint, {r, q});
        } else {
          VectorXDecisionVariable q_swap(7);
          q_swap << q.segment(0, 3), q.segment(4, 1), q.segment(3, 1),
              q.segment(6, 1), q.segment(5, 1);
          // cout << "q_swap = " << q_swap.transpose() << endl;
          // math_prog.AddConstraint(kin_constraint, q_swap);
          math_prog.AddConstraint(kin_constraint, {r, q_swap});
        }
        // math_prog.AddLinearConstraint(r == r_i);
        // cout << "q = " << q.transpose() << endl;
      }
      //////////////////////////////////////////////////////////////////////////
      //////////////////////// Soft Constraint Version//////////////////////////
      else {
        auto kin_cost = std::make_shared<planning::KinematicsConstraintCost>(
            *rom, plant, FLAGS_soft_constraint_weight);
        if (left_stance) {
          // math_prog.AddCost(kin_cost, q);
          math_prog.AddCost(kin_cost, {r, q});
        } else {
          VectorXDecisionVariable q_swap(7);
          q_swap << q.segment(0, 3), q.segment(4, 1), q.segment(3, 1),
              q.segment(6, 1), q.segment(5, 1);
          // cout << "q_swap = " << q_swap.transpose() << endl;
          // math_prog.AddCost(kin_cost, q_swap);
          math_prog.AddCost(kin_cost, {r, q_swap});
        }
        math_prog.AddLinearConstraint(r == r_i);
      }
      //////////////////////////////////////////////////////////////////////////

      // Add stance foot constraint
      // cout << "Adding full-order model stance foot constraint...\n";
      auto fom_sf_constraint =
          std::make_shared<planning::FomStanceFootConstraintGivenPos>(
              left_stance, n_q, stance_foot_pos_each_mode.col(i));
      math_prog.AddConstraint(fom_sf_constraint, q);

      // Add cost
      // Among the feasible solutions, pick the one that's closest to
      // interpolated_q
      VectorXd interpolated_q =
          x0_each_mode.col(i).head(7) +
          (xf_each_mode.col(i).head(7) - x0_each_mode.col(i).head(7)) * j /
              (n_nodes - 1);
      // cout << "interpolated_q = " << interpolated_q.transpose() << endl;
      //////////////////////////////////////////////////////////////////////////
      // MatrixXd Id = MatrixXd::Identity(1, 1);
      // VectorXd zero_1d_vec = VectorXd::Zero(1);
      // math_prog.AddQuadraticErrorCost(Id, zero_1d_vec, q.segment(2, 1));
      MatrixXd Id = MatrixXd::Identity(7, 7);
      VectorXd zero_7d_vec = VectorXd::Zero(7);
      if (FLAGS_assign_des_torso_angle) {
        double difference = desired_torso - interpolated_q(2);
        interpolated_q(2) += difference;
        interpolated_q(3) -= difference;
        interpolated_q(4) -= difference;
      }
      if (d_clearance > 0) {
        double delta_angle;
        if (j < 0.5 * n_nodes) {
          delta_angle = d_clearance * 2.0 * j / n_nodes;
        } else {
          delta_angle = d_clearance * 2 - d_clearance * 2.0 * j / n_nodes;
        }

        if (left_stance) {
          interpolated_q(6) += delta_angle;
        } else {
          interpolated_q(5) += delta_angle;
        }
      }
      math_prog.AddQuadraticErrorCost(Id, interpolated_q, q);
      //////////////////////////////////////////////////////////////////////////

      // Set initial guess
      // math_prog.SetInitialGuess(r, r_i);
      // math_prog.SetInitialGuess(q, interpolated_q);

      // math_prog.SetSolverOption(drake::solvers::GurobiSolver::id(),
      // "BarConvTol", 1E-9);
      // math_prog.SetSolverOption(drake::solvers::SnoptSolver::id(),
      //                           "Print file", "snopt.out");
      // math_prog.SetSolverOption(drake::solvers::SnoptSolver::id(),
      //                           "Major iterations limit", 10000);
      // math_prog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major
      // feasibility tolerance", 1.0e-14); //1.0e-10
      // math_prog.SetSolverOption(drake::solvers::SnoptSolver::id(), "Minor
      // feasibility tolerance", 1.0e-14); //1.0e-10
      cout << "Start solving...\n";
      const auto result = Solve(math_prog);
      auto solution_result = result.get_solution_result();
      cout << solution_result << " | Cost:" << result.get_optimal_cost()
           << endl;
      VectorXd q_sol = result.GetSolution(q);
      cout << "q_sol = " << q_sol.transpose() << endl;

      q_at_all_knots.col(j + (n_nodes - 1) * i) = q_sol;
      // q_at_all_knots.col(j + (n_nodes - 1) * i) = interpolated_q;
    }
  }
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  cout << "Time to solve IK:" << elapsed.count() << "\n";

  // TODO: When you create the video, you need to adjust the ratio of duration
  //  for each mode. (look at the time)
  //  Right now, we can just set it to be equal time, cause you don't have time.

  writeCSV(data_directory + string("q_at_knots_IK.csv"), q_at_all_knots);

  // Pause the system for 1 second
  if (FLAGS_realtime_factor != 1) {
    usleep(1000000);
  }

  // Create a piecewise polynomial
  double dt = FLAGS_dt;
  std::vector<double> T_breakpoint;
  std::vector<MatrixXd> Y;
  bool add_one_extra_frame_to_pause = false;
  double t0 = 0;
  if (add_one_extra_frame_to_pause) {
    T_breakpoint.push_back(0);
    VectorXd qv(14);
    qv << q_at_all_knots.col(0), VectorXd::Zero(7);
    Y.push_back(qv);
    t0 = 1;
  }
  VectorXd planner_time =
      readCSV(data_directory + string("time_at_knots.csv")).col(0);
  for (int i = 0; i < states.cols(); i++) {
    // T_breakpoint.push_back(t0 + i * dt);
    // cout << t0 + i * dt << endl;
    T_breakpoint.push_back(planner_time(i));
  }
  for (int i = 0; i < states.cols(); i++) {
    VectorXd qv(14);
    qv << q_at_all_knots.col(i), VectorXd::Zero(7);
    Y.push_back(qv);
  }
  PiecewisePolynomial<double> pp_xtraj =
      PiecewisePolynomial<double>::FirstOrderHold(T_breakpoint, Y);

  // Create MBP for visualization
  drake::systems::DiagramBuilder<double> builder;
  MultibodyPlant<double> plant_vis(0.0);
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  Vector3d ground_normal(0, 0, 1);
  CreateMBPForVisualization(&plant_vis, &scene_graph, ground_normal,
                            FLAGS_robot_option);

  // visualizer
  multibody::connectTrajectoryVisualizer(&plant_vis, &builder, &scene_graph,
                                         pp_xtraj);
  auto diagram = builder.Build();
  // while (true)
  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(FLAGS_realtime_factor);
  simulator.Initialize();
  simulator.AdvanceTo(pp_xtraj.end_time());
}
}  // namespace planning
}  // namespace goldilocks_models
}  // namespace dairlib

int main(int argc, char* argv[]) {
  dairlib::goldilocks_models::planning::visualizeFullOrderModelTraj(argc, argv);
  return 0;
}
