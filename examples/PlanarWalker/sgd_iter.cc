#include "examples/PlanarWalker/sgd_iter.h"

#include <gflags/gflags.h>

#include <memory>
#include <chrono>



#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/solve.h"

#include "attic/systems/trajectory_optimization/dircon_util.h"

#include "attic/systems/trajectory_optimization/dircon_position_data.h"
#include "attic/systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "attic/systems/trajectory_optimization/dircon_opt_constraints.h"

#include "systems/goldilocks_models/symbolic_manifold.h"
#include "systems/goldilocks_models/file_utils.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using drake::VectorX;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::trajectories::PiecewisePolynomial;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using std::vector;
using std::shared_ptr;
using std::cout;
using std::endl;
using std::string;

/// Inputs: initial trajectory
/// Outputs: trajectory optimization problem
namespace dairlib {
namespace goldilocks_models {

using systems::trajectory_optimization::HybridDircon;
using systems::trajectory_optimization::DirconDynamicConstraint;
using systems::trajectory_optimization::DirconKinematicConstraint;
using systems::trajectory_optimization::DirconOptions;
using systems::trajectory_optimization::DirconKinConstraintType;

shared_ptr<HybridDircon<double>> sgdIter(double stride_length, double duration,
                                         int iter, string directory,
                                         string init_file, string weights_file,
                                         string output_prefix) {
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      "PlanarWalkerWithTorso.urdf", drake::multibody::joints::kFixed, &tree);

// world
// base
// base_x
// base_xz
// torso
// torso_mass
// left_upper_leg
// left_upper_leg_mass
// left_lower_leg
// left_lower_leg_mass
// right_upper_leg
// right_upper_leg_mass
// right_lower_leg
// right_lower_leg_mass

// left_hip_torque
// right_hip_torque
// left_knee_torque
// right_knee_torque

// planar_x
// planar_z
// planar_roty
// left_hip_pin
// left_knee_pin
// right_hip_pin
// right_knee_pin

// planar_xdot
// planar_zdot
// planar_rotydot
// left_hip_pindot
// left_knee_pindot
// right_hip_pindot
// right_knee_pindot


  int n = tree.get_num_positions();

  int leftLegIdx = tree.FindBodyIndex("left_lower_leg");
  int rightLegIdx = tree.FindBodyIndex("right_lower_leg");

  Vector3d pt;
  pt << 0, 0, -.5;
  bool isXZ = true;

  auto leftFootConstraint = DirconPositionData<double>(tree, leftLegIdx, pt,
                                                       isXZ);
  auto rightFootConstraint = DirconPositionData<double>(tree, rightLegIdx, pt,
                                                        isXZ);

  Vector3d normal;
  normal << 0, 0, 1;
  double mu = 1;
  leftFootConstraint.addFixedNormalFrictionConstraints(normal, mu);
  rightFootConstraint.addFixedNormalFrictionConstraints(normal, mu);

  std::vector<DirconKinematicData<double>*> leftConstraints;
  leftConstraints.push_back(&leftFootConstraint);
  auto leftDataSet = DirconKinematicDataSet<double>(tree, &leftConstraints);

  std::vector<DirconKinematicData<double>*> rightConstraints;
  rightConstraints.push_back(&rightFootConstraint);
  auto rightDataSet = DirconKinematicDataSet<double>(tree, &rightConstraints);

  auto leftOptions = DirconOptions(leftDataSet.countConstraints());
  leftOptions.setConstraintRelative(0, true);

  auto rightOptions = DirconOptions(rightDataSet.countConstraints());
  rightOptions.setConstraintRelative(0, true);

  std::vector<int> timesteps;
  timesteps.push_back(20);
  timesteps.push_back(1);
  std::vector<double> min_dt;
  min_dt.push_back(.01);
  min_dt.push_back(.01);
  std::vector<double> max_dt;
  max_dt.push_back(.3);
  max_dt.push_back(.3);

  int N = 1 - timesteps.size();
  for (uint i = 0; i < timesteps.size(); i++) {
    N += timesteps[i];
  }

  std::vector<DirconKinematicDataSet<double>*> dataset_list;
  dataset_list.push_back(&leftDataSet);
  dataset_list.push_back(&rightDataSet);

  std::vector<DirconOptions> options_list;
  options_list.push_back(leftOptions);
  options_list.push_back(rightOptions);

  auto trajopt = std::make_shared<HybridDircon<double>>(tree, timesteps, min_dt,
                                                        max_dt, dataset_list,
                                                        options_list);

  trajopt->AddDurationBounds(duration, duration);

  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Print file", "snopt.out");
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", iter);

  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                           0);

  // Periodicity constraints
  // planar_x - 0
  // planar_z - 1
  // planar_roty - 2
  // left_hip_pin - 3
  // left_knee_pin - 4
  // right_hip_pin - 5
  // right_knee_pin - 6
  auto x0 = trajopt->initial_state();
  // auto xf = trajopt->final_state();
  auto xf = trajopt->state_vars_by_mode(timesteps.size()-1,
                                        timesteps[timesteps.size()-1]-1);

  trajopt->AddLinearConstraint(x0(1) == xf(1));
  trajopt->AddLinearConstraint(x0(2) == xf(2));
  trajopt->AddLinearConstraint(x0(3) == xf(5));
  trajopt->AddLinearConstraint(x0(4) == xf(6));
  trajopt->AddLinearConstraint(x0(5) == xf(3));
  trajopt->AddLinearConstraint(x0(6) == xf(4));

  trajopt->AddLinearConstraint(x0(7) == xf(7));
  trajopt->AddLinearConstraint(x0(8) == xf(8));
  trajopt->AddLinearConstraint(x0(9) == xf(9));
  trajopt->AddLinearConstraint(x0(10) == xf(12));
  trajopt->AddLinearConstraint(x0(11) == xf(13));
  trajopt->AddLinearConstraint(x0(12) == xf(10));
  trajopt->AddLinearConstraint(x0(13) == xf(11));

  // u periodic constraint
  auto u0 = trajopt->input(0);
  auto uf = trajopt->input(N-1);
  trajopt->AddLinearConstraint(u0(0) == uf(1));
  trajopt->AddLinearConstraint(u0(1) == uf(0));
  trajopt->AddLinearConstraint(u0(2) == uf(3));
  trajopt->AddLinearConstraint(u0(3) == uf(2));


  // Knee joint limits
  auto x = trajopt->state();
  trajopt->AddConstraintToAllKnotPoints(x(4) >= 5.0/180.0*M_PI);
  trajopt->AddConstraintToAllKnotPoints(x(6) >= 5.0/180.0*M_PI);
  trajopt->AddConstraintToAllKnotPoints(x(4) <= M_PI/2.0);
  trajopt->AddConstraintToAllKnotPoints(x(6) <= M_PI/2.0);

  // hip joint limits
  trajopt->AddConstraintToAllKnotPoints(x(3) >= -M_PI/2);
  trajopt->AddConstraintToAllKnotPoints(x(5) >= -M_PI/2);
  trajopt->AddConstraintToAllKnotPoints(x(3) <= M_PI/2.0);
  trajopt->AddConstraintToAllKnotPoints(x(5) <= M_PI/2.0);


  // x-distance constraint constraints
  trajopt->AddLinearConstraint(x0(0) == 0);
  trajopt->AddLinearConstraint(xf(0) == stride_length);

  const double R = 10;  // Cost on input effort
  auto u = trajopt->input();
  trajopt->AddRunningCost(u.transpose()*R*u);
  MatrixXd Q = MatrixXd::Zero(2*n, 2*n);
  for (int i=0; i < n; i++) {
    Q(i+n, i+n) = 10;
  }
  trajopt->AddRunningCost(x.transpose()*Q*x);

  // MatrixXd weights = MatrixXd::Zero(1,6*n+1);
  // weights(0,0) = -0.1;
  // weights(0,5) = 1; //left knee pitch

  MatrixXd weights = readCSV(directory + weights_file);

  std::vector<Binding<Constraint>> manifold_bindings;

  // auto m_constraint = std::make_shared<ManifoldConstraint>(tree, weights);
  //   for (int i = 0; i < N; i++) {
  //    manifold_bindings.push_back(trajopt->AddConstraint(m_constraint, trajopt->state(i)));
  // }

  VectorX<drake::symbolic::Expression> features(10);
  features << 1, x(2), x(3), x(4), cos(x(2)), cos(x(3)), cos(x(4)), sin(x(2)), sin(x(3)), sin(x(4));
  SymbolicManifold m_constraint(tree, features, weights);

  //add constraints to optimization problem and add bindings
  for (int i = 0; i < N; i++) {
    auto m_i = trajopt->SubstitutePlaceholderVariables(m_constraint.getConstraintExpressions(), i);
    manifold_bindings.push_back(trajopt->AddConstraint(m_i, VectorXd::Zero(m_i.size()), VectorXd::Zero(m_i.size())));
  }

  if (!init_file.empty()) {
    MatrixXd z0 = readCSV(directory + init_file);
    trajopt->SetInitialGuessForAllVariables(z0);
  }

  auto start = std::chrono::high_resolution_clock::now();
  const auto result = Solve(*trajopt, trajopt->initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  // trajopt->PrintSolution();
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;

  // systems::trajectory_optimization::checkConstraints(trajopt.get());

  MatrixXd A,H;
  VectorXd y,lb,ub,w;
  VectorXd x_sol = result.get_x_val();
  systems::trajectory_optimization::linearizeConstraints(trajopt.get(),
    x_sol, y, A, lb, ub);

  systems::trajectory_optimization::secondOrderCost(trajopt.get(), x_sol, H, w);

  VectorXd z = x_sol;

  //get feature vectors
  MatrixXd B = MatrixXd::Zero(A.rows(), weights.rows() * m_constraint.n_features());
  for (int i = 0; i < N; i++) {
    VectorXd xi = result.GetSolution(trajopt->state(i));
    // VectorXd features = m_constraint->CalcFeatures<double>(xi);
    VectorXd features(m_constraint.n_features());
    for (int j = 0; j < m_constraint.n_features(); j++) {
      auto m_ij = trajopt->SubstitutePlaceholderVariables(m_constraint.getFeature(j), i);
      features(j) = drake::ExtractDoubleOrThrow(result.GetSolution(m_ij));
    }

    VectorXd ind = systems::trajectory_optimization::getConstraintRows(
      trajopt.get(), manifold_bindings[i]);

    for (int k = 0; k < ind.size(); k++) {
      for (int j = 0; j < features.size(); j++) {
        B(ind(k), k*features.size() + j) = features(j);
      }
    }
  }

  writeCSV(directory + output_prefix + string("B.csv"), B);
  writeCSV(directory + output_prefix + string("A.csv"), A);
  writeCSV(directory + output_prefix + string("y.csv"), y);
  writeCSV(directory + output_prefix + string("lb.csv"), lb);
  writeCSV(directory + output_prefix + string("ub.csv"), ub);
  writeCSV(directory + output_prefix + string("H.csv"), H);
  writeCSV(directory + output_prefix + string("w.csv"), w);
  writeCSV(directory + output_prefix + string("z.csv"), z);

  // visualizer
  drake::lcm::DrakeLcm lcm;
  drake::systems::DiagramBuilder<double> builder;
  const drake::trajectories::PiecewisePolynomial<double> pp_xtraj =
      trajopt->ReconstructStateTrajectory(result);
  auto state_source = builder.AddSystem<drake::systems::TrajectorySource>(pp_xtraj);
  auto publisher = builder.AddSystem<drake::systems::DrakeVisualizer>(tree, &lcm);
  publisher->set_publish_period(1.0 / 60.0);
  builder.Connect(state_source->get_output_port(),
                  publisher->get_input_port(0));

  auto diagram = builder.Build();


  // while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(1);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
  // }

  return trajopt;
}

}  // namespace goldilocks_models
}  // namespace dairlib


// int main(int argc, char* argv[]) {
//   gflags::ParseCommandLineFlags(&argc, &argv, true);
//   std::srand(time(0));  // Initialize random number generator.

//   auto prog = drake::sgdIter(FLAGS_strideLength, FLAGS_duration, FLAGS_iter,
//     FLAGS_dir, FLAGS_init, FLAGS_weights, FLAGS_prefix);
// }

