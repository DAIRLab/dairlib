#include <memory>
#include <chrono>

#include <gflags/gflags.h>

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

#include "common/find_resource.h"
#include "attic/systems/trajectory_optimization/dircon_util.h"
#include "attic/systems/trajectory_optimization/dircon_position_data.h"
#include "attic/systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "attic/systems/trajectory_optimization/hybrid_dircon.h"
#include "attic/systems/trajectory_optimization/dircon_opt_constraints.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using drake::trajectories::PiecewisePolynomial;
using std::vector;
using std::shared_ptr;
using std::cout;
using std::endl;

DEFINE_double(strideLength, 0.1, "The stride length.");
DEFINE_double(duration, 1, "The stride duration");

/// Inputs: initial trajectory
/// Outputs: trajectory optimization problem
namespace dairlib {
namespace {

using systems::trajectory_optimization::HybridDircon;
using systems::trajectory_optimization::DirconDynamicConstraint;
using systems::trajectory_optimization::DirconKinematicConstraint;
using systems::trajectory_optimization::DirconOptions;
using systems::trajectory_optimization::DirconKinConstraintType;

shared_ptr<HybridDircon<double>> runDircon(double stride_length, double duration,
    PiecewisePolynomial<double> init_x_traj,
    PiecewisePolynomial<double> init_u_traj,
    vector<PiecewisePolynomial<double>> init_l_traj,
    vector<PiecewisePolynomial<double>> init_lc_traj,
    vector<PiecewisePolynomial<double>> init_vc_traj) {
  RigidBodyTree<double> tree;

  std::string full_name =
      FindResourceOrThrow("examples/PlanarWalker/PlanarWalker.urdf");
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(full_name,
      drake::multibody::joints::kFixed, &tree);
  //const std::unique_ptr<const RigidBodyTree<double>> tree =  std::unique_ptr<const RigidBodyTree<double>>(&model);

  for (int i = 0; i < tree.get_num_bodies(); i++)
    cout << tree.getBodyOrFrameName(i) << endl;
  for (int i = 0; i < tree.get_num_actuators(); i++)
    cout << tree.actuators[i].name_ << endl;
  for (int i = 0; i < tree.get_num_positions(); i++)
    cout << tree.get_position_name(i) << endl;
  for (int i = 0; i < tree.get_num_velocities(); i++)
    cout << tree.get_velocity_name(i) << endl;

// world
// base
// base_x
// base_xz
// hip
// left_upper_leg
// left_upper_leg_mass
// left_lower_leg
// left_lower_leg_mass
// right_upper_leg
// right_upper_leg_mass
// right_lower_leg
// right_lower_leg_mass

// hip_torque
// left_knee_torque
// right_knee_torque




  // int n = tree.get_num_positions();
  // int nu = tree.get_num_actuators();

  int leftLegIdx = tree.FindBodyIndex("left_lower_leg");
  int rightLegIdx = tree.FindBodyIndex("right_lower_leg");

  Vector3d pt;
  pt << 0,0,-.5;
  bool isXZ = true;

  auto leftFootConstraint = DirconPositionData<double>(tree,leftLegIdx,pt,isXZ);
  auto rightFootConstraint = DirconPositionData<double>(tree,rightLegIdx,pt,isXZ);

  Vector3d normal;
  normal << 0,0,1;
  double mu = 1;
  leftFootConstraint.addFixedNormalFrictionConstraints(normal,mu);
  rightFootConstraint.addFixedNormalFrictionConstraints(normal,mu);

  std::vector<DirconKinematicData<double>*> leftConstraints;
  leftConstraints.push_back(&leftFootConstraint);
  auto leftDataSet = DirconKinematicDataSet<double>(tree, &leftConstraints);

  std::vector<DirconKinematicData<double>*> rightConstraints;
  rightConstraints.push_back(&rightFootConstraint);
  auto rightDataSet = DirconKinematicDataSet<double>(tree, &rightConstraints);

  auto leftOptions = DirconOptions(leftDataSet.countConstraints());
  leftOptions.setConstraintRelative(0,true);

  auto rightOptions = DirconOptions(rightDataSet.countConstraints());
  rightOptions.setConstraintRelative(0,true);

  std::vector<int> timesteps;
  timesteps.push_back(10);
  timesteps.push_back(10);
  std::vector<double> min_dt;
  min_dt.push_back(.01);
  min_dt.push_back(.01);
  std::vector<double> max_dt;
  max_dt.push_back(.3);
  max_dt.push_back(.3);

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
                           "Major iterations limit", 200);

  // trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
  //    "Verify level","1");

  for (uint j = 0; j < timesteps.size(); j++) {
    trajopt->drake::systems::trajectory_optimization::MultipleShooting::
        SetInitialTrajectory(init_u_traj, init_x_traj);
    trajopt->SetInitialForceTrajectory(j, init_l_traj[j], init_lc_traj[j],
                                      init_vc_traj[j]);
  }

  // Periodicity constraints
  // planar_x-0
  // planar_z-1
  // planar_roty-2
  // left_knee_pin-3
  // hip_pin-4
  // right_knee_pin-5
  // planar_xdot-6
  // planar_zdot-7
  // planar_rotydot-8
  // left_knee_pindot-9
  // hip_pindot-10
  // right_knee_pindot-11
  auto x0 = trajopt->initial_state();
  auto xf = trajopt->final_state();

  trajopt->AddLinearConstraint(x0(1) == xf(1));
  trajopt->AddLinearConstraint(x0(2) + x0(4) == xf(2));
  trajopt->AddLinearConstraint(x0(3) == xf(5));
  trajopt->AddLinearConstraint(x0(4) == -xf(4));
  trajopt->AddLinearConstraint(x0(5) == xf(3));

  trajopt->AddLinearConstraint(x0(6) == xf(6));
  trajopt->AddLinearConstraint(x0(7) == xf(7));
  trajopt->AddLinearConstraint(x0(8) + x0(10) == xf(8));
  trajopt->AddLinearConstraint(x0(9) == xf(11));
  trajopt->AddLinearConstraint(x0(10) == -xf(10));
  trajopt->AddLinearConstraint(x0(11) == xf(9));

  // Knee joint limits
  auto x = trajopt->state();
  trajopt->AddConstraintToAllKnotPoints(x(3) >= 0);
  trajopt->AddConstraintToAllKnotPoints(x(5) >= 0);

  // Hip constraints
  trajopt->AddLinearConstraint(x0(0) == 0);
  trajopt->AddLinearConstraint(xf(0) == stride_length);

  const double R = 10;  // Cost on input effort
  auto u = trajopt->input();
  trajopt->AddRunningCost(u.transpose()*R*u);
  const double Q = 1;
  trajopt->AddRunningCost(x.transpose()*Q*x);


  auto start = std::chrono::high_resolution_clock::now();
  const auto result = Solve(*trajopt, trajopt->initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;

  systems::trajectory_optimization::checkConstraints(trajopt.get(), result);

  MatrixXd A;
  VectorXd y, lb, ub;
  VectorXd x_sol = result.get_x_val();
  systems::trajectory_optimization::linearizeConstraints(trajopt.get(),
    x_sol, y, A, lb, ub);

//  MatrixXd y_and_bounds(y.size(),3);
//  y_and_bounds.col(0) = lb;
//  y_and_bounds.col(1) = y;
//  y_and_bounds.col(2) = ub;
//  cout << "*************y***************" << endl;
//  cout << y_and_bounds << endl;
//  cout << "*************A***************" << endl;
//  cout << A << endl;

  // visualizer
  drake::lcm::DrakeLcm lcm;
  drake::systems::DiagramBuilder<double> builder;
  const drake::trajectories::PiecewisePolynomial<double> pp_xtraj =
      trajopt->ReconstructStateTrajectory();
  auto state_source = builder.AddSystem<drake::systems::TrajectorySource>(
        pp_xtraj);
  auto publisher = builder.AddSystem<drake::systems::DrakeVisualizer>(tree,
                                                                      &lcm);
  publisher->set_publish_period(1.0 / 60.0);
  builder.Connect(state_source->get_output_port(),
                  publisher->get_input_port(0));

  auto diagram = builder.Build();


  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(.2);
    simulator.Initialize();
    simulator.StepTo(pp_xtraj.end_time());
  }

  return trajopt;
}
}  // namespace
}  // namespace dairlib


int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::srand(time(0));  // Initialize random number generator.

  RigidBodyTree<double> tree;
  std::string full_name =
      dairlib::FindResourceOrThrow("examples/PlanarWalker/PlanarWalker.urdf");
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(full_name,
      drake::multibody::joints::kFixed, &tree);

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(tree.get_num_positions() +
                       tree.get_num_velocities());

  Eigen::VectorXd init_l_vec(2);
  init_l_vec << 0, tree.getMass()*9.81;
  int nu = 3;
  int N = 10;

  std::vector<MatrixXd> init_x;
  std::vector<MatrixXd> init_u;
  std::vector<PiecewisePolynomial<double>> init_l_traj;
  std::vector<PiecewisePolynomial<double>> init_lc_traj;
  std::vector<PiecewisePolynomial<double>> init_vc_traj;

  // Initialize state trajectory
  std::vector<double> init_time;
  for (int i = 0; i < 2*N-1; i++) {
    init_time.push_back(i*.2);
    init_x.push_back(x0);
    init_u.push_back(VectorXd::Random(nu));
  }
  auto init_x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);
  auto init_u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_u);

  //Initialize force trajectories
  for (int j = 0; j < 2; j++) {    
    std::vector<MatrixXd> init_l_j;
    std::vector<MatrixXd> init_lc_j;
    std::vector<MatrixXd> init_vc_j;
    std::vector<double> init_time_j;
    for (int i = 0; i < N; i++) {
      init_time_j.push_back(i*.2);
      init_l_j.push_back(init_l_vec);
      init_lc_j.push_back(init_l_vec);
      init_vc_j.push_back(VectorXd::Zero(2));
    }

    auto init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
    auto init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
    auto init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

    init_l_traj.push_back(init_l_traj_j);
    init_lc_traj.push_back(init_lc_traj_j);
    init_vc_traj.push_back(init_vc_traj_j);
  }

  auto prog = dairlib::runDircon(FLAGS_strideLength, FLAGS_duration,
    init_x_traj, init_u_traj,  init_l_traj, init_lc_traj, init_vc_traj);
}

