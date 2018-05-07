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

#include "systems/trajectory_optimization/dircon_util.h"

#include "systems/trajectory_optimization/dircon_position_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"

#include "src/manifold_constraint.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using drake::systems::trajectory_optimization::HybridDircon;
using drake::systems::trajectory_optimization::DirconDynamicConstraint;
using drake::systems::trajectory_optimization::DirconKinematicConstraint;
using drake::systems::trajectory_optimization::DirconOptions;
using drake::systems::trajectory_optimization::DirconKinConstraintType;
using drake::trajectories::PiecewisePolynomial;
using std::vector;
using std::shared_ptr;
using std::cout;
using std::endl;
using drake::goldilocks_walking::ManifoldConstraint;

DEFINE_double(strideLength, 0.1, "The stride length.");
DEFINE_double(duration, 1, "The stride duration");
DEFINE_double(height, 0.8, "The height");
DEFINE_int64(iter, 100, "Number of iterations");

/// Inputs: initial trajectory
/// Outputs: trajectory optimization problem
namespace drake{
namespace dircon {
shared_ptr<HybridDircon<double>> runDircon(double stride_length, double duration, double height, int iter,
    PiecewisePolynomial<double> init_x_traj,
    PiecewisePolynomial<double> init_u_traj,
    vector<PiecewisePolynomial<double>> init_l_traj,
    vector<PiecewisePolynomial<double>> init_lc_traj,
    vector<PiecewisePolynomial<double>> init_vc_traj) {
  RigidBodyTree<double> tree;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld("PlanarWalkerWithTorso.urdf", multibody::joints::kFixed, &tree);
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
  int nu = tree.get_num_actuators();

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

  int N = 1 - timesteps.size();
  for (int i = 0; i < timesteps.size(); i++) {
    N += timesteps[i];
  }

  std::vector<DirconKinematicDataSet<double>*> dataset_list;
  dataset_list.push_back(&leftDataSet);
  dataset_list.push_back(&rightDataSet);

  std::vector<DirconOptions> options_list;
  options_list.push_back(leftOptions);
  options_list.push_back(rightOptions);

  auto trajopt = std::make_shared<HybridDircon<double>>(tree, timesteps, min_dt, max_dt, dataset_list, options_list);

  trajopt->AddDurationBounds(duration, duration);

  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file","snopt.out");
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Major iterations limit",iter);

  // trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level","1");

  for (int j = 0; j < timesteps.size(); j++) {
    trajopt->systems::trajectory_optimization::MultipleShooting::SetInitialTrajectory(init_u_traj,init_x_traj);
    trajopt->SetInitialForceTrajectory(j, init_l_traj[j], init_lc_traj[j], init_vc_traj[j]);
  }

  //Periodicity constraints
  // planar_x - 0
  // planar_z - 1
  // planar_roty - 2
  // left_hip_pin - 3
  // left_knee_pin - 4
  // right_hip_pin - 5
  // right_knee_pin - 6
  auto x0 = trajopt->initial_state();
  auto xf = trajopt->final_state();

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


  // Knee joint limits
  auto x = trajopt->state();

  trajopt->AddConstraintToAllKnotPoints(x(4) >= 0);
  trajopt->AddConstraintToAllKnotPoints(x(6) >= 0);

  // x-distance constraint constraints
  trajopt->AddLinearConstraint(x0(0) == 0);
  trajopt->AddLinearConstraint(xf(0) == stride_length);

  const double R = 10;  // Cost on input effort
  auto u = trajopt->input();
  trajopt->AddRunningCost(u.transpose()*R*u);
  const double Q = 1;
  trajopt->AddRunningCost(x.transpose()*Q*x);

  //Add manifold constraint: constaint height at z = 7
  // MatrixXd weights = MatrixXd::Zero(2,2*n);
  // weights(0,1) = 1;
  // weights(1,7) = 1;
  // VectorXd c(2);
  // c << 0.5, 0;
  MatrixXd weights = MatrixXd::Zero(1,6*n);
  weights(0,1) = 1; //z
  weights(0,2+2*n) = 0.3; //0.3*cos(pitch)

  // trajopt->AddConstraintToAllKnotPoints(x(1) == 0.9);

  VectorXd c(1);
  if (height > 0) {
    c << height;
    auto m_constraint = std::make_shared<ManifoldConstraint>(tree, weights, c);

    for (int i = 0; i < N; i++) {
       trajopt->AddConstraint(m_constraint, trajopt->state(i));
    }
  }

  auto l = trajopt->impulse_vars(0);
  // trajopt->AddLinearConstraint(l(1) == 0);

  auto start = std::chrono::high_resolution_clock::now();
  auto result = trajopt->Solve();
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  trajopt->PrintSolution();
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << result << std::endl;
  std::cout << "Cost:" << trajopt->GetOptimalCost() <<std::endl;

  // systems::trajectory_optimization::dircon::checkConstraints(trajopt.get());

  MatrixXd A;
  VectorXd y,lb,ub;
  VectorXd x_sol = trajopt->GetSolution(trajopt->decision_variables());
  systems::trajectory_optimization::dircon::linearizeConstraints(trajopt.get(),
    x_sol, y, A, lb, ub);

//  MatrixXd y_and_bounds(y.size(),3);
//  y_and_bounds.col(0) = lb;
//  y_and_bounds.col(1) = y;
//  y_and_bounds.col(2) = ub;
//  cout << "*************y***************" << endl;
//  cout << y_and_bounds << endl;
//  cout << "*************A***************" << endl;
//  cout << A << endl;

  //visualizer
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;
  const trajectories::PiecewisePolynomial<double> pp_xtraj = trajopt->ReconstructStateTrajectory();
  auto state_source = builder.AddSystem<systems::TrajectorySource>(pp_xtraj);
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);
  publisher->set_publish_period(1.0 / 60.0);
  builder.Connect(state_source->get_output_port(),
                  publisher->get_input_port(0));

  auto diagram = builder.Build();


  while (true) {
    systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(.2);
    simulator.Initialize();
    simulator.StepTo(pp_xtraj.end_time());
  }

  return trajopt;
}
}
}


int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::srand(time(0));  // Initialize random number generator.

  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld("PlanarWalkerWithTorso.urdf", drake::multibody::joints::kFixed, &tree);

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(tree.get_num_positions() + tree.get_num_velocities());
  x0(1) = 1;
  x0(3) = -.4;
  x0(4) = .8;
  x0(5) = -.4;
  x0(6) = .8;
  Eigen::VectorXd init_l_vec(2);
  init_l_vec << 0, tree.getMass()*9.81;
  int nu = 4;
  int nx = 14;
  int N = 10;

  
  std::vector<MatrixXd> init_x;
  std::vector<MatrixXd> init_u;
  std::vector<PiecewisePolynomial<double>> init_l_traj;
  std::vector<PiecewisePolynomial<double>> init_lc_traj;
  std::vector<PiecewisePolynomial<double>> init_vc_traj;

  //Initialize state trajectory
  std::vector<double> init_time;
  for (int i = 0; i < 2*N-1; i++) {
    init_time.push_back(i*.2);
    init_x.push_back(x0 + .1*VectorXd::Random(nx));
    init_u.push_back(VectorXd::Random(nu));
  }
  auto init_x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_x);
  auto init_u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_u);

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

  auto prog = drake::dircon::runDircon(FLAGS_strideLength, FLAGS_duration, FLAGS_height, FLAGS_iter,
    init_x_traj, init_u_traj,  init_l_traj, init_lc_traj, init_vc_traj);
}

