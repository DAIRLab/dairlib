#include <memory>
#include <chrono>

#include <gflags/gflags.h>
#include "drake/systems/trajectory_optimization/direct_collocation.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/math/autodiff.h"
#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "dircon_position_data.h"
#include "dircon_kinematic_data_set.h"
#include "dircon.h"
#include "hybrid_dircon.h"
#include "dircon_opt_constraints.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/constraint.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using std::cout;
using std::endl;
using drake::systems::trajectory_optimization::Dircon;
using drake::systems::trajectory_optimization::HybridDircon;
using drake::systems::trajectory_optimization::DirectCollocation;
using drake::systems::trajectory_optimization::DirconDynamicConstraint;
using drake::systems::trajectory_optimization::DirconKinematicConstraint;
using drake::systems::trajectory_optimization::DirconOptions;
using drake::systems::trajectory_optimization::DirconKinConstraintType;
using drake::trajectories::PiecewisePolynomial;

DEFINE_int64(testIndex, 0, "The index of the test to run");

//template VectorXd RigidBodyTree<double>::transformPointsJacobianDotTimesV<double, Matrix3Xd>(KinematicsCache<double> const&, Eigen::MatrixBase<Matrix3Xd> const&, int, int);

namespace dairlib {
namespace {

void checkConstraints(const solvers::MathematicalProgram* prog) {
  for (auto const& binding : prog->generic_constraints()) {
    double tol = 1e-6;
    auto y = prog->EvalBindingAtSolution(binding);
    auto c = binding.evaluator();
    bool isSatisfied = (y.array() >= c->lower_bound().array() - tol).all() &&
                       (y.array() <= c->upper_bound().array() + tol).all();
    if (!isSatisfied) {
      cout << "Constraint violation: " << c->get_description() << endl;
      MatrixXd tmp(y.size(),3);
      tmp << c->lower_bound(), y, c->upper_bound();
      cout << tmp << endl;
    }
  }
}

int testConstraints() {

  //auto model = std::make_unique<RigidBodyTree<double>>();
  RigidBodyTree<double> tree;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld("../../examples/Acrobot/Acrobot_floating.urdf", multibody::joints::kFixed, &tree);
  //const std::unique_ptr<const RigidBodyTree<double>> tree =  std::unique_ptr<const RigidBodyTree<double>>(&model);

  int bodyIdx = 5;
  Vector3d pt;
  pt << 0,0,0;
  Vector3d pt2;
  pt2 << 0,0,0;
  bool isXZ = true;
  DirconPositionData<AutoDiffXd> constraintd = DirconPositionData<AutoDiffXd>(tree,bodyIdx,pt,isXZ);
  DirconPositionData<AutoDiffXd> constraintd2 = DirconPositionData<AutoDiffXd>(tree,bodyIdx,pt2,isXZ);
  DirconPositionData<double> constraint = DirconPositionData<double>(tree,bodyIdx,pt,isXZ);
  DirconPositionData<double> constraint2 = DirconPositionData<double>(tree,bodyIdx,pt2,isXZ);
  cout << "??" <<endl;
  int n = 4;
  int nl = 4;
  int nu = 1;

  VectorXd q(n,1);
  VectorXd v(n,1);
  q << 0,0,M_PI,0;
  v << 0,0,0,0;
  VectorXd x(2*n,1);
  x << q, v;
  VectorXd u(nu,1);
  u << 0;
  VectorXd l(nl,1);
  l << 0, 2*9.81, 0, 0;

  VectorXd q1(n,1);
  VectorXd v1(n,1);
  q1 << 0,0,M_PI,0;
  v1 << 0,0,0,0;
  VectorXd x1(2*n,1);
  x1 << q1, v1;
  VectorXd u1(nu,1);
  u1 << 0;
  VectorXd l1(nl,1);
  l1 << 0,2*9.81,0,0;
  VectorXd lc(nl,1);
  lc << 0,2*9.81,0,0;
  VectorXd vc(nl,1);
  vc << 0, 0, 0, 0;

  VectorXd h(1,1);
  h << .1;

  VectorXd vars(1+4*n + 2*nu + 4*nl);
  vars << h,x,x1,u,u1,l,l1,lc,vc;

  AutoDiffVecXd vars_autodiff = math::initializeAutoDiff(vars);
  AutoDiffVecXd x_autodiff = vars_autodiff.segment(1,2*n);  
  AutoDiffVecXd q_autodiff = x_autodiff.head(n);
  AutoDiffVecXd v_autodiff = x_autodiff.tail(n);

  AutoDiffVecXd u_autodiff = vars_autodiff.segment(1+4*n, nu);
  AutoDiffVecXd l_autodiff = vars_autodiff.segment(1+4*n+2*nu, nl);

  std::vector<DirconKinematicData<AutoDiffXd>*> constraintsd;
  constraintsd.push_back(&constraintd);
  constraintsd.push_back(&constraintd2);
  auto datasetd = DirconKinematicDataSet<AutoDiffXd>(tree, &constraintsd);

  //auto dataset_ptr =  std::unique_ptr<DirconKinematicDataSet<AutoDiffXd>>(&datasetd);

   datasetd.updateData(x_autodiff, u_autodiff, l_autodiff);

  auto dynamicConstraint = std::make_shared<DirconDynamicConstraint<AutoDiffXd>>(tree, datasetd);
  auto kinematicConstraint = std::make_shared<DirconKinematicConstraint<AutoDiffXd>>(tree, datasetd);


  //AutoDiffVecXd xul 
  //VectorXd y(2*n);
  AutoDiffVecXd x_dynamic = vars_autodiff;
  AutoDiffVecXd y_dynamic;// = y.template cast<AutoDiffXd>();
  //std::cout << dynamicY << endl;
  //std::cout << x_dynamic.rows() << " " << dynamicConstraint.num_vars() << endl;
  dynamicConstraint->Eval(x_dynamic,y_dynamic);

  cout << "*********** x_dynamic  ***********" << endl;
  std::cout << x_dynamic << endl;
  cout << "*********** y_dynamic  ***********" << endl;
  std::cout << y_dynamic << endl;
  cout << "*********** dy_dynamic  ***********" << endl;
  std::cout << math::autoDiffToGradientMatrix(y_dynamic) << endl;

  AutoDiffVecXd x_kinematic = VectorX<AutoDiffXd>(2*n+nu+nl);
  x_kinematic << x_autodiff, u_autodiff, l_autodiff;
  AutoDiffVecXd y_kinematic;

  kinematicConstraint->Eval(x_kinematic,y_kinematic);

  cout << "*********** c  ***********" << endl;
  std::cout << datasetd.getC() << endl;


  cout << "*********** xdot  ***********" << endl;
  std::cout << datasetd.getXDot() << endl;

  cout << "*********** x  ***********" << endl;
  std::cout << x_autodiff << endl;
  cout << "*********** u  ***********" << endl;
  std::cout << u_autodiff << endl;
  cout << "*********** l  ***********" << endl;
  std::cout << l_autodiff << endl;
  cout << "*********** y_kinematic  ***********" << endl;
  std::cout << y_kinematic << endl;
  cout << "*********** dy_kinematic  ***********" << endl;
  std::cout << math::autoDiffToGradientMatrix(y_kinematic) << endl;

  return 0;
}

template <typename T>
int testDircon(bool addForceConstraints, Eigen::VectorXd x0 = Eigen::VectorXd::Zero(8)) {
  RigidBodyTree<double> tree;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld("../../examples/Acrobot/Acrobot_floating.urdf", multibody::joints::kFixed, &tree);
  //const std::unique_ptr<const RigidBodyTree<double>> tree =  std::unique_ptr<const RigidBodyTree<double>>(&model);

  cout << tree.getBodyOrFrameName(0) << endl;
  cout << tree.getBodyOrFrameName(1) << endl;
  cout << tree.getBodyOrFrameName(2) << endl;
  cout << tree.getBodyOrFrameName(3) << endl;
  cout << tree.getBodyOrFrameName(4) << endl;
  cout << tree.getBodyOrFrameName(5) << endl;
  int n = 4;
  int nu = 1;
  int nl = 2;
  int bodyIdx = 4;
  Vector3d pt;
  pt << 0,0,0;
  bool isXZ = true;
  auto constraint = DirconPositionData<T>(tree,bodyIdx,pt,isXZ);

  if (addForceConstraints) {
    Vector3d normal;
    normal << 0,0,1;
    double mu = 1;
    constraint.addFixedNormalFrictionConstraints(normal,mu);
  }

  std::vector<DirconKinematicData<T>*> constraints;
  constraints.push_back(&constraint);
  auto dataset = DirconKinematicDataSet<T>(tree, &constraints);

  int N = 11;
  auto options = DirconOptions(dataset.countConstraints());
  options.setStartType(DirconKinConstraintType::kAccelOnly);
  options.setEndType(DirconKinConstraintType::kAccelOnly);
  //options.setConstraintRelative(0,true);
  auto trajopt = std::make_shared<Dircon<T>>(tree, N, .02, .3, dataset, options);

  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file","snopt.out");
  // trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level","1");

  //Construct what should be a feasible trajectory
  std::vector<double> init_time;
  std::vector<MatrixXd> init_x;
  std::vector<MatrixXd> init_u;
  std::vector<MatrixXd> init_l;
  std::vector<MatrixXd> init_lc;
  std::vector<MatrixXd> init_vc;

  VectorXd init_l_vec(nl);
  init_l_vec << 0, 20*9.81;

  for (int i = 0; i < N; i++) {
    init_time.push_back(i*.1);
    // init_x.push_back(VectorXd::Zero(2*n));
    init_x.push_back(x0);
    init_u.push_back(VectorXd::Random(nu));
    //init_u[i](0) = .1;
    init_l.push_back(init_l_vec);
    init_lc.push_back(init_l_vec);
    init_vc.push_back(VectorXd::Zero(nl));
  }

  auto init_x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_x);
  auto init_u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_u);
  auto init_l_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_l);
  auto init_lc_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_lc);
  auto init_vc_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_vc);
  trajopt->SetInitialTrajectory(init_u_traj,init_x_traj,init_l_traj,init_lc_traj,init_vc_traj);
  // trajopt->systems::trajectory_optimization::MultipleShooting::SetInitialTrajectory(init_u_traj,init_x_traj);

  Eigen::VectorXd xG(2*n);
  xG << 0, 0, M_PI, 0, 0, 0, 0, 0;
  trajopt->AddLinearConstraint(trajopt->initial_state() == x0);
  trajopt->AddLinearConstraint(trajopt->final_state() == xG);

  const double kTorqueLimit = 8;
  auto u = trajopt->input();
  trajopt->AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
  trajopt->AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);

  const double R = 10;  // Cost on input "effort".
  trajopt->AddRunningCost((R * u) * u);

  trajopt->AddEqualTimeIntervalsConstraints();

  auto start = std::chrono::high_resolution_clock::now();
  auto result = trajopt->Solve();
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  trajopt->PrintSolution();
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << result << std::endl;
  std::cout << "Cost:" << trajopt->GetOptimalCost() <<std::endl;

  //visualizer
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;
  const trajectories::PiecewisePolynomial<double> pp_xtraj = trajopt->ReconstructStateTrajectory(result);
  auto state_source = builder.AddSystem<systems::TrajectorySource>(pp_xtraj);
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);
  publisher->set_publish_period(1.0 / 60.0);
  builder.Connect(state_source->get_output_port(),
                  publisher->get_input_port(0));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.set_target_realtime_rate(1);
  simulator.Initialize();
  simulator.AdvanceTo(pp_xtraj.end_time());
  return 0;
}


template <typename T>
int testHybridDircon(bool addForceConstraints, Eigen::VectorXd x0 = Eigen::VectorXd::Zero(8)) {
  RigidBodyTree<double> tree;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld("../../examples/Acrobot/Acrobot_floating.urdf", multibody::joints::kFixed, &tree);
  //const std::unique_ptr<const RigidBodyTree<double>> tree =  std::unique_ptr<const RigidBodyTree<double>>(&model);

  cout << tree.getBodyOrFrameName(0) << endl;
  cout << tree.getBodyOrFrameName(1) << endl;
  cout << tree.getBodyOrFrameName(2) << endl;
  cout << tree.getBodyOrFrameName(3) << endl;
  cout << tree.getBodyOrFrameName(4) << endl;
  cout << tree.getBodyOrFrameName(5) << endl;
  int n = 4;
  int nu = 1;
  int nl = 2;
  int bodyIdx = 4;
  Vector3d pt;
  pt << 0,0,0;
  bool isXZ = true;
  auto constraint = DirconPositionData<T>(tree,bodyIdx,pt,isXZ);

  if (addForceConstraints) {
    Vector3d normal;
    normal << 0,0,1;
    double mu = 1;
    constraint.addFixedNormalFrictionConstraints(normal,mu);
  }

  std::vector<DirconKinematicData<T>*> constraints;
  constraints.push_back(&constraint);
  auto dataset = DirconKinematicDataSet<T>(tree, &constraints);

  int N = 11;
  auto options = DirconOptions(dataset.countConstraints());
  options.setStartType(DirconKinConstraintType::kAccelOnly);
  options.setEndType(DirconKinConstraintType::kAccelOnly);
  //options.setConstraintRelative(0,true);
  std::vector<int> timesteps;
  timesteps.push_back(N);
  std::vector<double> min_dt;
  min_dt.push_back(.02);
  std::vector<double> max_dt;
  max_dt.push_back(.3);
  std::vector<DirconKinematicDataSet<T>*> dataset_list;
  dataset_list.push_back(&dataset);

  std::vector<DirconOptions> options_list;
  options_list.push_back(options);
  
  auto trajopt = std::make_shared<HybridDircon<T>>(tree, timesteps, min_dt, max_dt, dataset_list, options_list);

  // HybridDircon(const RigidBodyTree<double>& tree, vector<int> num_time_samples, vector<double> minimum_timestep,
    // vector<double> maximum_timestep, vector<DirconKinematicDataSet<T>*> constraints, vector<DirconOptions> options);

  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file","snopt.out");
  // trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level","1");

  //Construct what should be a feasible trajectory
  std::vector<double> init_time;
  std::vector<MatrixXd> init_x;
  std::vector<MatrixXd> init_u;
  std::vector<MatrixXd> init_l;
  std::vector<MatrixXd> init_lc;
  std::vector<MatrixXd> init_vc;

  VectorXd init_l_vec(nl);
  init_l_vec << 0, tree.getMass()*9.81;

  for (int i = 0; i < N; i++) {
    init_time.push_back(i*.1);
    // init_x.push_back(VectorXd::Zero(2*n));
    init_x.push_back(x0);
    init_u.push_back(VectorXd::Random(nu));
    //init_u[i](0) = .1;
    init_l.push_back(init_l_vec);
    init_lc.push_back(init_l_vec);
    init_vc.push_back(VectorXd::Zero(nl));
  }

  auto init_x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_x);
  auto init_u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_u);
  auto init_l_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_l);
  auto init_lc_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_lc);
  auto init_vc_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_vc);
  // trajopt->SetInitialTrajectory(init_u_traj,init_x_traj,init_l_traj,init_lc_traj,init_vc_traj);
  trajopt->systems::trajectory_optimization::MultipleShooting::SetInitialTrajectory(init_u_traj,init_x_traj);
  trajopt->SetInitialForceTrajectory(0, init_l_traj, init_lc_traj, init_vc_traj);

  Eigen::VectorXd xG(2*n);
  xG << 0, 0, M_PI, 0, 0, 0, 0, 0;
  trajopt->AddLinearConstraint(trajopt->initial_state() == x0);
  trajopt->AddLinearConstraint(trajopt->final_state() == xG);

  const double kTorqueLimit = 8;
  auto u = trajopt->input();
  trajopt->AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
  trajopt->AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);

  const double R = 10;  // Cost on input "effort".
  trajopt->AddRunningCost((R * u) * u);

  trajopt->AddEqualTimeIntervalsConstraints();

  auto start = std::chrono::high_resolution_clock::now();
  auto result = trajopt->Solve();
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  trajopt->PrintSolution();
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << result << std::endl;
  std::cout << "Cost:" << trajopt->GetOptimalCost() <<std::endl;

  //visualizer
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;
  const trajectories::PiecewisePolynomial<double> pp_xtraj = trajopt->ReconstructStateTrajectory(result);
  auto state_source = builder.AddSystem<systems::TrajectorySource>(pp_xtraj);
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);
  publisher->set_publish_period(1.0 / 60.0);
  builder.Connect(state_source->get_output_port(),
                  publisher->get_input_port(0));

  auto diagram = builder.Build();

  systems::Simulator<double> simulator(*diagram);

  simulator.set_target_realtime_rate(1);
  simulator.Initialize();
  simulator.AdvanceTo(pp_xtraj.end_time());
  return 0;
}

template <typename T>
int testHybridDirconJump(bool addForceConstraints, Eigen::VectorXd x0 = Eigen::VectorXd::Zero(8)) {
  RigidBodyTree<double> tree;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld("../../examples/Acrobot/Acrobot_floating.urdf", multibody::joints::kFixed, &tree);
  //const std::unique_ptr<const RigidBodyTree<double>> tree =  std::unique_ptr<const RigidBodyTree<double>>(&model);

  cout << tree.getBodyOrFrameName(0) << endl;
  cout << tree.getBodyOrFrameName(1) << endl;
  cout << tree.getBodyOrFrameName(2) << endl;
  cout << tree.getBodyOrFrameName(3) << endl;
  cout << tree.getBodyOrFrameName(4) << endl;
  cout << tree.getBodyOrFrameName(5) << endl;
  int n = 4;
  int nu = 1;
  int nl = 2;
  int bodyIdx = 4;
  Vector3d pt;
  pt << 0,0,0;
  bool isXZ = true;
  auto constraint = DirconPositionData<T>(tree,bodyIdx,pt,isXZ);

  if (addForceConstraints) {
    Vector3d normal;
    normal << 0,0,1;
    double mu = 1;
    constraint.addFixedNormalFrictionConstraints(normal,mu);
  }


  std::vector<DirconKinematicData<T>*> constraints;
  constraints.push_back(&constraint);
  auto dataset = DirconKinematicDataSet<T>(tree, &constraints);

  std::vector<DirconKinematicData<T>*> no_constraints;
  auto dataset_free = DirconKinematicDataSet<T>(tree, &no_constraints);

  auto options = DirconOptions(dataset.countConstraints());
  options.setStartType(DirconKinConstraintType::kAccelOnly);
  auto options_end = DirconOptions(dataset.countConstraints());
  options_end.setEndType(DirconKinConstraintType::kAll);
  options_end.setConstraintRelative(0,true);

  std::vector<int> timesteps;
  timesteps.push_back(5);
  timesteps.push_back(5);
  timesteps.push_back(10);
  std::vector<double> min_dt;
  min_dt.push_back(.1);
  min_dt.push_back(.05);
  min_dt.push_back(.1);
  std::vector<double> max_dt;
  max_dt.push_back(.3);
  max_dt.push_back(.3);
  max_dt.push_back(.5);

  std::vector<DirconKinematicDataSet<T>*> dataset_list;
  dataset_list.push_back(&dataset);
  dataset_list.push_back(&dataset_free);
  dataset_list.push_back(&dataset);

  std::vector<DirconOptions> options_list;
  options_list.push_back(options);
  options_list.push_back(DirconOptions(0));
  options_list.push_back(options_end);
  
  auto trajopt = std::make_shared<HybridDircon<T>>(tree, timesteps, min_dt, max_dt, dataset_list, options_list);


  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file","snopt.out");
  // trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level","1");





  for (int j = 0; j < timesteps.size(); j++) {
    if (timesteps[j] <= 1)
      continue;
    VectorXd init_l_vec(nl);
    init_l_vec << 0, tree.getMass()*9.81;
    std::vector<double> init_time;
    std::vector<MatrixXd> init_x;
    std::vector<MatrixXd> init_u;
    std::vector<MatrixXd> init_l;
    std::vector<MatrixXd> init_lc;
    std::vector<MatrixXd> init_vc;
    for (int i = 0; i < timesteps[j]; i++) {
      init_time.push_back(i*.1);
      // init_x.push_back(VectorXd::Zero(2*n));
      init_x.push_back(x0);
      init_u.push_back(VectorXd::Random(nu));
      // init_u[i](0) = .1;
      init_l.push_back(init_l_vec);
      init_lc.push_back(init_l_vec);
      init_vc.push_back(VectorXd::Zero(nl));
    }

    auto init_x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_x);
    auto init_u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_u);
    auto init_l_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_l);
    auto init_lc_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_lc);
    auto init_vc_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_vc);
    // trajopt->SetInitialTrajectory(init_u_traj,init_x_traj,init_l_traj,init_lc_traj,init_vc_traj);
    trajopt->systems::trajectory_optimization::MultipleShooting::SetInitialTrajectory(init_u_traj,init_x_traj);
    if (j!= 1)
      trajopt->SetInitialForceTrajectory(j, init_l_traj, init_lc_traj, init_vc_traj);
  }

  Eigen::VectorXd xG(2*n);
  xG << 0, 0, M_PI, 0, 0, 0, 0, 0;
  trajopt->AddLinearConstraint(trajopt->initial_state() == x0);
  trajopt->AddLinearConstraint(trajopt->state(timesteps[0] + 1)(1) >= .1);
  trajopt->AddLinearConstraint(trajopt->final_state().tail(2*n-2) == xG.tail(2*n-2));

  const double kTorqueLimit = 8;
  auto u = trajopt->input();
  trajopt->AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
  trajopt->AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);

  const double R = 10;  // Cost on input "effort".
  trajopt->AddRunningCost((R * u) * u);


  auto start = std::chrono::high_resolution_clock::now();
  auto result = trajopt->Solve();
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  trajopt->PrintSolution();
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << result << std::endl;
  std::cout << "Cost:" << trajopt->GetOptimalCost() <<std::endl;

  checkConstraints(trajopt.get());

  //visualizer
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> builder;
  const trajectories::PiecewisePolynomial<double> pp_xtraj = trajopt->ReconstructStateTrajectory(result);
  auto state_source = builder.AddSystem<systems::TrajectorySource>(pp_xtraj);
  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);
  publisher->set_publish_period(1.0 / 60.0);
  builder.Connect(state_source->get_output_port(),
                  publisher->get_input_port(0));

  auto diagram = builder.Build();


  while (true) {
    systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(1);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
  }
  return 0;
}

int testDirconConstraints() {
RigidBodyTree<double> tree;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld("../../examples/Acrobot/Acrobot_floating.urdf", multibody::joints::kFixed, &tree);
  //const std::unique_ptr<const RigidBodyTree<double>> tree =  std::unique_ptr<const RigidBodyTree<double>>(&model);

  cout << tree.getBodyOrFrameName(0) << endl;
  cout << tree.getBodyOrFrameName(1) << endl;
  cout << tree.getBodyOrFrameName(2) << endl;
  cout << tree.getBodyOrFrameName(3) << endl;
  cout << tree.getBodyOrFrameName(4) << endl;
  cout << tree.getBodyOrFrameName(5) << endl;
  int n = 4;
  int nu = 1;
  int nl = 2;
  int bodyIdx = 4;
  Vector3d pt;
  pt << 0,0,0;
  bool isXZ = true;

  DirconPositionData<AutoDiffXd> constraint = DirconPositionData<AutoDiffXd>(tree,bodyIdx,pt,isXZ);
  std::vector<DirconKinematicData<AutoDiffXd>*> constraints;
  constraints.push_back(&constraint);
  auto dataset = DirconKinematicDataSet<AutoDiffXd>(tree, &constraints);
  auto dyn_constraint = std::make_shared<DirconDynamicConstraint<AutoDiffXd>>(tree, dataset);

  DirconPositionData<double> constraint_double = DirconPositionData<double>(tree,bodyIdx,pt,isXZ);
  std::vector<DirconKinematicData<double>*> constraints_double;
  constraints_double.push_back(&constraint_double);
  auto dataset_double = DirconKinematicDataSet<double>(tree, &constraints_double);
  auto dyn_constraint_double = std::make_shared<DirconDynamicConstraint<double>>(tree, dataset_double);

  VectorXd x = VectorXd::Ones(2*(2*n+nu)+4*nl+1);
  x(0) = .01;

  AutoDiffVecXd y;

  auto start = std::chrono::high_resolution_clock::now();
  for (int i=0; i < 100; i++) {
    x(3) = i/100.0;
    AutoDiffVecXd x_autodiff = math::initializeAutoDiff(x);
    dyn_constraint->DoEval(x_autodiff,y);
  }
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "T(autodiff):" << elapsed.count() <<std::endl;
  std::cout << y  << endl;
  std::cout << math::autoDiffToGradientMatrix(y) << endl;
  MatrixXd dy = math::autoDiffToGradientMatrix(y);

  start = std::chrono::high_resolution_clock::now();
  for (int i=0; i < 100; i++) {
    x(3) = i/100.0;
    AutoDiffVecXd x_autodiff = math::initializeAutoDiff(x);
    //y = dyn_constraint->tmp(x);
    //y = math::jacobian([dyn_constraint](const auto& x_arg) { return dyn_constraint->DoEvalJacobian(x_arg.template cast<AutoDiffXd>().eval()); }, x_autodiff);
    dyn_constraint_double->DoEval(x_autodiff,y);
  }
  finish = std::chrono::high_resolution_clock::now();
  elapsed = finish - start;
  std::cout << "T(double):" << elapsed.count() <<std::endl;
  std::cout << y  << endl;
  std::cout << math::autoDiffToGradientMatrix(y) << endl;
  MatrixXd dy_double = math::autoDiffToGradientMatrix(y);

  std::cout << "Gradient Check:" << endl;
  MatrixXd gradient_error = dy - dy_double;
  std::cout << (gradient_error*1e6).array().round()*1e-6 << endl;
  return 0;

}

int testDircol() {
  RigidBodyTree<double> tree_dircon;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld("../../examples/Acrobot/Acrobot.urdf", multibody::joints::kFixed, &tree_dircon);

  auto tree = std::make_unique<RigidBodyTree<double>>();
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld("../../examples/Acrobot/Acrobot.urdf", multibody::joints::kFixed, tree.get());
  drake::systems::DiagramBuilder<double> builder;
  auto plant = builder.AddSystem<systems::RigidBodyPlant<double>>(std::move(tree));
  const auto context = plant->CreateDefaultContext();

  int N = 10;

  // std::vector<DirconKinematicData<AutoDiffXd>*> constraints;
  // auto dataset = DirconKinematicDataSet<AutoDiffXd>(tree_dircon, &constraints);
  // auto options = DirconOptions(dataset.getNumConstraints());
  // auto trajopt = std::make_shared<Dircon>(tree_dircon, N, .01, 3.0, dataset, options);

  auto trajopt = std::make_shared<DirectCollocation>(plant, *context, N, .1,.3);

  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Print file","snopt.out");

  Eigen::VectorXd x0(4);
  x0 << 0, 0, 0, 0;
  Eigen::VectorXd xG(4);
  xG << M_PI, 0, 0, 0;
  cout << trajopt->initial_state() << endl;
  trajopt->AddLinearConstraint(trajopt->initial_state() == x0);
  trajopt->AddLinearConstraint(trajopt->final_state() == xG);

  const double kTorqueLimit = 8;
  auto u = trajopt->input();
  trajopt->AddConstraintToAllKnotPoints(-kTorqueLimit <= u(0));
  trajopt->AddConstraintToAllKnotPoints(u(0) <= kTorqueLimit);

  const double R = 10;  // Cost on input "effort".
  trajopt->AddRunningCost((R * u) * u);

  trajopt->AddEqualTimeIntervalsConstraints();

  const double timespan_init = 4;
  auto traj_init_x = PiecewisePolynomial<double>::FirstOrderHold(
      {0, timespan_init}, {x0,xG});
  //trajopt->SetInitialTrajectory(PiecewisePolynomial<double>(), traj_init_x,PiecewisePolynomial<double>(),PiecewisePolynomial<double>(),PiecewisePolynomial<double>());
  trajopt->SetInitialTrajectory(PiecewisePolynomial<double>(), traj_init_x);

  auto result = trajopt->Solve();
  trajopt->PrintSolution();

  //visualizer
  lcm::DrakeLcm lcm;
  systems::DiagramBuilder<double> vis_builder;
  const trajectories::PiecewisePolynomial<double> pp_xtraj = trajopt->ReconstructStateTrajectory(result);
  auto state_source = vis_builder.AddSystem<systems::TrajectorySource>(pp_xtraj);
  auto publisher = vis_builder.AddSystem<systems::DrakeVisualizer>(plant->get_rigid_body_tree(), &lcm);
  publisher->set_publish_period(1.0 / 60.0);
  vis_builder.Connect(state_source->get_output_port(),
                  publisher->get_input_port(0));
  auto diagram = vis_builder.Build();
  systems::Simulator<double> simulator(*diagram);

  simulator.set_target_realtime_rate(1);
  simulator.Initialize();
  simulator.AdvanceTo(pp_xtraj.end_time());
  return 0;

}



}  // namespace
}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::srand(time(0));  // Initialize random number generator.

  Eigen::VectorXd init_up = Eigen::VectorXd(8);
  init_up << 0, 0, 3, .2, 0, 0, 0, 0;
  switch(FLAGS_testIndex) {
    case 0:
      std::cout << "Testing DIRCON with AutoDiffXd" << std::endl;
      drake::dircon::examples::testDircon<drake::AutoDiffXd>(false);
      break;
    case 1:
    std::cout << "Testing DIRCON with double" << std::endl;
      drake::dircon::examples::testDircon<double>(false);
      break;
    case 2:
      std::cout << "Testing DIRCON with force constraints (autodiff)" << std::endl;
      drake::dircon::examples::testDircon<drake::AutoDiffXd>(true, init_up);
      break;
    case 3:
      std::cout << "Testing DIRCON with force constraints (double)" << std::endl;
      drake::dircon::examples::testDircon<double>(true, init_up);
      break;
    case 4:
    std::cout << "Testing Dircol with AutoDiffXd" << std::endl;
      drake::dircon::examples::testDircol();
      break;
    case 5:
    std::cout << "Testing DIRCON optimization constraints" << std::endl;
      drake::dircon::examples::testDirconConstraints();
      break;
    case 6:
    std::cout << "Testing kinematic constraints" << std::endl;
      drake::dircon::examples::testConstraints();
      break;
    case 7:
      std::cout << "Testing hybrid DIRCON (Acrobot, one mode) with double" << std::endl;
      drake::dircon::examples::testHybridDircon<double>(false);
      break;
    case 8:
      Eigen::VectorXd init_jump = Eigen::VectorXd(8);
      init_jump << 0, 0, M_PI*3/4, M_PI/2, 0, 0, 0, 0;
      std::cout << "Testing hybrid DIRCON (Acrobot, three modes jumping then swingup) with double" << std::endl;
      drake::dircon::examples::testHybridDirconJump<double>(true);
      break;
  }
  return 0;
}
