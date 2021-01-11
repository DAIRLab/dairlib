#include <memory>
#include <chrono>
#include <unistd.h>
#include <gflags/gflags.h>
#include <string.h>
#include <cmath>

#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include "drake/geometry/geometry_visualization.h"
#include "drake/solvers/solve.h"

#include "common/find_resource.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "solvers/nonlinear_constraint.h"
#include "multibody/kinematic/distance_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "multibody/kinematic/kinematic_constraints.h"

#include "examples/Spirit/animate_spirit.h"

DEFINE_double(duration, 1, "The stand duration");
DEFINE_double(front2BackToeDistance, 0.35, "Nominal distance between the back and front toes.");
DEFINE_double(side2SideToeDistance, 0.2, "Nominal distance between the back and front toes.");
DEFINE_double(bodyHeight, 0.104, "The spirit body start height (defined in URDF)");
// DEFINE_double(lowerHeight,0.15, "The sitting height of the bottom of the robot");
DEFINE_double(standHeight, 0.25, "The standing height.");
DEFINE_double(apexGoal, 0.5, "Apex state goal");
DEFINE_double(knotpointsPerMode, 7, "Number of knotpoints in each contact mode" );
DEFINE_double(inputCost, 3, "The standing height.");
DEFINE_double(velocityCost, 10, "The standing height.");
DEFINE_double(eps, 1e-2, "The wiggle room.");
DEFINE_double(optTol, 1e-4,"Optimization Tolerance");
DEFINE_double(feasTol, 1e-4,"Feasibility Tolerance");
DEFINE_bool(autodiff, false, "Double or autodiff version");
DEFINE_bool(runInitTraj, false, "Animate initial conditions?");
// Parameters which enable dircon -improving features
DEFINE_bool(scale_constraint, true, "Scale the nonlinear constraint values");
DEFINE_bool(scale_variable, false, "Scale the decision variable");

using drake::AutoDiffXd;
using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using drake::trajectories::PiecewisePolynomial;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;

namespace dairlib {
namespace {

using systems::trajectory_optimization::DirconModeSequence;
using systems::trajectory_optimization::DirconMode;
using systems::trajectory_optimization::Dircon;
using systems::trajectory_optimization::KinematicConstraintType;

using std::vector;
using std::cout;
using std::endl;


/// Get a nominal Spirit Stand (i.e. zero hip ad/abduction motor torque, toes below motors) for initializing
template <typename T>
void nominalSpiritStand(MultibodyPlant<T>& plant, Eigen::VectorXd& xState, double height){
  //Get joint name dictionaries
  auto positions_map = dairlib::multibody::makeNameToPositionsMap(plant);
  auto velocities_map = dairlib::multibody::makeNameToVelocitiesMap(plant);
  
  // Initialize state vector and add orienation and goal height
  xState = Eigen::VectorXd::Zero(plant.num_positions() + plant.num_velocities());
  xState(positions_map.at("base_qw")) = 1;
  xState(positions_map.at("base_z")) = height;
  
  //Calculate the inverse kinematics to get the joint angles for toe placement
  double upperLegLength = 0.206; // length of the upper leg link
  double hipLength = 0.10098; //absOffset ToeLocation
  double hip2toeZLength = sqrt(height*height - hipLength*hipLength);// length of lower legs (2dof)
  double theta1 = asin(hip2toeZLength/(2*upperLegLength )) ; //upper angle
  double theta2 = 2*theta1 ; //lower angle
  double alpha = asin(hipLength/(height)); //hip angle

  int upperInd, kneeInd, hipInd;
  int mirroredFlag;
  for (int j = 0; j < 4; j++){
    upperInd = j * 2;
    kneeInd = j * 2 + 1;
    hipInd = j + 8;
    xState(positions_map.at("joint_" + std::to_string(upperInd))) = theta1 ;
    xState(positions_map.at("joint_" + std::to_string(kneeInd))) = theta2 ;
    mirroredFlag = -1;
    if ( hipInd > 9 ){
      mirroredFlag = 1;
    }
    xState(positions_map.at("joint_" + std::to_string(hipInd))) = mirroredFlag * alpha;
  }
}
  /// See runSpiritJump()
    // std::unique_ptr<MultibodyPlant<T>> plant_ptr,
    // MultibodyPlant<double>* plant_double_ptr,
    // std::unique_ptr<SceneGraph<double>> scene_graph_ptr,
    // double duration,
    // PiecewisePolynomial<double> init_x_traj,
    // PiecewisePolynomial<double> init_u_traj,
    // vector<PiecewisePolynomial<double>> init_l_traj,
    // vector<PiecewisePolynomial<double>> init_lc_traj,
    // vector<PiecewisePolynomial<double>> init_vc_traj
  ///
  /// Given an initial guess for state, control, and forces optimizes a standing behaviour for the spirit robot
template <typename T>
void runSpiritJump(
    std::unique_ptr<MultibodyPlant<T>> plant_ptr,
    MultibodyPlant<double>* plant_double_ptr,
    std::unique_ptr<SceneGraph<double>> scene_graph_ptr,
    double duration,
    PiecewisePolynomial<double> init_x_traj,
    PiecewisePolynomial<double> init_u_traj,
    vector<PiecewisePolynomial<double>> init_l_traj,
    vector<PiecewisePolynomial<double>> init_lc_traj,
    vector<PiecewisePolynomial<double>> init_vc_traj
    ) {

  drake::systems::DiagramBuilder<double> builder;
  MultibodyPlant<T>& plant = *plant_ptr;
  SceneGraph<double>& scene_graph =
      *builder.AddSystem(std::move(scene_graph_ptr));
  
  // Get position and velocity dictionaries 
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);

  /// For Spirit front left leg->0, back left leg->1, front right leg->2, back right leg->3
  /// Get the frame of each toe and attach a world point to the toe tip (frame is at toe ball center).

  int num_legs = 4;
  double toeRadius = 0.02; // Radius of toe ball
  Vector3d toeOffset(toeRadius,0,0); // vector to "contact point"
  double mu = 3; //friction //prev 1
  
  auto toe_evaluators = multibody::KinematicEvaluatorSet<T>(plant); //Initialize kinematic evaluator set
  
  // Get the toe frames
  const auto& toe0_frontLeft  = plant.GetFrameByName( "toe" + std::to_string(0) );
  const auto& toe1_backLeft   = plant.GetFrameByName( "toe" + std::to_string(1) );
  const auto& toe2_frontRight = plant.GetFrameByName( "toe" + std::to_string(2) );
  const auto& toe3_backRight  = plant.GetFrameByName( "toe" + std::to_string(3) );
  // Create offset worldpoints
  auto toe0_eval = multibody::WorldPointEvaluator<T>(plant, toeOffset, toe0_frontLeft , Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2}); //Shift to tip;
  auto toe1_eval = multibody::WorldPointEvaluator<T>(plant, toeOffset, toe1_backLeft  , Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2}); //Shift to tip;
  auto toe2_eval = multibody::WorldPointEvaluator<T>(plant, toeOffset, toe2_frontRight, Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2}); //Shift to tip;
  auto toe3_eval = multibody::WorldPointEvaluator<T>(plant, toeOffset, toe3_backRight , Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2}); //Shift to tip;
  // Set frictional properties (not sure what this does to the optimization)
  toe0_eval.set_frictional(); toe0_eval.set_mu(mu);
  toe1_eval.set_frictional(); toe1_eval.set_mu(mu);
  toe2_eval.set_frictional(); toe2_eval.set_mu(mu);
  toe3_eval.set_frictional(); toe3_eval.set_mu(mu);
  // Consolidate the evaluators for contant constraint
  toe_evaluators.add_evaluator(&(toe0_eval));
  toe_evaluators.add_evaluator(&(toe1_eval));
  toe_evaluators.add_evaluator(&(toe2_eval));
  toe_evaluators.add_evaluator(&(toe3_eval));
  
  /// Setup the full stance mode.
  
  double min_T = .03;
  double max_T = 3;
  auto full_support = DirconMode<T>(toe_evaluators,FLAGS_knotpointsPerMode, min_T, max_T); //No min and max mode times

  for (int i = 0; i < num_legs; i++ ){
    full_support.MakeConstraintRelative(i, 0);  // x-coordinate can be non-zero
    full_support.MakeConstraintRelative(i, 1);  // y-coordinate can be non-zero
  }

  full_support.SetDynamicsScale(
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17},  150.0);
  full_support.SetKinVelocityScale(
    {0, 1, 2, 3}, {0, 1, 2}, 1);
    full_support.SetKinPositionScale(
        {0, 1, 2, 3}, {0, 1, 2}, 150);

  /// Add flight mode 
  auto evaluators_flight = multibody::KinematicEvaluatorSet<T>(plant);
  auto flight_mode = DirconMode<T>(evaluators_flight, FLAGS_knotpointsPerMode,
                                   min_T, max_T);

                                   
  flight_mode.SetDynamicsScale(
    {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17},  1.0);
  
  ///Mode Sequence
  // Adding the ONE mode to the sequence, will not leave full_support
  auto sequence = DirconModeSequence<T>(plant);
  sequence.AddMode(&full_support);
  sequence.AddMode(&flight_mode);
  sequence.AddMode(&flight_mode);
  sequence.AddMode(&full_support);
  
  vector<int> mode_lengths(sequence.num_modes(),FLAGS_knotpointsPerMode);
  
  ///Setup trajectory optimization
  auto trajopt = Dircon<T>(sequence);
  // Set up Trajectory Optimization options
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Print file", "../snopt.out");
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", 200000);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Iterations limit", 1000000);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major optimality tolerance",
                           FLAGS_optTol);  // target optimality
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major feasibility tolerance", FLAGS_feasTol);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                           0);  // 0

    // Add duration constraint, currently constrained not bounded
  trajopt.AddDurationBounds(0, duration*2);
  // Initialize the trajectory control state and forces
  
  for (int j = 0; j < sequence.num_modes(); j++) {
    trajopt.drake::systems::trajectory_optimization::MultipleShooting::
        SetInitialTrajectory(init_u_traj, init_x_traj);
    trajopt.SetInitialForceTrajectory(j, init_l_traj[j], init_lc_traj[j],
                                      init_vc_traj[j]);
  }

  

  /// Setup all the optimization constraints 
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_u = plant.num_actuators();

  int numTotalKnotpoints = trajopt.N();
  
  auto u   = trajopt.input();
  auto x   = trajopt.state();

  auto   x0  = trajopt.initial_state();
  auto   xlo = trajopt.state_vars(1, 0);
  auto xapex = trajopt.state_vars(2, 0);
  auto   xtd = trajopt.state_vars(3, 0);
  auto   xf  = trajopt.final_state();
  
  
  // Initial body positions
  trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, x0(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position
  trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, x0(positions_map.at("base_y")));
  trajopt.AddBoundingBoxConstraint(FLAGS_standHeight - FLAGS_eps, FLAGS_standHeight + FLAGS_eps, x0(positions_map.at("base_z")));
  // Lift off body positions conditions
  //trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xlo(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position (helps with positive knee constraint)
  trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xlo(positions_map.at("base_y")));
  // Apex body positions conditions
  //trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xapex(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position (helps with positive knee constraint)
  trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xapex(positions_map.at("base_y")));
  // Touchdown body positions conditions
  trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xtd(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position (helps with positive knee constraint)
  trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xtd(positions_map.at("base_y")));
  // Final body positions conditions
  trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xf(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position (helps with positive knee constraint)
  trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xf(positions_map.at("base_y")));
  trajopt.AddBoundingBoxConstraint(FLAGS_standHeight - FLAGS_eps, FLAGS_standHeight + FLAGS_eps, xf(positions_map.at("base_z")));


    // // Body pose constraints (keep the body flat) at initial state
    // trajopt.AddBoundingBoxConstraint(1 - FLAGS_eps, 1 + FLAGS_eps, x0(positions_map.at("base_qw")));
    // trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, x0(positions_map.at("base_qx")));
    // trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, x0(positions_map.at("base_qy")));
    // trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, x0(positions_map.at("base_qz")));

    // // Body pose constraints (keep the body flat) at liftoff state
    // trajopt.AddBoundingBoxConstraint(1 - FLAGS_eps, 1 + FLAGS_eps, xlo(positions_map.at("base_qw")));
    // trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, xlo(positions_map.at("base_qx")));
    // trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, xlo(positions_map.at("base_qy")));
    // trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, xlo(positions_map.at("base_qz")));

    // // Body pose constraints (keep the body flat) at apex state
    // trajopt.AddBoundingBoxConstraint(1 - FLAGS_eps, 1 + FLAGS_eps, xapex(positions_map.at("base_qw")));
    // trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, xapex(positions_map.at("base_qx")));
    // trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, xapex(positions_map.at("base_qy")));
    // trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, xapex(positions_map.at("base_qz")));

    // // Body pose constraints (keep the body flat) at touchdown state
    // trajopt.AddBoundingBoxConstraint(1 - FLAGS_eps, 1 + FLAGS_eps, xtd(positions_map.at("base_qw")));
    // trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, xtd(positions_map.at("base_qx")));
    // trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, xtd(positions_map.at("base_qy")));
    // trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, xtd(positions_map.at("base_qz")));

    // // Body pose constraints (keep the body flat) at final state
    // trajopt.AddBoundingBoxConstraint(1 - FLAGS_eps, 1 + FLAGS_eps, xf(positions_map.at("base_qw")));
    // trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, xf(positions_map.at("base_qx")));
    // trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, xf(positions_map.at("base_qy")));
    // trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, xf(positions_map.at("base_qz")));




/// Start/End velocity constraints of the behavior
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), x0.tail(n_v));
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), xf.tail(n_v));
                                   
  trajopt.AddBoundingBoxConstraint( FLAGS_apexGoal-FLAGS_eps, FLAGS_apexGoal+FLAGS_eps, xapex(positions_map.at("base_z")) );
  //trajopt.AddBoundingBoxConstraint( 0, 0, xapex(n_q + velocities_map.at("base_vz")) );
                                   
  double upperSet = 1;
  double kneeSet = 2;

  //STATIC LEGS IN FLIGHT
    trajopt.AddBoundingBoxConstraint( upperSet - FLAGS_eps, upperSet + FLAGS_eps, xapex( positions_map.at("joint_0") ) );
    trajopt.AddBoundingBoxConstraint(  kneeSet - FLAGS_eps,  kneeSet + FLAGS_eps, xapex( positions_map.at("joint_1") ) );

    trajopt.AddBoundingBoxConstraint( upperSet - FLAGS_eps, upperSet + FLAGS_eps, xapex( positions_map.at("joint_2") ) );
    trajopt.AddBoundingBoxConstraint(  kneeSet - FLAGS_eps,  kneeSet + FLAGS_eps, xapex( positions_map.at("joint_3") ) );

    trajopt.AddBoundingBoxConstraint( upperSet - FLAGS_eps, upperSet + FLAGS_eps, xapex( positions_map.at("joint_4") ) );
    trajopt.AddBoundingBoxConstraint(  kneeSet - FLAGS_eps,  kneeSet + FLAGS_eps, xapex( positions_map.at("joint_5") ) );

    trajopt.AddBoundingBoxConstraint( upperSet - FLAGS_eps, upperSet + FLAGS_eps, xapex( positions_map.at("joint_6") ) );
    trajopt.AddBoundingBoxConstraint(  kneeSet - FLAGS_eps,  kneeSet + FLAGS_eps, xapex( positions_map.at("joint_7") ) );

    // trajopt.AddBoundingBoxConstraint(      -0 - FLAGS_eps,        0 + FLAGS_eps, xapex( positions_map.at("joint_8")));
    // trajopt.AddBoundingBoxConstraint(      -0 - FLAGS_eps,        0 + FLAGS_eps, xapex( positions_map.at("joint_9")));
    // trajopt.AddBoundingBoxConstraint(      -0 - FLAGS_eps,        0 + FLAGS_eps, xapex( positions_map.at("joint_10")));
    // trajopt.AddBoundingBoxConstraint(      -0 - FLAGS_eps,        0 + FLAGS_eps, xapex( positions_map.at("joint_11")));

    
    // trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xapex( n_q + velocities_map.at("joint_0dot")));
    // trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xapex( n_q + velocities_map.at("joint_1dot")));
    // trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xapex( n_q + velocities_map.at("joint_2dot")));
    // trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xapex( n_q + velocities_map.at("joint_3dot")));
    // trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xapex( n_q + velocities_map.at("joint_4dot")));
    // trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xapex( n_q + velocities_map.at("joint_5dot")));
    // trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xapex( n_q + velocities_map.at("joint_6dot")));
    // trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xapex( n_q + velocities_map.at("joint_7dot")));
    // trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xapex( n_q + velocities_map.at("joint_8dot")));
    // trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xapex( n_q + velocities_map.at("joint_9dot")));
    // trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xapex( n_q + velocities_map.at("joint_10dot")));
    // trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xapex( n_q + velocities_map.at("joint_11dot")));

    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_0dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_1dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_2dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_3dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_4dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_5dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_6dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_7dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_8dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_9dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_10dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_11dot")));

  for (int i = 0; i < trajopt.N(); i++){
    auto xi = trajopt.state(i);
    // Joint limits
    trajopt.AddBoundingBoxConstraint(FLAGS_eps, M_PI - FLAGS_eps, xi( positions_map.at("joint_1")));
    trajopt.AddBoundingBoxConstraint(FLAGS_eps, M_PI - FLAGS_eps, xi( positions_map.at("joint_3")));
    trajopt.AddBoundingBoxConstraint(FLAGS_eps, M_PI - FLAGS_eps, xi( positions_map.at("joint_5")));
    trajopt.AddBoundingBoxConstraint(FLAGS_eps, M_PI - FLAGS_eps, xi( positions_map.at("joint_7")));

    trajopt.AddBoundingBoxConstraint(FLAGS_eps, M_PI - FLAGS_eps, xi( positions_map.at("joint_0")));
    trajopt.AddBoundingBoxConstraint(FLAGS_eps, M_PI - FLAGS_eps, xi( positions_map.at("joint_2")));
    trajopt.AddBoundingBoxConstraint(FLAGS_eps, M_PI - FLAGS_eps, xi( positions_map.at("joint_4")));
    trajopt.AddBoundingBoxConstraint(FLAGS_eps, M_PI - FLAGS_eps, xi( positions_map.at("joint_6")));

    trajopt.AddBoundingBoxConstraint(-0.5, 0.1, xi( positions_map.at("joint_8")));
    trajopt.AddBoundingBoxConstraint(-0.5, 0.1, xi( positions_map.at("joint_9")));
    trajopt.AddBoundingBoxConstraint(-0.1, 0.5, xi( positions_map.at("joint_10")));
    trajopt.AddBoundingBoxConstraint(-0.1, 0.5, xi( positions_map.at("joint_11")));

    //Orientation
    trajopt.AddBoundingBoxConstraint(1 - FLAGS_eps, 1 + FLAGS_eps, xi(positions_map.at("base_qw")));
    trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, xi(positions_map.at("base_qx")));
    trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, xi(positions_map.at("base_qy")));
    trajopt.AddBoundingBoxConstraint(0 - FLAGS_eps, 0 + FLAGS_eps, xi(positions_map.at("base_qz")));

    // Height
    trajopt.AddBoundingBoxConstraint( 0.15, 2, xi( positions_map.at("base_z")));
  }

 double upperLegLength = 0.206; // length of the upper leg link

  ///Setup the traditional cost function
  const double R = FLAGS_inputCost;  // Cost on input effort
  const MatrixXd Q = FLAGS_velocityCost  * MatrixXd::Identity(n_v, n_v); // Cost on velocity

  trajopt.AddRunningCost( x.tail(n_v).transpose() * Q * x.tail(n_v) );
  trajopt.AddRunningCost( u.transpose()*R*u );
  for(int joint = 0; joint< 12; joint ++ )
  {
    auto joint_v = x(n_q+velocities_map.at("joint_"+std::to_string(joint)+"dot"));
    trajopt.AddRunningCost(joint_v * 0 * joint_v);
  }
  
  /// Setup the visualization during the optimization
  int num_ghosts = 3;// Number of ghosts in visualization. NOTE: there are limitations on number of ghosts based on modes and knotpoints
  std::vector<unsigned int> visualizer_poses; // Ghosts for visualizing during optimization
  for(int i = 0; i < sequence.num_modes(); i++){
      visualizer_poses.push_back(num_ghosts); 
  }
  trajopt.CreateVisualizationCallback(
      dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf"),
      visualizer_poses, 0.2); // setup which URDF, how many poses, and alpha transparency 

  
  /// Run the optimization using your initial guess
  auto start = std::chrono::high_resolution_clock::now();
  const auto result = Solve(trajopt, trajopt.initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;
  std::cout << (result.is_success() ? "Optimization Success" : "Optimization Fail") << std::endl;

  /// Run animation of the final trajectory
  const drake::trajectories::PiecewisePolynomial<double> pp_xtraj =
      trajopt.ReconstructStateTrajectory(result);
  multibody::connectTrajectoryVisualizer(plant_double_ptr,
      &builder, &scene_graph, pp_xtraj);
  auto diagram = builder.Build();
  while (true) {
    
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(0.25);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
    sleep(2);
  }
}
}  // namespace
}  // namespace dairlib


int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::srand(time(0));  // Initialize random number generator.
 
  auto plant = std::make_unique<MultibodyPlant<double>>(0.0);
  auto plant_vis = std::make_unique<MultibodyPlant<double>>(0.0);
  auto scene_graph = std::make_unique<SceneGraph<double>>();
  Parser parser(plant.get());
  Parser parser_vis(plant_vis.get(), scene_graph.get());
  std::string full_name =
      dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf");

  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);
  
  plant->mutable_gravity_field().set_gravity_vector(-9.81 *
      Eigen::Vector3d::UnitZ());

  plant->Finalize();
  plant_vis->Finalize();
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant->num_positions() +
                       plant->num_velocities());

  int nu = plant->num_actuators();
  int nx = plant->num_positions() + plant->num_velocities();
  int nq = plant->num_positions();
  int nv = plant->num_velocities();
  int N = 20; // number of timesteps

  std::vector<MatrixXd> init_x;
  std::vector<MatrixXd> init_u;
  std::vector<PiecewisePolynomial<double>> init_l_traj;
  std::vector<PiecewisePolynomial<double>> init_lc_traj;
  std::vector<PiecewisePolynomial<double>> init_vc_traj;

  // Initialize state trajectory
  std::vector<double> init_time;

  VectorXd xInit(nx);
  VectorXd xMid(nx);
  VectorXd xState(nx);
  xInit = Eigen::VectorXd::Zero(plant->num_positions() + plant->num_velocities());
  xMid = Eigen::VectorXd::Zero(plant->num_positions() + plant->num_velocities());
  xState = Eigen::VectorXd::Zero(plant->num_positions() + plant->num_velocities());

  auto positions_map = dairlib::multibody::makeNameToPositionsMap(*plant);
  auto velocities_map = dairlib::multibody::makeNameToVelocitiesMap(*plant);
  int num_joints = 12;

  // Print joint dictionary
  std::cout<<"**********************Joints***********************"<<std::endl;
  for (auto const& element : positions_map)
    std::cout << element.first << " = " << element.second << std::endl;
  for (auto const& element : velocities_map)
    std::cout << element.first << " = " << element.second << std::endl;
  std::cout<<"***************************************************"<<std::endl;
    
  dairlib::nominalSpiritStand( *plant, xInit,  0.16); //Update xInit
  dairlib::nominalSpiritStand( *plant, xMid,  0.35); //Update xMid
  
  xMid(positions_map.at("base_z"))=FLAGS_apexGoal;

  VectorXd deltaX(nx);
  VectorXd averageV(nx);
  deltaX = xMid-xInit;
  averageV = deltaX / FLAGS_duration;
  xInit.tail(nv-3) = (averageV.head(nq)).tail(nq-4); //Ignoring Orientation make velocity the average
  double time = 0;
  double dt = FLAGS_duration/(N-1)/2;

  // Initial pose
  xState = xInit;

  for (int i = 0; i < N; i++) {
    time=i*dt; // calculate iteration's time
    init_time.push_back(time);

    // Switch the direction of the stand to go back to the initial state (not actually properly periodic initial)
    if ( i > (N-1)/2 ){
         xState.tail(nv) = -xInit.tail(nv);
    }
    // Integrate the positions based on constant velocity  for joints and xyz
    for (int j = 0; j < num_joints; j++){
          xState(positions_map.at("joint_" + std::to_string(j))) =  
                        xState(positions_map.at("joint_" + std::to_string(j))) + xState(nq + velocities_map.at("joint_" + std::to_string(j)+"dot" )) * dt;
    }
    xState(positions_map.at("base_x")) = 
                        xState(positions_map.at("base_x")) + xState(nq + velocities_map.at("base_vx")) * dt;
    
    xState(positions_map.at("base_y")) = 
                        xState(positions_map.at("base_y")) + xState(nq + velocities_map.at("base_vy")) * dt;
    
    xState(positions_map.at("base_z")) = 
                        xState(positions_map.at("base_z")) + xState(nq + velocities_map.at("base_vz")) * dt;
    // Save timestep state into matrix
    init_x.push_back(xState);
    init_u.push_back(Eigen::VectorXd::Zero(nu));
  }
  // Make matrix into trajectory
  auto init_x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);
  auto init_u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_u);

  
  // Four contacts so forces are 12 dimensional
  Eigen::VectorXd init_l_vec(12);
  // Initial guess
  init_l_vec << 0, 0, 3*9.81, 0, 0, 3*9.81, 0, 0, 3*9.81, 0, 0, 3*9.81; //gravity and mass distributed
  
  //Initialize force trajectories

  //Stance
  std::vector<MatrixXd> init_l_j;
  std::vector<MatrixXd> init_lc_j;
  std::vector<MatrixXd> init_vc_j;
  std::vector<double> init_time_j;
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * FLAGS_duration / (N - 1));
    init_l_j.push_back(init_l_vec);
    init_lc_j.push_back(init_l_vec);
    init_vc_j.push_back(VectorXd::Zero(12));
  }

  auto init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
  auto init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
  auto init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

  init_l_traj.push_back(init_l_traj_j);
  init_lc_traj.push_back(init_lc_traj_j);
  init_vc_traj.push_back(init_vc_traj_j);

  // Flight
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * FLAGS_duration / (N - 1));
    init_l_j.push_back(VectorXd::Zero(12));
    init_lc_j.push_back(VectorXd::Zero(12));
    init_vc_j.push_back(VectorXd::Zero(12));
  }

  init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
  init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
  init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

  init_l_traj.push_back(init_l_traj_j);
  init_lc_traj.push_back(init_lc_traj_j);
  init_vc_traj.push_back(init_vc_traj_j);
  init_l_traj.push_back(init_l_traj_j);
  init_lc_traj.push_back(init_lc_traj_j);
  init_vc_traj.push_back(init_vc_traj_j);

  // Stance
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * FLAGS_duration / (N - 1));
    init_l_j.push_back(init_l_vec);
    init_lc_j.push_back(init_l_vec);
    init_vc_j.push_back(VectorXd::Zero(12));
  }

  init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
  init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
  init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

  init_l_traj.push_back(init_l_traj_j);
  init_lc_traj.push_back(init_lc_traj_j);
  init_vc_traj.push_back(init_vc_traj_j);

  if (FLAGS_autodiff) {
    std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_autodiff =
        drake::systems::System<double>::ToAutoDiffXd(*plant);
    dairlib::runSpiritJump<drake::AutoDiffXd>(
      std::move(plant_autodiff), plant_vis.get(), std::move(scene_graph),
      FLAGS_duration, init_x_traj, init_u_traj, init_l_traj,
      init_lc_traj, init_vc_traj);
  } else if (FLAGS_runInitTraj){
    dairlib::runAnimate<double>(
      std::move(plant), plant_vis.get(), std::move(scene_graph), init_x_traj);
  }else {
    dairlib::runSpiritJump<double>(
      std::move(plant), plant_vis.get(), std::move(scene_graph),
      FLAGS_duration, init_x_traj, init_u_traj, init_l_traj,
      init_lc_traj, init_vc_traj);
  }

}

