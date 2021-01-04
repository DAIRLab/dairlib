#include <memory>
#include <chrono>
#include <unistd.h>
#include <gflags/gflags.h>
#include <string.h>

#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"
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

DEFINE_double(duration, 1, "The stand duration");
DEFINE_double(front2BackToeDistance, 0.35, "Nominal distance between the back and front toes.");
DEFINE_double(side2SideToeDistance, 0.2, "Nominal distance between the back and front toes.");
DEFINE_double(bodyHeight, 0.104, "The spirit body start height (defined in URDF)");
DEFINE_double(lowerHeight,0.15, "The sitting height of the bottom of the robot");
DEFINE_double(upperHeight, 0.2, "The standing height.");
DEFINE_bool(autodiff, false, "Double or autodiff version");
DEFINE_bool(runInitTraj, false, "Animate initial conditions?");
// Parameters which enable dircon-improving features
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
  /// See runAnimate(
  ///    std::unique_ptr<MultibodyPlant<T>> plant_ptr,
  ///    MultibodyPlant<double>* plant_double_ptr,
  ///    std::unique_ptr<SceneGraph<double>> scene_graph_ptr,
  ///    PiecewisePolynomial<double> pp_xtraj )
  ///
  /// Takes the plants and scenegraph and a trajectory and 
  /// creates a visualization of that trajectory (example
  /// built in the main file).
template <typename T>
void runAnimate(
    std::unique_ptr<MultibodyPlant<T>> plant_ptr,
    MultibodyPlant<double>* plant_double_ptr,
    std::unique_ptr<SceneGraph<double>> scene_graph_ptr,
    PiecewisePolynomial<double> pp_xtraj
    ) {

  drake::systems::DiagramBuilder<double> builder;
  MultibodyPlant<T>& plant = *plant_ptr;
  SceneGraph<double>& scene_graph =
      *builder.AddSystem(std::move(scene_graph_ptr));

  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
   
  // Print joint dictionary
  for (auto const& element : positions_map)
    cout << element.first << " = " << element.second << endl;
  for (auto const& element : velocities_map)
    cout << element.first << " = " << element.second << endl;

  multibody::connectTrajectoryVisualizer(plant_double_ptr,
      &builder, &scene_graph, pp_xtraj);
  auto diagram = builder.Build();
  while (true) {
    
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(1);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
    sleep(1);
  }
}

  /// See runSpiritStand()
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
void runSpiritStand(
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

  // Print joint dictionary
  for (auto const& element : positions_map)
    cout << element.first << " = " << element.second << endl;
  for (auto const& element : velocities_map)
    cout << element.first << " = " << element.second << endl;
  

  /// For Spirit front left leg->0, back left leg->1, front right leg->2, back right leg->3
  /// Get the frame of each toe and attach a world point to the toe tip (frame is at toe ball center).

  int num_legs = 4;
  double toeRadius = 0.02; // Radius of toe ball
  Vector3d toeOffset(toeRadius,0,0); // vector to "contact point"
  double mu = 1; //friction
  
  
  auto evaluators = multibody::KinematicEvaluatorSet<T>(plant); //Initialize kinematic evaluator set
  // toe stand mode code written to be functionalized to remove this repeated code
  
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
  evaluators.add_evaluator(&(toe0_eval));
  evaluators.add_evaluator(&(toe1_eval));
  evaluators.add_evaluator(&(toe2_eval));
  evaluators.add_evaluator(&(toe3_eval));
  

////////////////////////TODO/DEBUG
//// Attempted to create an array to not have all of the copy and pasted code above. Reference arrays are illegal in c++. Initializing auto like this is wrong. 
//// Eventually I will create a "leg/toe" helper function that replaced all of this or figure out the ptr logic necessary to make something like this work. 
////
    // const auto& toes[num_legs]; //Initialize toeframes //Doesnt work 
    // // const dairlib::multibody::Plant<T> *toes[num_legs]; 
    // // multibody::WorldPointEvaluator<T> toe_eval[num_legs]; //Initialize worldpoints
    // auto toe_eval[num_legs];
    
    // for ( int i = 0; i < num_legs; i++) {
    //   // For each toe, set up world point evaluator
    //   toes[i] = plant.GetFrameByName( "toe"+std::to_string(i) ); //Get toe frame
    //   toe_eval[i] = multibody::WorldPointEvaluator<T>(plant, toeOffset, toes[i], Matrix3d::Identity(), Vector3d::Zero(), {0, 1, 2}); //Shift to tip
    //   toe_eval[i].set_frictional(); //
    //   toe_eval[i].set_mu(mu);
    //   evaluators.add_evaluator(&(toe_eval[i]));
    // }
/////////////////////////TODO/DEBUG

  /// Setup the standing mode. This behavior only has one mode.
  int num_knotpoints = 10; // number of knot points in the collocation
  auto full_support = DirconMode<T>(evaluators,num_knotpoints); //No min and max mode times

  for (int i = 0; i < num_legs; i++ ){
    full_support.MakeConstraintRelative(i, 0);  // x-coordinate can be non-zero
    full_support.MakeConstraintRelative(i, 1);  // y-coordinate can be non-zero
  }

  
  ///Mode Sequence
  // Adding the ONE mode to the sequence, will not leave full_support
  auto sequence = DirconModeSequence<T>(plant);
  sequence.AddMode(&full_support);

  ///Setup trajectory optimization
  auto trajopt = Dircon<T>(sequence);

  // Set up Trajectory Optimization options
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Print file", "../snopt.out");
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", 20000);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Iterations limit", 100000);
  // Add duration constraint, currently constrained not bounded
  trajopt.AddDurationBounds(0, duration);
  // Initialize the trajectory control state and forces
  for (int j = 0; j < sequence.num_modes(); j++) {
    trajopt.drake::systems::trajectory_optimization::MultipleShooting::
        SetInitialTrajectory(init_u_traj, init_x_traj);
    trajopt.SetInitialForceTrajectory(j, init_l_traj[j], init_lc_traj[j],
                                      init_vc_traj[j]);
  }

  /// Setup all the optimization constraints 
  int n_v = plant.num_velocities();
  int n_q = plant.num_positions();
  auto u = trajopt.input();
  auto x = trajopt.state();
  auto x0 = trajopt.initial_state();
  auto xmid = trajopt.state_vars(0, (num_knotpoints - 1) / 2);
  auto xf = trajopt.final_state();
  
  // Initial body positions
  trajopt.AddBoundingBoxConstraint( -0.5,  0.5, x0(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position (helps with positive knee constraint)
  trajopt.AddBoundingBoxConstraint(    0,    0, x0(positions_map.at("base_y")));
  trajopt.AddBoundingBoxConstraint(FLAGS_lowerHeight, FLAGS_lowerHeight, x0(positions_map.at("base_z")));
  
  // Mid body positions
  trajopt.AddBoundingBoxConstraint(   -0,    0, xmid(positions_map.at("base_x"))); // If both init and final are open the gradient can become flat, so constrain the final x position
  trajopt.AddBoundingBoxConstraint(    0,    0, xmid(positions_map.at("base_y")));
  trajopt.AddBoundingBoxConstraint(FLAGS_upperHeight, FLAGS_upperHeight, xmid(positions_map.at("base_z")));

  //Final Position Periodicity Constraints
  trajopt.AddLinearConstraint( xf( positions_map.at("base_x") ) == x0( positions_map.at("base_x") ) );
  trajopt.AddBoundingBoxConstraint(    0,    0, xf(positions_map.at("base_y")));
  trajopt.AddBoundingBoxConstraint(FLAGS_lowerHeight, FLAGS_lowerHeight, xf(positions_map.at("base_z")));
  
  
  /// Decide whether or not to constrain the body orientation at all knotpoints or only at the beginning and end
  bool isConstrainOrientation = false;
  if (isConstrainOrientation){
    // Body pose constraints (keep the body flat) at all the knotpoints including the ends
    for (int i = 0; i < num_knotpoints; i++) {
      auto xi = trajopt.state(i);
      trajopt.AddBoundingBoxConstraint(1, 1, xi(positions_map.at("base_qw")));
      trajopt.AddBoundingBoxConstraint(0, 0, xi(positions_map.at("base_qx")));
      trajopt.AddBoundingBoxConstraint(0, 0, xi(positions_map.at("base_qy")));
      trajopt.AddBoundingBoxConstraint(0, 0, xi(positions_map.at("base_qz")));
    }
  }else{
    // Body pose constraints (keep the body flat) at initial state
    trajopt.AddBoundingBoxConstraint(1, 1, x0(positions_map.at("base_qw")));
    trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_qx")));
    trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_qy")));
    trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_qz")));

    // Body pose constraints (keep the body flat) at mid state
    trajopt.AddBoundingBoxConstraint(1, 1, xmid(positions_map.at("base_qw")));
    trajopt.AddBoundingBoxConstraint(0, 0, xmid(positions_map.at("base_qx")));
    trajopt.AddBoundingBoxConstraint(0, 0, xmid(positions_map.at("base_qy")));
    trajopt.AddBoundingBoxConstraint(0, 0, xmid(positions_map.at("base_qz")));

    // Body pose constraints (keep the body flat) at final state
    trajopt.AddBoundingBoxConstraint(1, 1, xf(positions_map.at("base_qw")));
    trajopt.AddBoundingBoxConstraint(0, 0, xf(positions_map.at("base_qx")));
    trajopt.AddBoundingBoxConstraint(0, 0, xf(positions_map.at("base_qy")));
    trajopt.AddBoundingBoxConstraint(0, 0, xf(positions_map.at("base_qz")));
  }


  /// Start/End velocity constraints of the behavior
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v),
                                   x0.tail(n_v));
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v),
                                   xmid.tail(n_v));
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v),
                                   xf.tail(n_v));

///////////////TODO/DEBUG
  //   Might need this, but not sure. currently I personally parse this as constraining the first force_vars element
  //     in the 0th mode to zero at all the knotpoints not sure why? 
  // // Not sure what this is doing
  // for (int i = 0; i < num_knotpoints; i++) {
  //   trajopt.AddBoundingBoxConstraint(0, 0, trajopt.force_vars(0, i)(1));
  // }
//////////////TODO/DEBUG

//   /// Find and constrain distances between toes
//   // Distance evaluators for constraints (note one front back, one side to side, then symmetry)
//   auto toe_dist_eval_front_back =  multibody::DistanceEvaluator<T>(plant, toeOffset, toe0_frontLeft, toeOffset, toe1_backLeft, FLAGS_front2BackToeDistance);
//   auto toe_dist_eval_side_side  =  multibody::DistanceEvaluator<T>(plant, toeOffset, toe0_frontLeft, toeOffset, toe2_frontRight, FLAGS_side2SideToeDistance);
  
//   auto toe_distance_evaluators = multibody::KinematicEvaluatorSet<T>(plant);
//   toe_distance_evaluators.add_evaluator(&toe_dist_eval_front_back);
//   toe_distance_evaluators.add_evaluator(&toe_dist_eval_side_side);
//   // Allow a range of toe distances around the nominal
//   auto toe_distance_lb = Eigen::VectorXd(2);
//   auto toe_distance_ub = Eigen::VectorXd(2);

//   toe_distance_lb(0) = -0.1;
//   toe_distance_lb(1) = -0.1;
//   toe_distance_ub(0) =  0.1;
//   toe_distance_ub(1) =  0.1;
//   // For toe distance constraints
//   auto toe_constraints = std::make_shared<multibody::KinematicPositionConstraint<T>>(plant,
//               toe_distance_evaluators, toe_distance_lb, toe_distance_ub);
  
//   // Implement toe distance constraints (if commented dist constraints not active)
//   // for (int i = 0; i < num_knotpoints; i++){
//   //   auto xi = trajopt.state(i);
//   //   trajopt.AddConstraint(toe_constraints,xi.head(n_q));
//   // }

/// Symmetry constraints
  for (int i = 0; i < num_knotpoints; i++){
    auto xi = trajopt.state(i);
    //legs lined up (front and back hips equal)
    trajopt.AddLinearConstraint( xi( positions_map.at("joint_8") ) ==  xi( positions_map.at("joint_9") ) );
    trajopt.AddLinearConstraint( xi( positions_map.at("joint_10") ) ==  xi( positions_map.at("joint_11") ) );
  
    // Symmetry constraints (mirrored hips all else equal across)
    trajopt.AddLinearConstraint( xi( positions_map.at("joint_8") ) == -xi( positions_map.at("joint_10") ) );
    trajopt.AddLinearConstraint( xi( positions_map.at("joint_9") ) == -xi( positions_map.at("joint_11") ) );// Technically overconstrains included for testing and completeness
    
    // // Front legs equal and back legs equal constraints 
    // trajopt.AddLinearConstraint( xi( positions_map.at("joint_0") ) == xi( positions_map.at("joint_4") ) );
    // trajopt.AddLinearConstraint( xi( positions_map.at("joint_1") ) == xi( positions_map.at("joint_5") ) );
    // trajopt.AddLinearConstraint( xi( positions_map.at("joint_2") ) == xi( positions_map.at("joint_6") ) );
    // trajopt.AddLinearConstraint( xi( positions_map.at("joint_3") ) == xi( positions_map.at("joint_7") ) );
  }
  

  /// Constraints for keeping "knees" above the floor 

  // Create the knee evaluators for non-linear constraint
  double upperLegLength = 0.206; // length of the upper leg link
  try{
    if (FLAGS_lowerHeight<FLAGS_bodyHeight/2){
        throw "Body to close to floor" ;
    }
    if (FLAGS_upperHeight>=upperLegLength*2){
        throw "Body can't stand to that height without leaving the ground" ;
    }
  }catch(char const* exc){
      std::cerr<< exc <<std::endl;
  }


//   Vector3d kneeOffset(upperLegLength/2,0,0); // vector to "knee" point in upper leg frame
//   std::vector<int> z_active({2});

//   // Get frames (written out with to_string as reminder to functionalize)
//   const auto& upper0_frontLeft  = plant.GetFrameByName( "upper" + std::to_string(0) );
//   const auto& upper1_backLeft   = plant.GetFrameByName( "upper" + std::to_string(1) );
//   const auto& upper2_frontRight = plant.GetFrameByName( "upper" + std::to_string(2) );
//   const auto& upper3_backRight  = plant.GetFrameByName( "upper" + std::to_string(3) );
//   // Make world point evaluators
//   auto knee0z_eval = multibody::WorldPointEvaluator<T>(plant, kneeOffset, upper0_frontLeft , Matrix3d::Identity(), Vector3d::Zero(), z_active); //Shift to tip;
//   auto knee1z_eval = multibody::WorldPointEvaluator<T>(plant, kneeOffset, upper1_backLeft  , Matrix3d::Identity(), Vector3d::Zero(), z_active); //Shift to tip;
//   auto knee2z_eval = multibody::WorldPointEvaluator<T>(plant, kneeOffset, upper2_frontRight, Matrix3d::Identity(), Vector3d::Zero(), z_active); //Shift to tip;
//   auto knee3z_eval = multibody::WorldPointEvaluator<T>(plant, kneeOffset, upper3_backRight , Matrix3d::Identity(), Vector3d::Zero(), z_active); //Shift to tip;
//   // Consolidate evaluators
//   auto knee_z_evaluators = multibody::KinematicEvaluatorSet<T>(plant); //Initialize kinematic evaluator set
//   knee_z_evaluators.add_evaluator(&(knee0z_eval));
//   knee_z_evaluators.add_evaluator(&(knee1z_eval));
//   knee_z_evaluators.add_evaluator(&(knee2z_eval));
//   knee_z_evaluators.add_evaluator(&(knee3z_eval));
//   // Add lower bound on constraint
//   auto knee_lb = Eigen::Vector4d(0, 0, 0, 0);
//   auto knee_ub = 
//           Eigen::Vector4d(std::numeric_limits<double>::infinity(), 
//                           std::numeric_limits<double>::infinity(), 
//                           std::numeric_limits<double>::infinity(), 
//                           std::numeric_limits<double>::infinity());
//   auto positive_knee_constraints =
//       std::make_shared<multibody::KinematicPositionConstraint<T>>(
//           plant, knee_z_evaluators, knee_lb, knee_ub);

//   // Allow scaling of knee constraint (Not 100% on what this does)
//   if (FLAGS_scale_constraint) {
//     double scale = 1;
//     std::unordered_map<int, double> odbp_constraint_scale;
//     odbp_constraint_scale.insert(std::pair<int, double>(0, scale));
//     odbp_constraint_scale.insert(std::pair<int, double>(1, scale));
//     positive_knee_constraints->SetConstraintScaling(odbp_constraint_scale);
//   }
//   // Add positive knee constraint to all knot_points
//   for (int i = 0; i < num_knotpoints; i++) {
//     auto xi = trajopt.state(i);
//     trajopt.AddConstraint(positive_knee_constraints, xi.head(n_q));
//   }

  ///Setup the cost function
  const double R = 50;  // Cost on input effort
//   const double Rf = R*30;
  const MatrixXd Q = 10  * MatrixXd::Identity(n_v, n_v); // Cost on velocity
  trajopt.AddRunningCost(x.tail(n_v).transpose() * Q * x.tail(n_v));
  trajopt.AddRunningCost(u.transpose()*R*u);
//   trajopt.AddFinalCost(u.transpose()*Rf*u);

  /// Setup the visualization during the optimization
  int num_ghosts = 3;// Number of ghosts in visualization. NOTE: there are limitations on number of ghosts based on modes and knotpoints
  std::vector<unsigned int> visualizer_poses; // Ghosts for visualizing during optimization
  visualizer_poses.push_back(num_ghosts); 

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
    simulator.set_target_realtime_rate(.25);
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
  int N = 20; // number of timesteps

  std::vector<MatrixXd> init_x;
  std::vector<MatrixXd> init_u;
  std::vector<PiecewisePolynomial<double>> init_l_traj;
  std::vector<PiecewisePolynomial<double>> init_lc_traj;
  std::vector<PiecewisePolynomial<double>> init_vc_traj;

  // Initialize state trajectory
  std::vector<double> init_time;
  VectorXd xState(nx);
  xState = Eigen::VectorXd::Zero(plant->num_positions() + plant->num_velocities());
  auto positions_map = dairlib::multibody::makeNameToPositionsMap(*plant);
  auto velocities_map = dairlib::multibody::makeNameToVelocitiesMap(*plant);
  int num_joints = 12;

  // Print joint dictionary
  for (auto const& element : positions_map)
    std::cout << element.first << " = " << element.second << std::endl;
  for (auto const& element : velocities_map)
    std::cout << element.first << " = " << element.second << std::endl;
  
  int upperInd,kneeInd,hipInd;
  
  

  double time = 0;
  double dt = FLAGS_duration/(N-1);
  int switchDirections;

  // Initial pose
  xState(positions_map.at("base_qw")) = 1;  // Initialize flat pose and quat unit constraint
  xState(positions_map.at("base_z")) = FLAGS_lowerHeight; // Initialize some height TODO MAKE THE START HEIGHT  

  double initLegAngle = .3;
  for (int j = 0; j < 4; j++){
    upperInd = j * 2;
    kneeInd = j * 2 + 1;
    xState(positions_map.at( "joint_" + std::to_string(upperInd))) = initLegAngle;
    xState(positions_map.at( "joint_" + std::to_string(kneeInd))) = xState(positions_map.at( "joint_" + std::to_string(upperInd))) * 2;
  }

  for (int i = 0; i < N; i++) {
    time=i*dt;
    init_time.push_back(time);

    // Switch the direction of the stand to go back to the initial state (not actually properly periodic initial)
    switchDirections = 1;
    if ( i > (N-1)/2 ){
        switchDirections = -1;
    }
    /// VELOCITIES
    // Upper link velocities
    for (int j = 0; j < 4; j++){
        upperInd = j * 2;
        xState(nq + velocities_map.at( "joint_" + std::to_string(upperInd) + "dot" )) = 1 * switchDirections;
    }
    // Lower link velocites (for standing useful for it to be about 2 times the upper)
    for (int j = 0; j < 4; j++){
        upperInd = j * 2;
        kneeInd = j * 2 + 1;
        xState(nq + velocities_map.at( "joint_" + std::to_string(kneeInd) + "dot" )) = 2 * xState(nq + velocities_map.at( "joint_" + std::to_string(upperInd) + "dot"));
    }
    // Hip velocities at zero
    for (int j = 0; j < 4; j++){
        hipInd = j + 8;
        xState(nq + velocities_map.at( "joint_" + std::to_string(hipInd) + "dot" )) = 0;
    }
    xState( nq + velocities_map.at("base_vz") ) = (2*(FLAGS_upperHeight - FLAGS_lowerHeight)/FLAGS_duration) * switchDirections;

    /// POSITIONS
    // Get last position
    
    for (int j = 0; j < num_joints; j++){
    //     xState(nq + velocities_map.at( std::to_string(j)+"dot"  )) = 0;
        // if (j==0||j==2||j==4||j==6){
          xState(positions_map.at("joint_" + std::to_string(j))) =  
                        xState(positions_map.at("joint_" + std::to_string(j))) + xState(nq + velocities_map.at("joint_" + std::to_string(j)+"dot" )) * dt;
        // }
        // else{
        //   xState(positions_map.at("joint_" + std::to_string(j))) =   xState(positions_map.at("joint_" + std::to_string(j))) + xState(nq + velocities_map.at("joint_" + std::to_string(j)+"dot" )) * dt;
        // }
    }
    xState(positions_map.at("base_z")) = 
                        xState(positions_map.at("base_z")) + xState(nq + velocities_map.at("base_vz")) * dt;
    

    init_x.push_back(xState);
    init_u.push_back(Eigen::VectorXd::Zero(nu));
    // init_u.push_back(VectorXd::Random(nu));
  }
  auto init_x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);
  auto init_u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_u);

  
  // Four contacts so forces are 12 dimensional
  Eigen::VectorXd init_l_vec(12);
  // Initial guess
  init_l_vec << 0, 0, 3*9.81, 0, 0, 3*9.81, 0, 0, 3*9.81, 0, 0, 3*9.81; //gravity and mass distributed
  
  //Initialize force trajectories
  int num_modes = 1; // MAKE DYNAMIC DEBUG
  for (int j = 0; j < num_modes; j++) {    
    std::vector<MatrixXd> init_l_j;
    std::vector<MatrixXd> init_lc_j;
    std::vector<MatrixXd> init_vc_j;
    std::vector<double> init_time_j;
    for (int i = 0; i < N; i++) {
      init_time_j.push_back(i*FLAGS_duration/(N-1));
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
  }

  if (FLAGS_autodiff) {
    std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_autodiff =
        drake::systems::System<double>::ToAutoDiffXd(*plant);
    dairlib::runSpiritStand<drake::AutoDiffXd>(
      std::move(plant_autodiff), plant_vis.get(), std::move(scene_graph),
      FLAGS_duration, init_x_traj, init_u_traj, init_l_traj,
      init_lc_traj, init_vc_traj);
  } else if (FLAGS_runInitTraj){
    dairlib::runAnimate<double>(
      std::move(plant), plant_vis.get(), std::move(scene_graph), init_x_traj);
  }else {
    dairlib::runSpiritStand<double>(
      std::move(plant), plant_vis.get(), std::move(scene_graph),
      FLAGS_duration, init_x_traj, init_u_traj, init_l_traj,
      init_lc_traj, init_vc_traj);
  }

}

