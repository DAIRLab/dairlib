#include "examples/goldilocks_models/misc/snopt_multithread_test/run_sample_trajopt.h"

#include <memory>
#include <chrono>

#include <string>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"

#include "drake/lcm/drake_lcm.h"

#include "drake/multibody/parsing/parser.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "drake/geometry/geometry_visualization.h"

#include "common/find_resource.h"
#include "systems/primitives/subvector_pass_through.h"

#include "solvers/optimization_utils.h"
#include "systems/trajectory_optimization/dircon_position_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"

#include "multibody/visualization_utils.h"

#include "systems/goldilocks_models/symbolic_manifold.h"
#include "systems/goldilocks_models/file_utils.h"

#include "drake/solvers/choose_best_solver.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using drake::VectorX;
using drake::systems::trajectory_optimization::MultipleShooting;
using drake::trajectories::PiecewisePolynomial;
using drake::solvers::Binding;
using drake::solvers::Constraint;
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::MatrixXDecisionVariable;
using drake::symbolic::Variable;
using drake::symbolic::Expression;
using std::shared_ptr;
using std::cout;
using std::endl;
using std::string;
using std::map;

using drake::geometry::SceneGraph;
using drake::multibody::Body;
using drake::multibody::Parser;
using drake::systems::rendering::MultibodyPositionToGeometryPose;


// using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>

namespace dairlib {
namespace goldilocks_models {
namespace misc {

using systems::trajectory_optimization::HybridDircon;
using systems::trajectory_optimization::DirconDynamicConstraint;
using systems::trajectory_optimization::DirconKinematicConstraint;
using systems::trajectory_optimization::DirconOptions;
using systems::trajectory_optimization::DirconKinConstraintType;
using systems::SubvectorPassThrough;

void runSampleTrajopt(/*const MultibodyPlant<double> & plant,
                      const MultibodyPlant<AutoDiffXd> & plant_autoDiff,*/
                      double stride_length, double ground_incline,
                      string directory,
                      string init_file,
                      string prefix) {

  // Create MBP
  MultibodyPlant<double> plant;
  Parser parser(&plant);
  std::string full_name = FindResourceOrThrow(
                            "examples/goldilocks_models/PlanarWalkerWithTorso.urdf");
  parser.AddModelFromFile(full_name);
  plant.AddForceElement<drake::multibody::UniformGravityFieldElement>(
    -9.81 * Eigen::Vector3d::UnitZ());
  plant.WeldFrames(
    plant.world_frame(), plant.GetFrameByName("base"),
    drake::math::RigidTransform<double>());
  plant.Finalize();
  // Create autoDiff version of the plant
  MultibodyPlant<AutoDiffXd> plant_autoDiff(plant);


  map<string, int> positions_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> velocities_map = multibody::makeNameToVelocitiesMap(
                                      plant);
  map<string, int> actuators_map = multibody::makeNameToActuatorsMap(plant);
  // for (auto const& element : positions_map)
  //   cout << element.first << " = " << element.second << endl;
  // std::cout << "\n";
  // for (auto const& element : velocities_map)
  //   cout << element.first << " = " << element.second << endl;
  // std::cout << "\n";
  // for (auto const& element : actuators_map)
  //   cout << element.first << " = " << element.second << endl;
  // std::cout << "\n";


  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_x = n_q + n_v;
  int n_u = plant.num_actuators();
  // std::cout<<"n_x = "<<n_x<<"\n";
  // std::cout<<"n_u = "<<n_u<<"\n";

  const Body<double>& left_lower_leg = plant.GetBodyByName("left_lower_leg");
  const Body<double>& right_lower_leg =
    plant.GetBodyByName("right_lower_leg");

  Vector3d pt;
  pt << 0, 0, -.5;
  bool isXZ = true;
  Eigen::Vector2d ground_rp(0, ground_incline);  // gournd incline in roll pitch

  auto leftFootConstraint = DirconPositionData<double>(plant, left_lower_leg,
                            pt, isXZ, ground_rp);
  auto rightFootConstraint = DirconPositionData<double>(plant,
                             right_lower_leg,
                             pt, isXZ, ground_rp);

  Vector3d normal;
  normal << 0, 0, 1;
  double mu = 1;
  leftFootConstraint.addFixedNormalFrictionConstraints(normal, mu);
  rightFootConstraint.addFixedNormalFrictionConstraints(normal, mu);
  // std::cout<<leftFootConstraint.getLength()<<"\n"; //2 dim. I guess the contact point constraint in the x and z direction

  std::vector<DirconKinematicData<double>*> leftConstraints;
  leftConstraints.push_back(&leftFootConstraint);
  auto leftDataSet = DirconKinematicDataSet<double>(plant, &leftConstraints);

  std::vector<DirconKinematicData<double>*> rightConstraints;
  rightConstraints.push_back(&rightFootConstraint);
  auto rightDataSet = DirconKinematicDataSet<double>(plant,
                      &rightConstraints);

  auto leftOptions = DirconOptions(leftDataSet.countConstraints());
  leftOptions.setConstraintRelative(0, true);
  // std::cout<<"leftDataSet.countConstraints() = "<<leftDataSet.countConstraints()<<"\n";

  auto rightOptions = DirconOptions(rightDataSet.countConstraints());
  rightOptions.setConstraintRelative(0, true);

  // Stated in the MultipleShooting class:
  // This class assumes that there are a fixed number (N) time steps/samples,
  // and that the trajectory is discretized into timesteps h (N-1 of these),
  // state x (N of these), and control input u (N of these).
  std::vector<int> num_time_samples;
  num_time_samples.push_back(20); // First mode (20 sample points)
  num_time_samples.push_back(1);  // Second mode (1 sample point)
  std::vector<double> min_dt;
  min_dt.push_back(.01);
  min_dt.push_back(.01);
  std::vector<double> max_dt;
  max_dt.push_back(.3);
  max_dt.push_back(.3);

  int N = 0;
  for (uint i = 0; i < num_time_samples.size(); i++)
    N += num_time_samples[i];
  N -= num_time_samples.size() - 1; //Overlaps between modes
  // std::cout<<"N = "<<N<<"\n";

  std::vector<DirconKinematicDataSet<double>*> dataset_list;
  dataset_list.push_back(&leftDataSet);
  dataset_list.push_back(&rightDataSet);

  std::vector<DirconOptions> options_list;
  options_list.push_back(leftOptions);
  options_list.push_back(rightOptions);

  // auto trajopt = std::make_unique<HybridDircon<double>>(plant,
  //                num_time_samples, min_dt, max_dt, dataset_list, options_list);
  HybridDircon<double> trajopt(plant,
                 num_time_samples, min_dt, max_dt, dataset_list, options_list);

  // You can comment this out to not put any constraint on the time
  // However, we need it now, since we add the running cost by hand
  // trajopt.AddDurationBounds(duration, duration);

  // Periodicity constraints
  auto x0 = trajopt.initial_state();
  // auto xf = trajopt.final_state();
  auto xf = trajopt.state_vars_by_mode(num_time_samples.size() - 1,
                                        num_time_samples[num_time_samples.size() - 1] - 1);

  //Careful! if you have a string typo, the code still runs and the mapped value will be 0.
  // trajopt.AddLinearConstraint(x0(positions_map.at("planar_z")) == xf(
  //                                positions_map.at("planar_z")));
  trajopt.AddLinearConstraint(x0(positions_map.at("planar_roty")) == xf(
                                 positions_map.at("planar_roty")));
  trajopt.AddLinearConstraint(x0(positions_map.at("left_hip_pin")) == xf(
                                 positions_map.at("right_hip_pin")));
  trajopt.AddLinearConstraint(x0(positions_map.at("left_knee_pin")) == xf(
                                 positions_map.at("right_knee_pin")));
  trajopt.AddLinearConstraint(x0(positions_map.at("right_hip_pin")) == xf(
                                 positions_map.at("left_hip_pin")));
  trajopt.AddLinearConstraint(x0(positions_map.at("right_knee_pin")) == xf(
                                 positions_map.at("left_knee_pin")));

  trajopt.AddLinearConstraint(x0(n_q + velocities_map.at("planar_xdot"))
                               == xf(n_q + velocities_map.at("planar_xdot")));
  trajopt.AddLinearConstraint(x0(n_q + velocities_map.at("planar_zdot"))
                               == xf(n_q + velocities_map.at("planar_zdot")));
  trajopt.AddLinearConstraint(x0(n_q + velocities_map.at("planar_rotydot"))
                               == xf(n_q + velocities_map.at("planar_rotydot")));
  trajopt.AddLinearConstraint(x0(n_q + velocities_map.at("left_hip_pindot"))
                               == xf(n_q + velocities_map.at("right_hip_pindot")));
  trajopt.AddLinearConstraint(x0(n_q + velocities_map.at("left_knee_pindot"))
                               == xf(n_q + velocities_map.at("right_knee_pindot")));
  trajopt.AddLinearConstraint(x0(n_q + velocities_map.at("right_hip_pindot"))
                               == xf(n_q + velocities_map.at("left_hip_pindot")));
  trajopt.AddLinearConstraint(x0(n_q + velocities_map.at("right_knee_pindot"))
                               == xf(n_q + velocities_map.at("left_knee_pindot")));

  // u periodic constraint
  auto u0 = trajopt.input(0);
  auto uf = trajopt.input(N - 1);
  trajopt.AddLinearConstraint(u0(actuators_map.at("left_hip_torque")) == uf(
                                 actuators_map.at("right_hip_torque")));
  trajopt.AddLinearConstraint(u0(actuators_map.at("right_hip_torque")) == uf(
                                 actuators_map.at("left_hip_torque")));
  trajopt.AddLinearConstraint(u0(actuators_map.at("left_knee_torque")) == uf(
                                 actuators_map.at("right_knee_torque")));
  trajopt.AddLinearConstraint(u0(actuators_map.at("right_knee_torque")) == uf(
                                 actuators_map.at("left_knee_torque")));

  // Knee joint limits
  auto x = trajopt.state();
  trajopt.AddConstraintToAllKnotPoints(x(positions_map.at("left_knee_pin")) >=
                                        5.0 / 180.0 * M_PI);
  trajopt.AddConstraintToAllKnotPoints(x(positions_map.at("right_knee_pin")) >=
                                        5.0 / 180.0 * M_PI);
  trajopt.AddConstraintToAllKnotPoints(x(positions_map.at("left_knee_pin")) <=
                                        M_PI / 2.0);
  trajopt.AddConstraintToAllKnotPoints(x(positions_map.at("right_knee_pin")) <=
                                        M_PI / 2.0);

  // hip joint limits
  trajopt.AddConstraintToAllKnotPoints(x(positions_map.at("left_hip_pin")) >=
                                        -M_PI / 2.0);
  trajopt.AddConstraintToAllKnotPoints(x(positions_map.at("right_hip_pin")) >=
                                        -M_PI / 2.0);
  trajopt.AddConstraintToAllKnotPoints(x(positions_map.at("left_hip_pin")) <=
                                        M_PI / 2.0);
  trajopt.AddConstraintToAllKnotPoints(x(positions_map.at("right_hip_pin")) <=
                                        M_PI / 2.0);

  // x-distance constraint constraints
  trajopt.AddLinearConstraint(x0(positions_map.at("planar_x")) == 0);
  trajopt.AddLinearConstraint(xf(positions_map.at("planar_x")) ==
                               stride_length);

  // make sure it's left stance
  trajopt.AddLinearConstraint(x0(positions_map.at("left_hip_pin")) <=
                               x0(positions_map.at("right_hip_pin")));

  // Add cost
  MatrixXd R = MatrixXd::Identity(n_u, n_u);
  MatrixXd Q = MatrixXd::Zero(n_x, n_x);
  for (int i = 0; i < n_v; i++) {
    Q(i + n_q, i + n_q) = 1;
  }
  auto u = trajopt.input();
  trajopt.AddRunningCost(u.transpose()*R * u);
  trajopt.AddRunningCost(x.transpose()*Q * x);


  // trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
  //                          "Print file", "snopt_find_model.out");
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", 10000);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                           0);

  // initial guess if the file exists
  if (!init_file.empty()) {
    VectorXd w0 = readCSV(directory + init_file).col(0);
    trajopt.SetInitialGuessForAllVariables(w0);
  }

  // Testing
  cout << "Choose the best solver: " << drake::solvers::ChooseBestSolver(trajopt).name() << endl;

  cout << prefix << " starts solving...\n";
  const MathematicalProgramResult result = Solve(trajopt);
  auto solution_result = result.get_solution_result();
  cout << prefix << " " << solution_result << " | ";
  cout << "Cost:" << result.get_optimal_cost() << " | ";
  VectorXd w_sol = result.GetSolution(trajopt.decision_variables());
  cout << "w_sol norm:" << w_sol.norm() << endl;

  // Check which solver we are using
  cout << "Solver: " << result.get_solver_id().name() << endl;

  // Store solution of all decision variables
  writeCSV(directory + prefix + string("w.csv"), w_sol);

  // Store a bool indicating whehter the problem was solved.
  VectorXd is_success(1);
  if (result.is_success()) is_success << 1;
  else is_success << 0;
  writeCSV(directory + prefix + string("is_success.csv"), is_success);

  // Store the vectors and matrices
  // cout << "\nStoring vectors and matrices into csv.\n";
  double c_double = result.get_optimal_cost();
  VectorXd c(1); c << c_double;
  writeCSV(directory + prefix + string("c.csv"), c);

  return;
}

}  // namespace misc
}  // namespace goldilocks_models
}  // namespace dairlib

