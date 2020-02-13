#include <gflags/gflags.h>

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

#include "drake/solvers/solve.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/constraint.h"
#include "solvers/optimization_utils.h"
#include "systems/trajectory_optimization/dircon_position_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"

#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"

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
using drake::solvers::VectorXDecisionVariable;
using drake::solvers::MatrixXDecisionVariable;
using drake::symbolic::Variable;
using drake::symbolic::Expression;
using std::vector;
using std::shared_ptr;
using std::cout;
using std::endl;
using std::string;
using std::map;

using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::Body;
using drake::multibody::Parser;
using drake::systems::rendering::MultibodyPositionToGeometryPose;

// using Isometry3 = Eigen::Transform<Scalar, 3, Eigen::Isometry>

namespace dairlib {
namespace goldilocks_models {

using systems::trajectory_optimization::HybridDircon;
using systems::trajectory_optimization::DirconDynamicConstraint;
using systems::trajectory_optimization::DirconKinematicConstraint;
using systems::trajectory_optimization::DirconOptions;
using systems::trajectory_optimization::DirconKinConstraintType;
using systems::SubvectorPassThrough;

void simpleTrajOpt(double stride_length, double duration, int iter,
                   string directory,
                   string init_file,
                   string output_prefix) {

  drake::systems::DiagramBuilder<double> builder;
  MultibodyPlant<double> plant;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  Parser parser(&plant, &scene_graph);

  std::string full_name =
    FindResourceOrThrow("examples/goldilocks_models/PlanarWalkerWithTorso.urdf");
  parser.AddModelFromFile(full_name);
  plant.mutable_gravity_field().set_gravity_vector(
    -9.81 * Eigen::Vector3d::UnitZ());
  plant.WeldFrames(
    plant.world_frame(), plant.GetFrameByName("base"),
      drake::math::RigidTransform<double>(Vector3d::Zero()));
  plant.Finalize();


  auto context = plant.CreateDefaultContext();
  auto plant_symb_smrt_ptr =
    plant.ToSymbolic(); // return std::unique_ptr<System<Expression>>
  auto context_symb_smrt_ptr = plant_symb_smrt_ptr->CreateDefaultContext();
  context_symb_smrt_ptr->SetTimeStateAndParametersFrom(*context);
  auto plant_symb_ptr = plant_symb_smrt_ptr.get();

  // MultibodyPlant<Expression> plant_symb;
  // Parser parser_symb(&plant_symb); // parser doesn't support this
  // std::string full_name =
  //     FindResourceOrThrow("examples/goldilocks_models/PlanarWalkerWithTorso.urdf");
  // parser_symb.AddModelFromFile(full_name);
  // plant_symb.AddForceElement<drake::multibody::UniformGravityFieldElement>(
  //     -9.81 * Eigen::Vector3d::UnitZ());
  // plant_symb.WeldFrames(
  //     plant_symb.world_frame(), plant_symb.GetFrameByName("base"),
  //     drake::math::RigidTransform<Expression>(Vector3d::Zero()).GetAsIsometry3());
  // plant_symb.Finalize();

  // drake::systems::DiagramBuilder<Expression> builder_symb;
  // auto system_symb = *builder_symb.AddSystem<MultibodyPlant<Expression>>(plant_symb_smrt_ptr); //return MultibodyPlant<Expression>
  // auto diagram_symb = builder_symb.Build();




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
  // int n_x = n_q + n_v;
  // int n_u = plant.num_actuators();
  // std::cout<<"n_x = "<<n_x<<"\n";
  // std::cout<<"n_u = "<<n_u<<"\n";

  const Body<double>& left_lower_leg = plant.GetBodyByName("left_lower_leg");
  const Body<double>& right_lower_leg =
    plant.GetBodyByName("right_lower_leg");

  Vector3d pt;
  pt << 0, 0, -.5;
  bool isXZ = true;

  auto leftFootConstraint = DirconPositionData<double>(plant, left_lower_leg,
                            pt, isXZ);
  auto rightFootConstraint = DirconPositionData<double>(plant,
                             right_lower_leg,
                             pt, isXZ);

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
  leftOptions.setConstraintRelative(0,
                                    true); //TODO: ask what is relative constraint here?
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
  min_dt.push_back(
    .01);   // bound for time difference between adjacent samples in the first mode // See HybridDircon constructor
  min_dt.push_back(
    .01);   // bound for time difference between adjacent samples in the second mode
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

  auto trajopt = std::make_shared<HybridDircon<double>>(plant,
                 num_time_samples, min_dt, max_dt, dataset_list, options_list);

  // trajopt->AddDurationBounds(duration, duration); // You can comment this out to not put any constraint on the time

  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Print file", "snopt.out");
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", iter);
  trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                           0);






  // for (uint j = 0; j < num_time_samples.size(); j++) {
  //   trajopt->drake::systems::trajectory_optimization::MultipleShooting::
  //       SetInitialTrajectory(init_u_traj, init_x_traj);
  //   trajopt->SetInitialForceTrajectory(j, init_l_traj[j], init_lc_traj[j],
  //                                     init_vc_traj[j]);
  // }







  // Periodicity constraints
  auto x0 = trajopt->initial_state();
  // auto xf = trajopt->final_state();
  auto xf = trajopt->state_vars_by_mode(num_time_samples.size() - 1,
                                        num_time_samples[num_time_samples.size() - 1] - 1);

  //Careful! if you have a string typo, the code still runs and the mapped value will be 0.
  trajopt->AddLinearConstraint(x0(positions_map.at("planar_z")) == xf(
                                 positions_map.at("planar_z")));
  trajopt->AddLinearConstraint(x0(positions_map.at("planar_roty")) == xf(
                                 positions_map.at("planar_roty")));
  trajopt->AddLinearConstraint(x0(positions_map.at("left_hip_pin")) == xf(
                                 positions_map.at("right_hip_pin")));
  trajopt->AddLinearConstraint(x0(positions_map.at("left_knee_pin")) == xf(
                                 positions_map.at("right_knee_pin")));
  trajopt->AddLinearConstraint(x0(positions_map.at("right_hip_pin")) == xf(
                                 positions_map.at("left_hip_pin")));
  trajopt->AddLinearConstraint(x0(positions_map.at("right_knee_pin")) == xf(
                                 positions_map.at("left_knee_pin")));

  trajopt->AddLinearConstraint(x0(n_q + velocities_map.at("planar_xdot"))
                           == xf(n_q + velocities_map.at("planar_xdot")));
  trajopt->AddLinearConstraint(x0(n_q + velocities_map.at("planar_zdot"))
                           == xf(n_q + velocities_map.at("planar_zdot")));
  trajopt->AddLinearConstraint(x0(n_q + velocities_map.at("planar_rotydot"))
                           == xf(n_q + velocities_map.at("planar_rotydot")));
  trajopt->AddLinearConstraint(x0(n_q + velocities_map.at("left_hip_pindot"))
                           == xf(n_q + velocities_map.at("right_hip_pindot")));
  trajopt->AddLinearConstraint(x0(n_q + velocities_map.at("left_knee_pindot"))
                           == xf(n_q + velocities_map.at("right_knee_pindot")));
  trajopt->AddLinearConstraint(x0(n_q + velocities_map.at("right_hip_pindot"))
                           == xf(n_q + velocities_map.at("left_hip_pindot")));
  trajopt->AddLinearConstraint(x0(n_q + velocities_map.at("right_knee_pindot"))
                           == xf(n_q + velocities_map.at("left_knee_pindot")));

  // u periodic constraint
  auto u0 = trajopt->input(0);
  auto uf = trajopt->input(N - 1);
  trajopt->AddLinearConstraint(u0(actuators_map.at("left_hip_torque")) == uf(
                                 actuators_map.at("right_hip_torque")));
  trajopt->AddLinearConstraint(u0(actuators_map.at("right_hip_torque")) == uf(
                                 actuators_map.at("left_hip_torque")));
  trajopt->AddLinearConstraint(u0(actuators_map.at("left_knee_torque")) == uf(
                                 actuators_map.at("right_knee_torque")));
  trajopt->AddLinearConstraint(u0(actuators_map.at("right_knee_torque")) == uf(
                                 actuators_map.at("left_knee_torque")));

  // Knee joint limits
  auto x = trajopt->state();
  trajopt->AddConstraintToAllKnotPoints(x(positions_map.at("left_knee_pin")) >=
                                        5.0 / 180.0 * M_PI);
  trajopt->AddConstraintToAllKnotPoints(x(positions_map.at("right_knee_pin")) >=
                                        5.0 / 180.0 * M_PI);
  trajopt->AddConstraintToAllKnotPoints(x(positions_map.at("left_knee_pin")) <=
                                        M_PI / 2.0);
  trajopt->AddConstraintToAllKnotPoints(x(positions_map.at("right_knee_pin")) <=
                                        M_PI / 2.0);

  // hip joint limits
  trajopt->AddConstraintToAllKnotPoints(x(positions_map.at("left_hip_pin")) >=
                                        -M_PI / 2.0);
  trajopt->AddConstraintToAllKnotPoints(x(positions_map.at("right_hip_pin")) >=
                                        -M_PI / 2.0);
  trajopt->AddConstraintToAllKnotPoints(x(positions_map.at("left_hip_pin")) <=
                                        M_PI / 2.0);
  trajopt->AddConstraintToAllKnotPoints(x(positions_map.at("right_hip_pin")) <=
                                        M_PI / 2.0);

  // x-distance constraint constraints
  trajopt->AddLinearConstraint(x0(positions_map.at("planar_x")) == 0);
  trajopt->AddLinearConstraint(xf(positions_map.at("planar_x")) ==
                               stride_length);

  // make sure it's left stance
  // trajopt->AddLinearConstraint(x0(positions_map.at("left_hip_pin")) <= x0(positions_map.at("right_hip_pin")));


  // swing foot clearance constraint (not finished; how to do this?)
  /*
  VectorXDecisionVariable xmid = trajopt->state_vars_by_mode(0,
                                 floor(num_time_samples[0] / 2));

  auto context_symb_ptr =
    context_symb_smrt_ptr.get(); //return std::unique_ptr< Context< T > >
  // const Isometry3<Expression> right_lower_leg_pose = plant_symb_ptr->EvalBodyPoseInWorld(*context_symb_ptr, right_lower_leg);
  const auto right_lower_leg_pose = plant_symb_ptr->EvalBodyPoseInWorld(
                                      *context_symb_ptr, right_lower_leg);



  // KinematicsCache<Expression> cache = tree.doKinematics(xmid.head(n_q), xmid.tail(n_v));
  // tree.CalcBodyPoseInWorldFrame(cache, tree.get_body(rightLegIdx)).translation();
  // VectorXDecisionVariable swingFootPos    = tree.CalcBodyPoseInWorldFrame(cache, tree.get_body(rightLegIdx)).translation();
  // MatrixXDecisionVariable swingFootRotmat = tree.CalcBodyPoseInWorldFrame(cache, tree.get_body(rightLegIdx)).linear();
  // VectorXDecisionVariable swingFootContactPtPos = swingFootPos + swingFootRotmat * pt;

  // auto leftFootConstraint_symb = DirconPositionData<Expression>(tree, leftLegIdx, pt,
  //                                                        isXZ);
  */


  // add cost
  const double R = 10;  // Cost on input effort
  auto u = trajopt->input();
  trajopt->AddRunningCost(u.transpose()*R * u);
  MatrixXd Q = MatrixXd::Zero(2 * n_q, 2 * n_q);
  for (int i = 0; i < n_q; i++) {
    Q(i + n_q, i + n_q) = 10;
  }
  trajopt->AddRunningCost(x.transpose()*Q * x);

  // initial guess if the file exists
  if (!init_file.empty()) {
    MatrixXd z0 = readCSV(directory + init_file);
    trajopt->SetInitialGuessForAllVariables(z0);
  }

  std::cout << "Solving DIRCON (based on MultipleShooting)\n";
  auto start = std::chrono::high_resolution_clock::now();
  const auto result = Solve(*trajopt, trajopt->initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  // trajopt->PrintSolution();
  std::cout << "Solve time:" << elapsed.count() << std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() << std::endl;

  // store the solution of the decision variable
  VectorXd z = result.GetSolution(
                 trajopt->decision_variables()); //solution of all decision variables
  writeCSV(directory + output_prefix + string("z.csv"), z);

  // store the time, state, and input at knot points
  // VectorXd time_at_knot_point = trajopt->GetSampleTimes();
  // MatrixXd state_at_knot_point = trajopt->GetStateSamples();
  // MatrixXd input_at_knot_point = trajopt->GetInputSamples();
  // writeCSV(directory + string("simple_traj_time_at_knots.csv"), time_at_knot_point);
  // writeCSV(directory + string("simple_traj_state_at_knots.csv"), state_at_knot_point);
  // writeCSV(directory + string("simple_traj_input_at_knots.csv"), input_at_knot_point);
  // std::cout<<"time_at_knot_point = \n"<<time_at_knot_point<<"\n";
  // std::cout<<state_at_knot_point.rows()<<", "<<state_at_knot_point.cols()<<"\n";
  // std::cout<<"state_at_knot_point = \n"<<state_at_knot_point<<"\n";
  // std::cout<<input_at_knot_point.rows()<<", "<<input_at_knot_point.cols()<<"\n";
  // std::cout<<"input_at_knot_point = \n"<<input_at_knot_point<<"\n";


  // visualizer
  const PiecewisePolynomial<double> pp_xtraj =
    trajopt->ReconstructStateTrajectory(result);
  multibody::connectTrajectoryVisualizer(&plant, &builder, &scene_graph,
                                         pp_xtraj);

  auto diagram = builder.Build();

  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(.1);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
  }

  return ;
}

}  // namespace goldilocks_models
}  // namespace dairlib


int main() {

  double stride_length = 0.3;
  double duration = .5;
  int iter = 500;
  string directory = "examples/goldilocks_models/find_models/data/";
  string init_file = "";
  // string init_file = "testing_z.csv";
  string output_prefix = "testing_";

  dairlib::goldilocks_models::simpleTrajOpt(stride_length, duration, iter,
      directory, init_file, output_prefix);
}

