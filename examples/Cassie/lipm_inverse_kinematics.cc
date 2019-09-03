#include <gflags/gflags.h>

#include <vector>

#include "examples/Cassie/cassie_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/trajectory_optimization/dircon_position_data.h"
#include "systems/trajectory_optimization/dircon_distance_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"
#include "multibody/multibody_utils.h"

#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/analysis/simulator.h"

#include "dairlib/lcmt_robot_output.hpp"

DEFINE_int64(N, 10, "The number of knot points.");
DEFINE_int64(T, 1, "The duration for visualization.");

using drake::solvers::VectorXDecisionVariable;
using drake::systems::Simulator;
using drake::multibody::Body;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;


namespace dairlib {

using systems::trajectory_optimization::PositionKinematicConstraint;

void doMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::multibody::MultibodyPlant<double> plant;
  // Create plant
  addCassieMultibody(&plant);
  plant.Finalize();

  int num_q = plant.num_positions();

  // Create kinematic constraint set

  // Contact constraints on the left foot
  const Body<double>& left_toe = plant.GetBodyByName("toe_left");
  Vector3d pt_front_contact(-0.0457, 0.112, 0);
  Vector3d pt_rear_contact(0.088, 0, 0);
  auto front_constraint = DirconPositionData<double>(plant, left_toe,
      pt_front_contact);
  auto rear_constraint = DirconPositionData<double>(plant, left_toe,
      pt_rear_contact);

  // Swing constraints on the right foot
  const Body<double>& right_toe = plant.GetBodyByName("toe_right");
  auto swing_front_constraint = DirconPositionData<double>(plant, right_toe,
      pt_front_contact);
  auto swing_rear_constraint = DirconPositionData<double>(plant, right_toe,
      pt_rear_contact);

  // Loop closure constraints
  const auto & thigh_left = plant.GetBodyByName("thigh_left");
  const auto & heel_spring_left = plant.GetBodyByName("heel_spring_left");
  const auto & thigh_right = plant.GetBodyByName("thigh_right");
  const auto & heel_spring_right = plant.GetBodyByName("heel_spring_right");
  double rod_length = 0.5012;  // from cassie_utils
  Vector3d pt_on_heel_spring = Vector3d(.11877, -.01, 0.0);
  Vector3d pt_on_thigh_left = Vector3d(0.0, 0.0, 0.045);
  Vector3d pt_on_thigh_right = Vector3d(0.0, 0.0, -0.045);
  auto distance_constraint_left = DirconDistanceData<double>(plant,
                                  thigh_left, pt_on_thigh_left,
                                  heel_spring_left, pt_on_heel_spring,
                                  rod_length);
  auto distance_constraint_right = DirconDistanceData<double>(plant,
                                   thigh_right, pt_on_thigh_right,
                                   heel_spring_right, pt_on_heel_spring,
                                   rod_length);

  // Pelvis constraint
  const auto & pelvis = plant.GetBodyByName("pelvis");
  auto pelvis_constraint = DirconPositionData<double>(plant, pelvis,
      Vector3d::Zero());

  // Assemble constraint set
  // Skip redundant constraint, rear-x and front-swing-x
  std::vector<int> skip_ind;
  skip_ind.push_back(3);
  skip_ind.push_back(14);

  std::vector<DirconKinematicData<double>*> constraint_list;
  constraint_list.push_back(&front_constraint);
  constraint_list.push_back(&rear_constraint);
  constraint_list.push_back(&distance_constraint_left);
  constraint_list.push_back(&distance_constraint_right);
  constraint_list.push_back(&pelvis_constraint);
  constraint_list.push_back(&swing_front_constraint);
  constraint_list.push_back(&swing_rear_constraint);
  auto constraint_set =
      DirconKinematicDataSet<double>(plant, &constraint_list, skip_ind);

  std::vector<bool> is_relative;
  is_relative.push_back(false);  // front-x
  is_relative.push_back(false);  // front-y
  is_relative.push_back(false);  // front-z
  is_relative.push_back(false);  // rear-x
  is_relative.push_back(false);  // rear-y
  is_relative.push_back(false);  // rear-z
  is_relative.push_back(false);  // distance-left
  is_relative.push_back(false);  // distance-right
  is_relative.push_back(false);  // pelvis-x
  is_relative.push_back(false);  // pelvis-y
  is_relative.push_back(false);  // pelvis-z
  is_relative.push_back(false);  // swing-front-x
  is_relative.push_back(false);  // swing-front-y
  is_relative.push_back(false);  // swing-front-z
  is_relative.push_back(false);  // swing-rear-x
  is_relative.push_back(false);  // swing-rear-y
  is_relative.push_back(false);  // swing-rear-yrear-z
  int num_relative = 0;
  int num_constraints = constraint_set.countConstraints();

  // Create optmization problem
  auto program = std::make_unique<drake::solvers::MathematicalProgram>();
  std::vector<VectorXDecisionVariable> q_vector;
  for (int i = 0; i < FLAGS_N; i++) {
    std::string name = "q(" + std::to_string(i) + ")";
    q_vector.push_back(program->NewContinuousVariables(num_q, name));
  }

  VectorXDecisionVariable relative_vars =
    program->NewContinuousVariables(num_relative, "relative");

  std::map<std::string, int> position_map =
      multibody::makeNameToPositionsMap(plant);
  int knee_joint_left = position_map.at("knee_joint_left");
  int knee_joint_right = position_map.at("knee_joint_right");
  int ankle_spring_joint_left = position_map.at("ankle_spring_joint_left");
  int ankle_spring_joint_right = position_map.at("ankle_spring_joint_right");

  // Create and add constraints to optimization program
  VectorXd quat(4);
  quat << 1, 0, 0, 0;
  for (int i = 0; i < FLAGS_N; i++) {
    // Build constraint offset
    // This should probably come from evaluation of desired trajectories for
    // COM and swing leg (e.g. LIPM trajectories)
    // These crude trajectories, using i and N
    VectorXd constant_offset = Eigen::VectorXd::Zero(num_constraints);
    constant_offset(0) = .05;  // front-x 
    constant_offset(7) = .3*i/FLAGS_N - .15;  // pelvis-x, offset by one from the skip
    constant_offset(8) = -.1;    // pelvis-y (should follow LIPM, not be constant)
    constant_offset(9) = .9;  // pelvis-z
    constant_offset(10)  = .3*i/FLAGS_N; //swing-front-x
    constant_offset(11)  = -.2; //swing-front-y
    constant_offset(12)  = .1*4/FLAGS_N/FLAGS_N*i*(FLAGS_N - 1 - i); //swing-front-z
    constant_offset(13)  = -.2; //swing-rear-y
    constant_offset(14)  = .1*4/FLAGS_N/FLAGS_N*i*(FLAGS_N - 1 - i); //swing-rear-z


    auto constraint = std::make_shared<PositionKinematicConstraint<double>>(
      plant, constraint_set, is_relative, constant_offset);

    program->AddConstraint(constraint, {q_vector.at(i), relative_vars});

    program->AddBoundingBoxConstraint(quat, quat, q_vector.at(i).head(4));

    program->AddBoundingBoxConstraint(0, 0, q_vector.at(i)(knee_joint_left));
    program->AddBoundingBoxConstraint(0, 0, q_vector.at(i)(knee_joint_right));
    program->AddBoundingBoxConstraint(0, 0,
        q_vector.at(i)(ankle_spring_joint_left));
    program->AddBoundingBoxConstraint(0, 0,
        q_vector.at(i)(ankle_spring_joint_right));
  }

  // Add cost
  for (int i = 1; i < FLAGS_N; i++) {
    auto delta_q = q_vector.at(i) - q_vector.at(i-1);
    program->AddQuadraticCost(delta_q.dot(delta_q));
  }

  // Create and set initial guess
  VectorXd q0 = VectorXd::Zero(num_q);
  q0(position_map.at("base_z")) = .9;
  q0(position_map.at("base_qw")) = 1.0;
  q0(position_map.at("hip_roll_left")) = 0.1;
  q0(position_map.at("hip_roll_right")) = -0.1;
  q0(position_map.at("hip_yaw_left")) = -0.01;
  q0(position_map.at("hip_yaw_right")) = 0.01;
  q0(position_map.at("hip_pitch_left")) = .269;
  q0(position_map.at("hip_pitch_right")) = .269;
  q0(position_map.at("knee_left")) = -.744;
  q0(position_map.at("knee_right")) = -.744;
  q0(position_map.at("ankle_joint_left")) = .81;
  q0(position_map.at("ankle_joint_right")) = .81;
  q0(position_map.at("toe_left")) = -60.0 * M_PI / 180.0;
  q0(position_map.at("toe_right")) = -60.0 * M_PI / 180.0;

  // program->SetSolverOption(drake::solvers::SnoptSolver::id(),
  //                          "Print file", "snopt.out");
  // program->SetSolverOption(drake::solvers::SnoptSolver::id(),
  //                          "Major iterations limit", 100);

  VectorXd init_guess(num_q * FLAGS_N);
  for (int i = 0; i < FLAGS_N; i++) {
    init_guess.segment(i * num_q, num_q) = q0;
  }
  const auto result = Solve(*program, init_guess);

  MatrixXd q_mat(num_q, FLAGS_N);
  for (int i = 0; i < FLAGS_N; i++) {
    q_mat.col(i) = result.GetSolution(q_vector.at(i));
  }
  std::cout << q_mat << std::endl;


  // Visualize solution
  // Create trajectory
  MatrixXd v_mat = MatrixXd::Zero(plant.num_velocities(), FLAGS_N);
  MatrixXd x_mat(num_q + plant.num_velocities(), FLAGS_N);
  x_mat << q_mat, v_mat;
  VectorXd t_vec = VectorXd::LinSpaced(FLAGS_N, 0, FLAGS_T);

  auto trajectory =
      drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
          t_vec, x_mat);

  // Create state publisher.
  drake::lcm::DrakeLcm lcm;
  drake::systems::DiagramBuilder<double> builder;
  auto state_pub = builder.AddSystem(drake::systems::lcm::LcmPublisherSystem::
      Make<dairlib::lcmt_robot_output>("CASSIE_STATE", &lcm, .01));
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(plant);
  builder.Connect(*state_sender, *state_pub);

  auto trajectory_source = builder.AddSystem<drake::systems::TrajectorySource>(
      trajectory);
  builder.Connect(*trajectory_source, *state_sender);

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);
  simulator.set_target_realtime_rate(1);
  simulator.Initialize();
  simulator.StepTo(FLAGS_T);
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  dairlib::doMain(argc, argv);
  return 0;
}
