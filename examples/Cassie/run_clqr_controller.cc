#include <memory>

#include <gflags/gflags.h>
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/multiplexer.h"

#include "cassie_utils.h"
#include "cassie_solver.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "multibody/rbp_utils.h"
#include "multibody/solve_multibody_constraints.h"
#include "systems/robot_lcm_systems.h"
#include "systems/controllers/clqr_controller.h"
#include "systems/framework/output_vector.h"
#include "systems/primitives/subvector_pass_through.h"

using std::cout;
using std::endl;
using std::vector;

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix;
using Eigen::Dynamic;
using drake::VectorX;
using drake::MatrixX;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::systems::BasicVector;
using drake::systems::ConstantVectorSource;
using drake::systems::Multiplexer;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;

using dairlib::multibody::SolveTreeConstraints;
using dairlib::multibody::CheckTreeConstraints;
using dairlib::multibody::SolveFixedPointConstraints;
using dairlib::multibody::CheckFixedPointConstraints;
using dairlib::multibody::SolveTreeAndFixedPointConstraints;
using dairlib::multibody::CheckTreeAndFixedPointConstraints;
using dairlib::multibody::SolveFixedPointFeasibilityConstraints;
using dairlib::multibody::utils::CalcTimeDerivativesUsingLambda;
using dairlib::systems::ClqrController;
using dairlib::systems::AffineParams;
using dairlib::systems::SubvectorPassThrough;
using dairlib::systems::OutputVector;



namespace dairlib{


// Simulation parameters.
DEFINE_double(timestep, 1e-5, "The simulator time step (s)");
DEFINE_double(youngs_modulus, 1e8, "The contact model's Young's modulus (Pa)");
DEFINE_double(us, 0.7, "The static coefficient of friction");
DEFINE_double(ud, 0.7, "The dynamic coefficient of friction");
DEFINE_double(v_tol, 0.01,
              "The maximum slipping speed allowed during stiction (m/s)");
DEFINE_double(dissipation, 2, "The contact model's dissipation (s/m)");
DEFINE_double(contact_radius, 2e-4,
              "The characteristic scale of contact patch (m)");
DEFINE_string(simulation_type, "compliant", "The type of simulation to use: "
              "'compliant' or 'timestepping'");
DEFINE_double(dt, 1e-3, "The step size to use for "
              "'simulation_type=timestepping' (ignored for "
              "'simulation_type=compliant'");


int do_main(int argc, char* argv[]) { 

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  
  drake::lcm::DrakeLcm lcm;

  std::unique_ptr<RigidBodyTree<double>> tree = makeFixedBaseCassieTreePointer("examples/Cassie/urdf/cassie_v2.urdf");
  std::unique_ptr<RigidBodyTree<double>> tree_solver = makeFixedBaseCassieTreePointer("examples/Cassie/urdf/cassie_v2.urdf");
  
  const int num_efforts = tree->get_num_actuators();
  const int num_positions = tree->get_num_positions();
  const int num_velocities = tree->get_num_velocities();
  const int num_states = num_positions + num_velocities;
  const int num_constraints = tree->getNumPositionConstraints();
  
  cout << "Number of actuators: " << num_efforts << endl;
  cout << "Number of generalized coordinates: " << num_positions << endl;
  cout << "Number of generalized velocities: " << num_velocities << endl;
  
  drake::systems::DiagramBuilder<double> builder;

  if (FLAGS_simulation_type != "timestepping")
    FLAGS_dt = 0.0;
  
  auto plant = builder.AddSystem<drake::systems::RigidBodyPlant<double>>(std::move(tree), FLAGS_dt);

  // plant to pass to the solver
  RigidBodyPlant<double> plant_solver(std::move(tree_solver), FLAGS_dt);
  
  drake::systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(FLAGS_youngs_modulus)
      .set_dissipation(FLAGS_dissipation)
      .set_friction(FLAGS_us, FLAGS_ud);
  plant->set_default_compliant_material(default_material);
  plant_solver.set_default_compliant_material(default_material);
  drake::systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = FLAGS_contact_radius;
  model_parameters.v_stiction_tolerance = FLAGS_v_tol;
  plant->set_contact_model_parameters(model_parameters);
  plant_solver.set_contact_model_parameters(model_parameters);


  VectorXd x0 = VectorXd::Zero(num_states);
  std::map<std::string, int> map = plant->get_rigid_body_tree().computePositionNameToIndexMap();

  for(auto elem: map)
  {
      cout << elem.first << " " << elem.second << endl;
  }

  x0(map.at("hip_roll_left")) = 0.1;
  x0(map.at("hip_roll_right")) = -0.01;
  x0(map.at("hip_yaw_left")) = 0.01;
  x0(map.at("hip_yaw_right")) = 0.01;
  x0(map.at("hip_pitch_left")) = -.169;
  x0(map.at("hip_pitch_right")) = .269;
  x0(map.at("knee_left")) = -.744;
  x0(map.at("knee_right")) = -.744;
  x0(map.at("ankle_joint_left")) = .81;
  x0(map.at("ankle_joint_right")) = .81;
  
  // x0(map.at("toe_crank_left")) = -90.0*M_PI/180.0;
  // x0(map.at("toe_crank_right")) = -90.0*M_PI/180.0;
  
  // x0(map.at("plantar_crank_pitch_left")) = 90.0*M_PI/180.0;
  // x0(map.at("plantar_crank_pitch_right")) = 90.0*M_PI/180.0;
  
  x0(map.at("toe_left")) = -30.0*M_PI/180.0;
  x0(map.at("toe_right")) = -60.0*M_PI/180.0;

  std::vector<int> fixed_joints;

  fixed_joints.push_back(map.at("hip_roll_left"));
  fixed_joints.push_back(map.at("hip_roll_right"));
  fixed_joints.push_back(map.at("hip_yaw_left"));
  fixed_joints.push_back(map.at("hip_yaw_right"));
  fixed_joints.push_back(map.at("hip_pitch_left"));
  fixed_joints.push_back(map.at("hip_pitch_right"));
  
  //fixed_joints.push_back(map.at("ankle_joint_left"));
  //fixed_joints.push_back(map.at("ankle_joint_right"));
 
  fixed_joints.push_back(map.at("knee_left"));
  fixed_joints.push_back(map.at("knee_right"));
  fixed_joints.push_back(map.at("toe_left"));
  fixed_joints.push_back(map.at("toe_right"));

  
  VectorXd x_start = VectorXd::Zero(num_states);
  x_start.head(num_positions) = SolveTreeConstraints(
          plant->get_rigid_body_tree(), VectorXd::Zero(num_positions));

  VectorXd q0 = SolveTreeConstraints(
          plant->get_rigid_body_tree(), x0.head(num_positions), fixed_joints);

  cout << "x_start: " << x_start.transpose() << endl;
  cout << "q0: " << q0.transpose() << endl;

  x0.head(num_positions) = q0;

  VectorXd x_init = x0;
  cout << "xinit: " << x_init.transpose() << endl;
  VectorXd q_init = x0.head(num_positions);
  VectorXd v_init = x0.tail(num_velocities);
  VectorXd u_init = VectorXd::Zero(num_efforts);

  KinematicsCache<double> k_cache_init = 
    plant->get_rigid_body_tree().doKinematics(q_init, v_init);

  // Throw an exception if the Joint angles are not within limits
  DRAKE_DEMAND(CassieJointsWithinLimits(plant->get_rigid_body_tree(), x_init));


  const int num_constraint_forces = num_constraints;
  bool print_debug = false;
  VectorXd lambda_init = VectorXd::Zero(num_constraint_forces);

  cout << "Starting to solve" << endl;
  vector<VectorXd> sol_tfp = SolveCassieTreeAndFixedPointConstraints(
      plant_solver, num_constraint_forces, q_init, u_init, lambda_init, fixed_joints, print_debug);

  VectorXd q_sol = sol_tfp.at(0);
  VectorXd u_sol = sol_tfp.at(1);
  VectorXd lambda_sol = sol_tfp.at(2);

  VectorXd x_sol(num_states);
  x_sol << q_sol, VectorXd::Zero(num_velocities);
  DRAKE_DEMAND(CassieJointsWithinLimits(plant->get_rigid_body_tree(), x_sol));

  cout << "********************q_sol*************************" << endl;
  cout << q_sol.transpose() << endl;
  cout << "********************u_sol*************************" << endl;
  cout << u_sol.transpose() << endl;
  cout << "******************lambda_sol*********************" << endl;
  cout << lambda_sol.transpose() << endl;

  MatrixXd J_collision = MatrixXd::Zero(0, 0);

  //Parameter matrices for LQR
  MatrixXd Q = MatrixXd::Identity(
          num_states - 2*num_constraints, num_states - 2*num_constraints);
  //Q corresponding to the positions
  MatrixXd Q_p = MatrixXd::Identity(
          num_positions - num_constraints, num_positions - num_constraints)*1.0;
  //Q corresponding to the velocities
  MatrixXd Q_v = MatrixXd::Identity(
          num_velocities - num_constraints, num_velocities - num_constraints)*1.0;
  Q.block(0, 0, Q_p.rows(), Q_p.cols()) = Q_p;
  Q.block(num_positions - num_constraints, num_positions - num_constraints, Q_v.rows(), Q_v.cols()) = Q_v;
  MatrixXd R = MatrixXd::Identity(num_efforts, num_efforts)*1.0;

  //Building the controller
  auto clqr_controller = builder.AddSystem<ClqrController>(plant,
                                                           x_sol,
                                                           u_sol,
                                                           lambda_sol,
                                                           J_collision,
                                                           Q,
                                                           R);
  VectorXd K_vec = clqr_controller->GetKVec();
  VectorXd C = u_sol; 
  VectorXd x_desired = x_sol;
  cout << "K: " << K_vec.transpose() << endl;
  cout << "C: " << C.transpose() << endl;
  cout << "xdes: " << x_desired.transpose() << endl;

  VectorXd params_vec(num_states*num_efforts + num_efforts + num_states);
  params_vec << K_vec, C, x_desired;
  AffineParams params(num_states, num_efforts);
  params.SetDataVector(params_vec);
  auto constant_params_source = builder.AddSystem<ConstantVectorSource<double>>(params);

  auto control_output = builder.AddSystem<SubvectorPassThrough<double>>(
          (clqr_controller->get_output_port(0)).size(), 0, (clqr_controller->get_output_port(0)).size() - 1);


  const string channel_x = "CASSIE_STATE";
  const string channel_u = "CASSIE_INPUT";
  const string channel_config = "PD_CONFIG";

  
  // Create state publisher.
  auto state_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(channel_x, &lcm));
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(plant->get_rigid_body_tree());
  state_pub->set_publish_period(1.0/200.0);
  builder.Connect(state_sender->get_output_port(0),
                  state_pub->get_input_port());


  // Create state receiver.
  auto state_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(channel_x, &lcm));
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant->get_rigid_body_tree());
  builder.Connect(state_sub->get_output_port(),
                  state_receiver->get_input_port(0));

  // Create command sender.
  auto command_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(channel_u, &lcm));
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant->get_rigid_body_tree());
  command_pub->set_publish_period(1.0/200.0);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());


  builder.Connect(plant->state_output_port(),
                  state_sender->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(0), 
                  clqr_controller->get_input_port_info());
  builder.Connect(constant_params_source->get_output_port(),
                  clqr_controller->get_input_port_params());
  builder.Connect(clqr_controller->get_output_port(0),
                  command_sender->get_input_port(0));
  builder.Connect(clqr_controller->get_output_port(0),
                  control_output->get_input_port());
  builder.Connect(control_output->get_output_port(),
                  plant->actuator_command_input_port()); 

  auto diagram = builder.Build();
  
  drake::systems::Simulator<double> simulator(*diagram);
  drake::systems::Context<double>& context =
      diagram->GetMutableSubsystemContext(*plant, &simulator.get_mutable_context());
  

  if (FLAGS_simulation_type != "timestepping") {
    drake::systems::ContinuousState<double>& state = context.get_mutable_continuous_state(); 
    state.SetFromVector(x_start);
  }
  
  //auto zero_input = Eigen::MatrixXd::Zero(num_efforts, 1);
  //context.FixInputPort(0, zero_input);
  
  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();
  
  lcm.StartReceiveThread();
  
  simulator.StepTo(std::numeric_limits<double>::infinity());
  return 0;
}

}  // namespace drake

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
