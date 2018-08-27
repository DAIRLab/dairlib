#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_pd_config.hpp"
#include "systems/robot_lcm_systems.h"
#include "systems/controllers/linear_controller.h"
#include "systems/controllers/mpc_balance_controller.h"
#include "systems/controllers/pd_config_lcm.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/cassie_solver.h"

using namespace std;
namespace dairlib {

using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;

int doMain() {
  drake::systems::DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm lcm;

  RigidBodyTree<double> tree;
  buildFloatBaseCassieTree(tree);
  //buildFixedBaseCassieTree(tree);
  const std::string channel_x = "CASSIE_STATE";
  const std::string channel_u = "CASSIE_INPUT";
  //const std::string channel_config = "PD_CONFIG";

  // Create state receiver.
  auto state_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(channel_x, &lcm));
  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(tree);
  builder.Connect(state_sub->get_output_port(),
                  state_receiver->get_input_port(0));

  // Create config receiver.
  //auto config_sub = builder.AddSystem(
  //    LcmSubscriberSystem::Make<dairlib::lcmt_pd_config>(channel_config, &lcm));
  //auto config_receiver = builder.AddSystem<systems::PDConfigReceiver>(tree);
  //builder.Connect(config_sub->get_output_port(),
  //                config_receiver->get_input_port(0));

  // Create command sender.
  auto command_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(channel_u, &lcm));
  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(tree);

  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());

  // tianze:  find desired 
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(
      tree.get_num_positions() +
      tree.get_num_velocities());

  std::map<std::string, int>  map =
      tree.computePositionNameToIndexMap();



  x0(map.at("base_x")) = 0.0;
  x0(map.at("base_y")) = 0.0;
  x0(map.at("base_z")) = 2.2;

  x0(map.at("base_roll")) = 0.0;
  x0(map.at("base_pitch")) = 0.0;
  x0(map.at("base_yaw")) = 0.0;

  x0(map.at("hip_roll_left")) = 0.1;
  x0(map.at("hip_roll_right")) = -0.1;
  x0(map.at("hip_yaw_left")) = 0;
  x0(map.at("hip_yaw_right")) = 0;
  x0(map.at("hip_pitch_left")) = .269;
  x0(map.at("hip_pitch_right")) = .269;
  // x0(map.at("achilles_hip_pitch_left")) = -.44;
  // x0(map.at("achilles_hip_pitch_right")) = -.44;
  // x0(map.at("achilles_heel_pitch_left")) = -.105;
  // x0(map.at("achilles_heel_pitch_right")) = -.105;
  x0(map.at("knee_left")) = -.744;
  x0(map.at("knee_right")) = -.744;
  x0(map.at("ankle_joint_left")) = .81;
  x0(map.at("ankle_joint_right")) = .81;
  
  // x0(map.at("toe_crank_left")) = -90.0*M_PI/180.0;
  // x0(map.at("toe_crank_right")) = -90.0*M_PI/180.0;
  
  // x0(map.at("plantar_crank_pitch_left")) = 90.0*M_PI/180.0;
  // x0(map.at("plantar_crank_pitch_right")) = 90.0*M_PI/180.0;
  
  x0(map.at("toe_left")) = -60*M_PI/180.0;
  x0(map.at("toe_right")) = -60*M_PI/180.0;

  cout << "********** initial state guess************" << endl;
  cout << x0.head(tree.get_num_positions()) << endl;
  
  //some joints are fixed
  std::vector<int> fixed_joints;
  fixed_joints.push_back(map.at("base_yaw"));
  fixed_joints.push_back(map.at("hip_roll_left"));
  fixed_joints.push_back(map.at("hip_roll_right"));

  //solver for a feasible state
  const int num_tree_constraints = 2;
  const int num_contacts = 4;
  const int num_constraints_per_contact = 3;
  const int num_contact_constraints = num_contacts * num_constraints_per_contact;
  const int num_efforts = tree.get_num_actuators();

    const int num_total_constraints = num_tree_constraints + num_contact_constraints;
    VectorXd u_init = VectorXd::Zero(num_efforts);
    VectorXd lambda_init = VectorXd::Zero(num_total_constraints);

    auto solution = solveCassieStandingFixedConstraints(
      tree,
      x0.head(tree.get_num_positions()),
      u_init,
      lambda_init,
      num_total_constraints,
      fixed_joints);

    VectorXd q0 = solution.head(tree.get_num_positions());  
    VectorXd u0 = solution.segment(tree.get_num_positions(),tree.get_num_actuators());
    VectorXd lambda0 = solution.tail(num_total_constraints);

    x0.head(tree.get_num_positions()) = q0;
  // add linear controller to the system
  // constructor of LinearController include number of position,velocity and input
  auto controller = builder.AddSystem<systems::MpcBalanceController>(
      tree.get_num_positions(), tree.get_num_velocities(),
      tree.get_num_actuators(),tree,x0,u0,lambda0);
  
  // get the robot state to compute the control output
  builder.Connect(state_receiver->get_output_port(0),
                  controller->get_input_port_output());
  
  // receive the gains
  //builder.Connect(config_receiver->get_output_port(0),
  //                controller->get_input_port_config());

  std::cout << controller->get_output_port(0).size() << std::endl;
  std::cout << command_sender->get_input_port(0).size() << std::endl;
  // send the result to the commander
  builder.Connect(controller->get_output_port(0),
                  command_sender->get_input_port(0));

  command_pub->set_publish_period(1.0/50.0);


  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice 
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper = std::make_unique<drake::systems::Simulator<double>>(*diagram, std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();

  lcm.StartReceiveThread();

  drake::log()->info("controller started");

  stepper->StepTo(std::numeric_limits<double>::infinity());

  return 0;
}


}

int main() { return dairlib::doMain(); }
