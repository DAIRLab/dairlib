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
// #include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

#include "systems/robot_lcm_systems.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "systems/primitives/subvector_pass_through.h"

#include "examples/Cassie/cassie_utils.h"

namespace dairlib{
  using dairlib::systems::SubvectorPassThrough;

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
  std::unique_ptr<RigidBodyTree<double>> tree = makeFixedBaseCassieTreePointer();

  drake::systems::DiagramBuilder<double> builder;

  if (FLAGS_simulation_type != "timestepping")
    FLAGS_dt = 0.0;
  auto plant = builder.AddSystem<drake::systems::RigidBodyPlant<double>>(std::move(tree), FLAGS_dt);

    // Note: this sets identical contact parameters across all object pairs:

  drake::systems::CompliantMaterial default_material;
  default_material.set_youngs_modulus(FLAGS_youngs_modulus)
      .set_dissipation(FLAGS_dissipation)
      .set_friction(FLAGS_us, FLAGS_ud);
  plant->set_default_compliant_material(default_material);
  drake::systems::CompliantContactModelParameters model_parameters;
  model_parameters.characteristic_radius = FLAGS_contact_radius;
  model_parameters.v_stiction_tolerance = FLAGS_v_tol;
  plant->set_contact_model_parameters(model_parameters);

  // Create input receiver.
  auto input_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>("CASSIE_INPUT", &lcm));
  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(plant->get_rigid_body_tree());
  builder.Connect(input_sub->get_output_port(),
                  input_receiver->get_input_port(0));

  // Create state publisher.
  auto state_pub = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<dairlib::lcmt_robot_output>("CASSIE_STATE", &lcm));
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(plant->get_rigid_body_tree());
  state_pub->set_publish_period(1.0/200.0);

  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
    input_receiver->get_output_port(0).size(),
    0,
    plant->get_input_port(0).size());

  builder.Connect(input_receiver->get_output_port(0),
                  passthrough->get_input_port());
  builder.Connect(passthrough->get_output_port(),
                  plant->get_input_port(0));

  builder.Connect(plant->state_output_port(), state_sender->get_input_port_state());

  builder.Connect(state_sender->get_output_port(0),
                  state_pub->get_input_port());


  // Creates and adds LCM publisher for visualization.
  //builder.AddVisualizer(&lcm);
  // auto visualizer = builder.AddSystem<systems::DrakeVisualizer>(plant->get_rigid_body_tree(), &lcm);  
  // Raw state vector to visualizer.
  // builder.Connect(plant->state_output_port(), visualizer->get_input_port(0));


  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);
  drake::systems::Context<double>& context =
      diagram->GetMutableSubsystemContext(*plant, &simulator.get_mutable_context());

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(
      plant->get_rigid_body_tree().get_num_positions() +
      plant->get_rigid_body_tree().get_num_velocities());
  std::map<std::string, int>  map =
      plant->get_rigid_body_tree().computePositionNameToIndexMap();
  x0(map.at("hip_pitch_left")) = .269;
  x0(map.at("hip_pitch_right")) = .269;
  // x0(map.at("achilles_hip_pitch_left")) = -.44;
  // x0(map.at("achilles_hip_pitch_right")) = -.44;
  // x0(map.at("achilles_heel_pitch_left")) = -.105;
  // x0(map.at("achilles_heel_pitch_right")) = -.105;
  x0(map.at("knee_left")) = -.644;
  x0(map.at("knee_right")) = -.644;
  x0(map.at("ankle_joint_left")) = .792;
  x0(map.at("ankle_joint_right")) = .792;

  // x0(map.at("toe_crank_left")) = -90.0*M_PI/180.0;
  // x0(map.at("toe_crank_right")) = -90.0*M_PI/180.0;

  // x0(map.at("plantar_crank_pitch_left")) = 90.0*M_PI/180.0;
  // x0(map.at("plantar_crank_pitch_right")) = 90.0*M_PI/180.0;

  x0(map.at("toe_left")) = -60.0*M_PI/180.0;
  x0(map.at("toe_right")) = -60.0*M_PI/180.0;

  std::vector<int> fixed_joints;
  fixed_joints.push_back(map.at("hip_pitch_left"));
  fixed_joints.push_back(map.at("hip_pitch_right"));
  fixed_joints.push_back(map.at("knee_left"));
  fixed_joints.push_back(map.at("knee_right"));

  auto q0 = solvePositionConstraints(
      plant->get_rigid_body_tree(),
      x0.head(plant->get_rigid_body_tree().get_num_positions()), fixed_joints);
  x0.head(plant->get_rigid_body_tree().get_num_positions()) = q0;

  std::cout << q0 << std::endl;


  if (FLAGS_simulation_type != "timestepping") {
    drake::systems::ContinuousState<double>& state = context.get_mutable_continuous_state(); 
    std::cout << "Continuous " << state.size() << std::endl;
    state.SetFromVector(x0);
    // state[4] = 1;
    // state[3] = 0;
    // state[4] = 0;
  } else {
    std::cout << "ngroups "<< context.get_num_discrete_state_groups() <<  std::endl;
    drake::systems::BasicVector<double>& state = context.get_mutable_discrete_state(0); 
    std::cout << "Discrete " << state.size() << std::endl;
    state.SetFromVector(x0);
    // state[4] = 1;
    // state[3] = 0;
    // state[4] = 0;
  }

  // int num_u = plant->get_num_actuators();
  // auto zero_input = Eigen::MatrixXd::Zero(num_u,1);
  // context.FixInputPort(0, zero_input);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();

  lcm.StartReceiveThread();

  simulator.StepTo(std::numeric_limits<double>::infinity());
  // simulator.StepTo(.001);
  return 0;
}

}  // namespace drake

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}