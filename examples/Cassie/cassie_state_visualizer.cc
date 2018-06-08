#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "systems/primitives/subvector_pass_through.h"

#include "dairlib/lcmt_cassie_state.hpp"
#include "cassie_controller_lcm.h"
#include "datatypes/cassie_names.h"

namespace drake{

using std::endl;
using std::cout;
using dairlib::systems::SubvectorPassThrough;

int doMain() {
  RigidBodyTree<double> tree;
  parsers::urdf::AddModelInstanceFromUrdfFileToWorld("examples/Cassie/urdf/cassie_fixed_springs.urdf", multibody::joints::kFixed, &tree);


  std::cout << "bodies" << std::endl;
  for (int i = 0; i < tree.get_num_bodies(); i++)
    cout << tree.getBodyOrFrameName(i) << endl;
  std::cout << "actuators" << std::endl;
  for (int i = 0; i < tree.get_num_actuators(); i++)
    cout << tree.actuators[i].name_ << endl;
  std::cout << "positions" << std::endl;
  for (int i = 0; i < tree.get_num_positions(); i++)
    cout << tree.get_position_name(i) << endl;
  std::cout << "velocities" << std::endl;
  for (int i = 0; i < tree.get_num_velocities(); i++)
    cout << tree.get_velocity_name(i) << endl;


  drake::systems::DiagramBuilder<double> builder;
  lcm::DrakeLcm lcm;

  const std::string channel_x = "CASSIE_STATE";

  // Create state receiver.
  auto state_sub = builder.AddSystem(
      systems::lcm::LcmSubscriberSystem::Make<dairlib::lcmt_cassie_state>(channel_x, &lcm));
  auto state_receiver = builder.AddSystem<CassieStateReceiver>();
  builder.Connect(state_sub->get_output_port(),
                  state_receiver->get_input_port(0));


  auto publisher = builder.AddSystem<systems::DrakeVisualizer>(tree, &lcm);

  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
    state_receiver->get_output_port(0).size(),
    0,
    state_receiver->get_output_port(0).size() - 1);

  std::cout << state_receiver->get_output_port(0).size() << std::endl;
  std::cout << publisher->get_input_port(0).size() << std::endl;
  
  builder.Connect(state_receiver->get_output_port(0),
                  passthrough->get_input_port());
  builder.Connect(passthrough->get_output_port(),
                  publisher->get_input_port(0));


  publisher->set_publish_period(1.0/30.0);


  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  /// Use the simulator to drive at a fixed rate
  /// If set_publish_every_time_step is true, this publishes twice 
  /// Set realtime rate. Otherwise, runs as fast as possible
  auto stepper = std::make_unique<systems::Simulator<double>>(*diagram, std::move(context));
  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();

  lcm.StartReceiveThread();

  drake::log()->info("visualizer started");

  stepper->StepTo(std::numeric_limits<double>::infinity());

  return 0;
}


}

int main() { return drake::doMain(); }
