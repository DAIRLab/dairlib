#include <gflags/gflags.h>
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "systems/primitives/subvector_pass_through.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "systems/robot_lcm_systems.h"
#include "examples/Cassie/cassie_utils.h"

namespace dairlib{

// Simulation parameters
// Flag to determine the model to use (Fixed or floating base)
DEFINE_string(model_type, "Fixed", 
              "Type of model to use - 'Fixed' or 'Floating' base");

using std::endl;
using std::cout;
using dairlib::systems::SubvectorPassThrough;
using drake::systems::Simulator;
using dairlib::systems::RobotOutputReceiver;

int doMain(int argc, char* argv[] ) {

  gflags::ParseCommandLineFlags(&argc, &argv, true);

  RigidBodyTree<double> tree;

  if (FLAGS_model_type == "Fixed") {
    buildFixedBaseCassieTree(tree);
  }
  else {
    buildFloatingBaseCassieTree(tree);
    // Adding the ground plane
    const double terrain_size = 4;
    const double terrain_depth = 0.05;
    //drake::multibody::AddFlatTerrainToWorld(&tree, terrain_size, terrain_depth);

  }

  cout << endl << "****bodies****" << endl;
  for (int i = 0; i < tree.get_num_bodies(); i++)
    cout << tree.getBodyOrFrameName(i) << endl;
  cout << endl << "****actuators****" << endl;
  for (int i = 0; i < tree.get_num_actuators(); i++)
    cout << tree.actuators[i].name_ << endl;
  cout << endl << "****positions****" << endl;
  for (int i = 0; i < tree.get_num_positions(); i++)
    cout << tree.get_position_name(i) << endl;
  cout << endl << "****velocities****" << endl;
  for (int i = 0; i < tree.get_num_velocities(); i++)
    cout << tree.get_velocity_name(i) << endl;


  drake::systems::DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm lcm;

  const std::string channel_x = "CASSIE_STATE";

  // Create state receiver.
  auto state_sub = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(channel_x, &lcm));
  auto state_receiver = builder.AddSystem<RobotOutputReceiver>(tree);
  builder.Connect(state_sub->get_output_port(),
                  state_receiver->get_input_port(0));


  auto publisher = builder.AddSystem<drake::systems::DrakeVisualizer>(tree, &lcm);

  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
    state_receiver->get_output_port(0).size(),
    0,
    publisher->get_input_port(0).size());

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
  auto stepper = std::make_unique<Simulator<double>>(*diagram, std::move(context));
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

int main(int argc, char* argv[]) { return dairlib::doMain(argc, argv); }
