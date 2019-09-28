#include "drake/common/find_resource.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_common.h"
#include "drake/examples/kuka_iiwa_arm/iiwa_lcm.h"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/manipulation/util/sim_diagram_builder.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/primitives/discrete_derivative.h"

namespace dairlib {
using drake::systems::RigidBodyPlant;
using drake::systems::Simulator;
using drake::manipulation::util::SimDiagramBuilder;
using drake::manipulation::kuka_iiwa::IiwaStatusReceiver;

int doMain(int argc, char* argv[]) {

  drake::systems::DiagramBuilder<double> builder;
  // Adds a plant.
  const char* kModelPath =
      "drake/manipulation/models/iiwa_description/"
      "urdf/iiwa14_polytope_collision.urdf";
  const std::string urdf = drake::FindResourceOrThrow(kModelPath);
  int num_joints;


  RigidBodyTree<double> tree;
  {
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
        urdf, drake::multibody::joints::kFixed, &tree);
    drake::multibody::AddFlatTerrainToWorld(&tree, 100., 10.);
    //plant = builder.AddPlant(std::move(tree));
    num_joints = tree.get_num_positions();
  }

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
  const std::string channel_x = "IIWA_STATUS";

  // Create state subscriber and state receiver.
  auto state_sub = builder.AddSystem(
                     drake::systems::lcm::LcmSubscriberSystem::Make <
                     drake::lcmt_iiwa_status > (channel_x, lcm));
  auto state_receiver = builder.AddSystem<IiwaStatusReceiver>(num_joints);

  // Create visualizer
  auto visualizer = builder.AddSystem<drake::systems::DrakeVisualizer>(tree, lcm);
  visualizer->set_publish_period(1.0/30.0);

  // Get the state from the LCM message and add it to the Visualizer
  auto desired_state_from_position = builder.AddSystem<
      drake::systems::StateInterpolatorWithDiscreteDerivative>(
          num_joints, 0.005);
  desired_state_from_position->set_name("desired_state_from_position");

  builder.Connect(state_sub->get_output_port(),
                  state_receiver->get_input_port());
  builder.Connect(state_receiver->get_position_measured_output_port(),
                  desired_state_from_position->get_input_port());
  builder.Connect(desired_state_from_position->get_output_port(),
                  visualizer->get_input_port(0));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  auto stepper = std::make_unique<Simulator<double>>(*diagram,
                 std::move(context));

  stepper->set_publish_every_time_step(false);
  stepper->set_publish_at_initialization(false);
  stepper->set_target_realtime_rate(1.0);
  stepper->Initialize();

  drake::log()->info("visualizer started");

  stepper->AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

} // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::doMain(argc, argv);
}
