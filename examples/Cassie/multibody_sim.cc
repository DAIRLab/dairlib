#include <memory>

#include <gflags/gflags.h>
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/multibody/joints/floating_base_types.h" 
#include "drake/geometry/geometry_visualization.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"

#include "systems/robot_lcm_systems.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "systems/primitives/subvector_pass_through.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"

namespace dairlib {
using drake::systems::DiagramBuilder;
using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::Simulator;
using drake::multibody::RevoluteJoint;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using dairlib::systems::SubvectorPassThrough;


// Simulation parameters.
DEFINE_bool(floating_base, true, "Fixed or floating base model");

DEFINE_double(target_realtime_rate, 1.0,  
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_bool(time_stepping, false, "If 'true', the plant is modeled as a "
    "discrete system with periodic updates. "
    "If 'false', the plant is modeled as a continuous system.");
DEFINE_double(dt, 1e-4, "The step size to use for compliant, ignored for time_stepping)");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  const double time_step = FLAGS_time_stepping ? FLAGS_dt : 0.0;

  DiagramBuilder<double> builder;
  drake::lcm::DrakeLcm lcm;

  auto items = drake::multibody::AddMultibodyPlantSceneGraph(&builder,
      std::make_unique<MultibodyPlant<double>>(time_step));
  MultibodyPlant<double>& plant = items.plant;
  SceneGraph<double>& scene_graph = items.scene_graph;

  if (FLAGS_floating_base) {
    multibody::addFlatTerrain(&plant, &scene_graph, .8, .8);
  }

  addCassieMultibody(&plant, &scene_graph, FLAGS_floating_base);

  plant.Finalize();

  // Create input receiver.
  auto input_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>("CASSIE_INPUT",
                                                           &lcm));
  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(plant);
  builder.Connect(*input_sub, *input_receiver);

  // connect input receiver
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
    input_receiver->get_output_port(0).size(),
    0,
    plant.get_actuation_input_port().size());

  builder.Connect(*input_receiver, *passthrough);
  builder.Connect(passthrough->get_output_port(),
                  plant.get_actuation_input_port());

  // Create state publisher.
  auto state_pub = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_robot_output>("CASSIE_STATE",
                                                           &lcm, 1.0/200.0));
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(plant);

  // connect state publisher
  builder.Connect(plant.get_continuous_state_output_port(),
                  state_sender->get_input_port_state());

  builder.Connect(*state_sender, *state_pub);

  auto diagram = builder.Build();


  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  plant.GetJointByName<RevoluteJoint>("hip_pitch_left").
      set_angle(&plant_context, .269);
  plant.GetJointByName<RevoluteJoint>("knee_left").
      set_angle(&plant_context, -.644);
  plant.GetJointByName<RevoluteJoint>("ankle_joint_left").
      set_angle(&plant_context, .792);
  plant.GetJointByName<RevoluteJoint>("toe_left").
      set_angle(&plant_context, -M_PI/3);

  plant.GetJointByName<RevoluteJoint>("hip_pitch_right").
      set_angle(&plant_context, .269);
  plant.GetJointByName<RevoluteJoint>("knee_right").
      set_angle(&plant_context, -.644);
  plant.GetJointByName<RevoluteJoint>("ankle_joint_right").
      set_angle(&plant_context, .792);
  plant.GetJointByName<RevoluteJoint>("toe_right").
      set_angle(&plant_context, -M_PI/3);


  if (FLAGS_floating_base) {
    Eigen::Isometry3d transform;
    transform.linear() = Eigen::Matrix3d::Identity();;
    transform.translation() = Eigen::Vector3d(0, 0, 1.2);
    plant.SetFreeBodyPose(&plant_context, plant.GetBodyByName("pelvis"),
        transform);
  }

  Simulator<double> simulator(*diagram, std::move(diagram_context));

  if (!FLAGS_time_stepping) {
    // simulator.get_mutable_integrator()->set_maximum_step_size(0.01);
    // simulator.get_mutable_integrator()->set_target_accuracy(1e-1);
    // simulator.get_mutable_integrator()->set_fixed_step_mode(true);
    simulator.reset_integrator<drake::systems::RungeKutta2Integrator<double>>(
      *diagram, FLAGS_dt, &simulator.get_mutable_context());
  }

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.StepTo(std::numeric_limits<double>::infinity());

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
