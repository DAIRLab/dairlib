#include <gflags/gflags.h>
#include <memory>
#include <chrono>


#include "drake/lcm/drake_lcm.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"

namespace dairlib {
using drake::systems::DiagramBuilder;
using drake::geometry::SceneGraph;
using drake::systems::Context;
using drake::systems::Simulator;
using drake::multibody::RevoluteJoint;


// Simulation parameters.
DEFINE_double(target_realtime_rate, 1.0,  
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_bool(time_stepping, false, "If 'true', the plant is modeled as a "
    "discrete system with periodic updates. "
    "If 'false', the plant is modeled as a continuous system.");
DEFINE_double(dt, 1e-3, "The step size to use for compliant, ignored for time_stepping)");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  drake::lcm::DrakeLcm lcm;

  DiagramBuilder<double> builder;
  auto tree = std::make_unique<RigidBodyTree<double>>();

  // NOTE: will need to change path as appropriate
  std::string full_name = "/home/posa/workspace/dairlib/examples/Cassie/urdf/cassie_v2.urdf";

  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      full_name,
      drake::multibody::joints::kFixed, tree.get());

  const double time_step = FLAGS_time_stepping ? FLAGS_dt : 0.0;

  auto plant = builder.AddSystem<drake::systems::RigidBodyPlant<double>>(std::move(tree), time_step);

  auto input_source = builder.AddSystem<drake::systems::ConstantVectorSource<double>>(
      Eigen::VectorXd::Zero(plant->get_num_actuators()));

  builder.Connect(input_source->get_output_port(),
                  plant->get_input_port(0));


  auto visualizer = builder.AddSystem<drake::systems::DrakeVisualizer>(plant->get_rigid_body_tree(), &lcm);  
  builder.Connect(plant->state_output_port(), visualizer->get_input_port(0));

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
  x0(map.at("knee_left")) = -.7;
  x0(map.at("knee_right")) = -.7;
  x0(map.at("ankle_joint_left")) = 1;
  x0(map.at("ankle_joint_right")) = 1;

  std::cout << x0 << std::endl;

  if (!FLAGS_time_stepping) {
    drake::systems::ContinuousState<double>& state = context.get_mutable_continuous_state();
    state.SetFromVector(x0);
  } else {
    drake::systems::BasicVector<double>& state = context.get_mutable_discrete_state(0); 
    state.SetFromVector(x0);
  }

  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  auto start = std::chrono::high_resolution_clock::now();
  simulator.AdvanceTo(5);
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = 
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  std::cout << "5 second simulation took " << duration.count() <<
               " milliseconds." << std::endl;

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
