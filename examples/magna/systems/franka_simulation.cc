#include <math.h>

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <drake/common/find_resource.h>
#include <drake/geometry/scene_graph.h>
#include <drake/lcm/drake_lcm.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/roll_pitch_yaw.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <gflags/gflags.h>
#include <optional>

#include "common/find_resource.h"
#include "examples/magna/systems/franka_common.h"
#include "systems/robot_lcm_systems.h"

using drake::geometry::SceneGraph;
using drake::lcm::DrakeLcm;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::lcm::LcmInterfaceSystem;

using Eigen::Vector3d;
using Eigen::VectorXd;

DEFINE_string(franka_input_channel, "FRANKA_INPUT",
              "LCM channel for receiving Franka input");
DEFINE_string(franka_state_channel, "FRANKA_STATE",
              "LCM channel for sending Franka state");
DEFINE_double(franka_state_publish_rate, 1000.0,
              "Rate (in Hz) at which to publish Franka state over LCM");
DEFINE_double(simulation_dt, 0.0001, "Simulation time step");
DEFINE_double(realtime_rate, 1.0, "Target realtime rate for simulation");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");

namespace dairlib {

using systems::AddActuationRecieverAndStateSenderLcm;

namespace examples {
namespace magna {
namespace systems {

int RunFrankaSimulation() {
  // load urdf and sphere
  DiagramBuilder<double> builder;
  double sim_dt = FLAGS_simulation_dt;
  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, sim_dt);

  auto franka_index = AddFrankaToPlant(&plant, &scene_graph,
                                       std::nullopt /* no end effector */);
  plant.Finalize();
  /* -------------------------------------------------------------------------------------------*/

  drake::lcm::DrakeLcm drake_lcm(FLAGS_lcm_url);
  auto lcm =
      builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>(&drake_lcm);
  AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, FLAGS_franka_input_channel,
      FLAGS_franka_state_channel, FLAGS_franka_state_publish_rate,
      franka_index, true, 0.0, true);

  int nq = plant.num_positions();
  int nv = plant.num_velocities();

  auto diagram = builder.Build();

  drake::systems::Simulator<double> simulator(*diagram);

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(FLAGS_realtime_rate);

  auto& plant_context = diagram->GetMutableSubsystemContext(
      plant, &simulator.get_mutable_context());

  VectorXd q = VectorXd::Zero(nq);

  q << 0, M_PI / 8, 0, -3 * M_PI / 4, 0, 7 * M_PI / 8, 0;
  plant.SetPositions(&plant_context, q);

  VectorXd v = VectorXd::Zero(nv);
  plant.SetVelocities(&plant_context, v);

  simulator.Initialize();
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  dairlib::examples::magna::systems::RunFrankaSimulation();
}
