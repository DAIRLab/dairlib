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

inline const double PI = 3.14159265358979323846;

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

/// This is the offset from the Panda's link7 frame to its flange where an end
/// effector can be attached.
inline const Eigen::Vector3d TOOL_ATTACHMENT_FRAME = {0, 0, 0.107};
inline const drake::math::RigidTransform<double> T_EE_L7 =
    drake::math::RigidTransform<double>(
        drake::math::RotationMatrix<double>(
            drake::math::RollPitchYaw<double>(3.1415, 0, 0)),
        TOOL_ATTACHMENT_FRAME);

drake::multibody::ModelInstanceIndex AddFrankaToPlant(
    drake::multibody::MultibodyPlant<double>* plant,
    drake::geometry::SceneGraph<double>* scene_graph,
    std::optional<std::string> end_effector_model_path) {
  drake::multibody::Parser parser(plant, scene_graph);
  parser.SetAutoRenaming(true);

  drake::multibody::ModelInstanceIndex franka_index = parser.AddModelsFromUrl(
      "package://drake_models/franka_description/urdf/panda_arm.urdf")[0];
  drake::math::RigidTransform<double> X_WI =
      drake::math::RigidTransform<double>::Identity();
  plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("panda_link0"),
                    X_WI);
  if (end_effector_model_path.has_value()) {
    parser.AddModels(FindResourceOrThrow(end_effector_model_path.value()));
    plant->WeldFrames(plant->GetFrameByName("panda_link7"),
                      plant->GetFrameByName("end_effector_flange"), T_EE_L7);
  }

  return franka_index;
}

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
      franka_index);

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

  q << 0, PI / 8, 0, -3 * PI / 4, 0, 7 * PI / 8, -PI / 4;
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
