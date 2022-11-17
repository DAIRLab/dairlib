#include <drake/common/yaml/yaml_io.h>
#include <drake/systems/framework/diagram_builder.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multipose_visualizer.h"
#include "systems/controllers/kinematic_centroidal_mpc/kinematic_centroidal_visualizer.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/system_utils.h"

namespace dairlib {

using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using Eigen::VectorXd;

namespace examples {

DEFINE_string(channel_reference, "KCMPC_REF",
              "The name of the channel where the reference trajectories from "
              "MPC are published");
DEFINE_int32(num_poses, 10, "Number of poses per mode to draw");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  /**** Initialize all the leaf systems ****/
  drake::lcm::DrakeLcm lcm("udpm://239.255.76.67:7667?ttl=0");

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::multibody::MultibodyPlant<double> plant(0.0);
  AddCassieMultibody(
      &plant, nullptr, true,
      "examples/Cassie/urdf/cassie_fixed_springs_conservative.urdf", false,
      false);
  plant.Finalize();
  auto context = plant.CreateDefaultContext();

  VectorXd alpha_scale = VectorXd::Ones(FLAGS_num_poses) -
                         VectorXd::LinSpaced(FLAGS_num_poses, 0.0, 0.8);
  auto visualizer = std::make_unique<multibody::MultiposeVisualizer>(
      FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"),
      FLAGS_num_poses, alpha_scale.array().square());

  /****** Leaf Systems ******/

  auto trajectory_visualizer = builder.AddSystem<KinematicCentroidalVisualizer>(
      plant, context.get(), std::move(visualizer));

  // Run lcm-driven simulation
  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("centroidal_mpc_visualizer"));
  auto diagram_context = owned_diagram->CreateDefaultContext();

  auto simulator = std::make_unique<drake::systems::Simulator<double>>(
      *owned_diagram, std::move(diagram_context));

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_timestamped_saved_traj> loop(
      &lcm, std::move(owned_diagram), trajectory_visualizer,
      FLAGS_channel_reference, true);
  DrawAndSaveDiagramGraph(*loop.get_diagram());

  loop.Simulate();

  return 0;
}
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::DoMain(argc, argv);
}
