
#include <dairlib/lcmt_elastoplastic_network.hpp>
#include <dairlib/lcmt_material_points.hpp>
#include <dairlib/lcmt_radio_out.hpp>
#include <drake/common/find_resource.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_publisher_system.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "examples/deform/deform_utils.h"
#include "examples/deform/mpm_model_reducer.h"
#include "examples/deform/parameter_headers/deform_settings.h"
#include "examples/deform/parameter_headers/lcm_channels.h"
#include "examples/deform/parameter_headers/reduced_model_params.h"
#include "multibody/multibody_utils.h"
#include "solvers/lcs_factory.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/franka_kinematics.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

namespace dairlib {

using dairlib::solvers::LCSFactory;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RigidTransform;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::systems::InputPort;
using drake::systems::InputPortIndex;
using drake::systems::OutputPort;
using drake::systems::OutputPortIndex;
using drake::systems::TriggerType;
using drake::systems::TriggerTypeSet;
using drake::systems::lcm::LcmInterfaceSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::MatrixXd;

using Eigen::Vector3d;
using Eigen::VectorXd;
using multibody::MakeNameToPositionsMap;
using multibody::MakeNameToVelocitiesMap;
using std::vector;

DEFINE_bool(is_simulation, true, "True for simulation, false for hardware");
DEFINE_string(lcm_url, "udpm://239.255.76.67:7667?ttl=0",
              "LCM URL with IP, port, and TTL settings");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  drake::lcm::DrakeLcm lcm(FLAGS_lcm_url);

  // Load parameters.
  DeformSettings deform_settings = drake::yaml::LoadYamlFile<DeformSettings>(
      "examples/deform/parameters/deform_settings.yaml");
  ReducedModelParams reduced_model_params =
      drake::yaml::LoadYamlFile<ReducedModelParams>(
          deform_settings.reduced_model_params_file);
  std::string lcm_channels_file =
      FLAGS_is_simulation ? deform_settings.lcm_channels_simulation_file
                          : deform_settings.lcm_channels_hardware_file;
  DeformLcmChannels lcm_channel_params =
      drake::yaml::LoadYamlFile<DeformLcmChannels>(lcm_channels_file);

  // Piece together the diagram.
  DiagramBuilder<double> builder;

  // 1) Franka state receiver.
  MultibodyPlant<double> plant_franka(0.0);
  ModelInstanceIndex robot_index =
      AddFrankaToPlant(&plant_franka, nullptr, true, true, true);
  plant_franka.Finalize();
  auto franka_context = plant_franka.CreateDefaultContext();
  auto franka_state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_franka);

  // 2) MPM object state receiver.
  auto mpm_points_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_material_points>(
          lcm_channel_params.mpm_channel, &lcm));

  // 3) Convert MPM points to a reduced elastoplastic network model.
  if (reduced_model_params.reduction_type !=
      ReducedModelTypes::kSupportDirections) {
    throw std::runtime_error("Other model reduction types not implemented.");
  }
  auto mpm_reducer =
      builder.AddSystem<dairlib::systems::MpmPointsToReducedModel>(
          reduced_model_params.support_directions,
          reduced_model_params.connections);
  auto reduced_model_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_elastoplastic_network>(
          lcm_channel_params.reduced_model_channel, &lcm,
          TriggerTypeSet({TriggerType::kForced})));
  builder.Connect(mpm_points_sub->get_output_port(),
                  mpm_reducer->get_input_port_lcmt_material_points());
  builder.Connect(mpm_reducer->get_output_port_lcmt_elastoplastic_network(),
                  reduced_model_publisher->get_input_port());

  // 4) LCS state.
  auto franka_kinematics = builder.AddSystem<systems::FrankaKinematics>(
      plant_franka, franka_context.get(), kEndEffectorName, false);
  builder.Connect(franka_state_receiver->get_output_port(),
                  franka_kinematics->get_input_port_franka_state());

  //////////////////////////////////////////////////////////////////////////////

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("deform_controller"));
  DrawAndSaveDiagramGraph(*owned_diagram);

  // Run lcm-driven simulation.  The buffer size argument is needed to ensure
  // the latest messages are used in the control loop.  See
  // https://github.com/DAIRLab/dairlib/pull/366 for more details.
  int lcm_buffer_size = 200;
  std::shared_ptr<Diagram<double>> shared_diagram = std::move(owned_diagram);
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm, shared_diagram, franka_state_receiver,
      lcm_channel_params.robot_state_channel, true, lcm_buffer_size);

  LcmHandleSubscriptionsUntil(
      &lcm, [&]() { return mpm_points_sub->GetInternalMessageCount() > 1; });
  loop.Simulate();
  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }
