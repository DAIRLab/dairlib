//
// Created by brian on 5/17/22.
//

#include <memory>

#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/perception/camera_utils.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/discrete_time_delay.h"
#include "drake/geometry/render_vtk/factory.h"
#include "drake/systems/sensors/rgbd_sensor.h"
#include "drake/systems/sensors/image_to_lcm_image_array_t.h"

namespace dairlib {
using drake::geometry::SceneGraph;
using drake::geometry::MakeRenderEngineVtk;
using drake::geometry::RenderEngineVtkParams;
using drake::multibody::ContactResultsToLcmSystem;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using drake::math::RotationMatrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

// Simulation parameters.
DEFINE_bool(floating_base, true, "Fixed or floating base model");

DEFINE_double(target_realtime_rate, 1.0,
              "Desired rate relative to real time.  See documentation for "
              "Simulator::set_target_realtime_rate() for details.");
DEFINE_bool(time_stepping, true,
            "If 'true', the plant is modeled as a "
            "discrete system with periodic updates. "
            "If 'false', the plant is modeled as a continuous system.");
DEFINE_double(dt, 8e-5,
              "The step size to use for time_stepping, ignored for continuous");
DEFINE_double(v_stiction, 1e-3, "Stiction tolernace (m/s)");
DEFINE_double(penetration_allowance, 1e-5,
              "Penetration allowance for the contact model. Nearly equivalent"
              " to (m)");
DEFINE_double(end_time, std::numeric_limits<double>::infinity(),
              "End time for simulator");
DEFINE_double(publish_rate, 1000, "Publish rate for simulator");
DEFINE_double(init_height, .7,
              "Initial starting height of the pelvis above "
              "ground");
DEFINE_double(toe_spread, .15, "Initial toe spread in m.");
DEFINE_bool(spring_model, true, "Use a URDF with or without legs springs");

DEFINE_string(radio_channel, "CASSIE_VIRTUAL_RADIO" ,"LCM channel for virtual radio command");
DEFINE_string(channel_u, "CASSIE_INPUT",
              "LCM channel to receive controller inputs on");
DEFINE_double(actuator_delay, 0.0,
              "Duration of actuator delay. Set to 0.0 by default.");
DEFINE_bool(publish_efforts, true, "Flag to publish the efforts.");

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Plant/System initialization
  DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const std::string renderer_name = "multibody_renderer_vtk";
  scene_graph.AddRenderer(renderer_name,
                          drake::geometry::
                          MakeRenderEngineVtk(drake::geometry::RenderEngineVtkParams()));

  const double time_step = FLAGS_time_stepping ? FLAGS_dt : 0.0;
  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(time_step);
  if (FLAGS_floating_base) {
    multibody::AddFlatTerrain(&plant, &scene_graph, .8, .8);
  }

  std::string urdf;
  if (FLAGS_spring_model) {
    urdf = "examples/Cassie/urdf/cassie_v2.urdf";
  } else {
    urdf = "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  }

  AddCassieMultibody(&plant, &scene_graph, FLAGS_floating_base, urdf,
                     FLAGS_spring_model, true);
  plant.Finalize();

  plant.set_penetration_allowance(FLAGS_penetration_allowance);
  plant.set_stiction_tolerance(FLAGS_v_stiction);

  // Create lcm systems.
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  auto passthrough = systems::AddActuationRecieverAndStateSenderLcm(
      &builder, plant, lcm, FLAGS_channel_u, "CASSIE_STATE_SIMULATION",
      FLAGS_publish_rate, FLAGS_publish_efforts, FLAGS_actuator_delay);

  // Contact Information
  ContactResultsToLcmSystem<double>& contact_viz =
  *builder.template AddSystem<ContactResultsToLcmSystem<double>>(plant);
  contact_viz.set_name("contact_visualization");
  auto& contact_results_publisher = *builder.AddSystem(
      LcmPublisherSystem::Make<drake::lcmt_contact_results_for_viz>(
          "CASSIE_CONTACT_DRAKE", lcm, 1.0 / FLAGS_publish_rate));
  contact_results_publisher.set_name("contact_results_publisher");

  // Sensor aggregator and publisher of lcmt_cassie_out
  auto radio_sub = builder.AddSystem(
      LcmSubscriberSystem::Make<dairlib::lcmt_radio_out>(FLAGS_radio_channel, lcm));
  const auto& sensor_aggregator =
      AddImuAndAggregator(&builder, plant, passthrough->get_output_port());

  auto sensor_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_cassie_out>(
          "CASSIE_OUTPUT", lcm, 1.0 / FLAGS_publish_rate));

  // connect leaf systems
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());
  builder.Connect(plant.get_contact_results_output_port(),
                  contact_viz.get_input_port(0));
  builder.Connect(contact_viz.get_output_port(0),
                  contact_results_publisher.get_input_port());
  builder.Connect(radio_sub->get_output_port(),
                  sensor_aggregator.get_input_port_radio());
  builder.Connect(sensor_aggregator.get_output_port(0),
                  sensor_pub->get_input_port());


  const auto& [color_camera, depth_camera] =
  camera::MakeD415CameraModel(renderer_name);
  const std::optional<drake::geometry::FrameId> parent_body_id =
      plant.GetBodyFrameIdIfExists(plant.GetFrameByName("pelvis").body().index());

  drake::math::RigidTransform<double> cam_transform =
      drake::math::RigidTransform<double>(
          drake::math::RollPitchYaw<double>(-2.6, 0.0, -1.57),
              Eigen::Vector3d(0.05, 0, -0.15));

  auto camera = builder.AddSystem<drake::systems::sensors::RgbdSensor>(
      parent_body_id.value(), cam_transform, color_camera, depth_camera);

  builder.Connect(scene_graph.get_query_output_port(),
                  camera->query_object_input_port());

  auto image_to_lcm_image_array =
      builder.AddSystem<drake::systems::sensors::ImageToLcmImageArrayT>();
  image_to_lcm_image_array->set_name("converter");
  const auto& cam_port = image_to_lcm_image_array->
      DeclareImageInputPort<drake::systems::sensors::PixelType::kDepth16U>(
      "camera_0");
  builder.Connect(camera->depth_image_16U_output_port(), cam_port);

  auto image_array_lcm_publisher = builder.AddSystem(LcmPublisherSystem::Make<drake::lcmt_image_array>(
      "DRAKE_RGBD_CAMERA_IMAGES", lcm,
      1.0 / 10));
  image_array_lcm_publisher->set_name("rgbd_publisher");
  builder.Connect(image_to_lcm_image_array->image_array_t_msg_output_port(),
                  image_array_lcm_publisher->get_input_port());
  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
                                       diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set initial conditions of the simulation
  VectorXd q_init, u_init, lambda_init;
  double mu_fp = 0;
  double min_normal_fp = 70;
  double toe_spread = FLAGS_toe_spread;
  // Create a plant for CassieFixedPointSolver.
  // Note that we cannot use the plant from the above diagram, because after the
  // diagram is built, plant.get_actuation_input_port().HasValue(*context)
  // throws a segfault error
  drake::multibody::MultibodyPlant<double> plant_for_solver(0.0);
  AddCassieMultibody(&plant_for_solver, nullptr,
                     FLAGS_floating_base /*floating base*/, urdf,
                     FLAGS_spring_model, true);
  plant_for_solver.Finalize();
  if (FLAGS_floating_base) {
    CassieFixedPointSolver(plant_for_solver, FLAGS_init_height, mu_fp,
                           min_normal_fp, true, toe_spread, &q_init, &u_init,
                           &lambda_init);
  } else {
    CassieFixedBaseFixedPointSolver(plant_for_solver, &q_init, &u_init,
                                    &lambda_init);
  }
  plant.SetPositions(&plant_context, q_init);
  plant.SetVelocities(&plant_context, VectorXd::Zero(plant.num_velocities()));

  Simulator<double> simulator(*diagram, std::move(diagram_context));

  if (!FLAGS_time_stepping) {
    // simulator.get_mutable_integrator()->set_maximum_step_size(0.01);
    // simulator.get_mutable_integrator()->set_target_accuracy(1e-1);
    // simulator.get_mutable_integrator()->set_fixed_step_mode(true);
    simulator.reset_integrator<drake::systems::RungeKutta2Integrator<double>>(
        FLAGS_dt);
  }

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();
  simulator.AdvanceTo(FLAGS_end_time);

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
