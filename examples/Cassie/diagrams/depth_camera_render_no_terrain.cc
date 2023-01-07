//
// Created by brian on 1/6/23.
//

#include "depth_camera_render_no_terrain.h"
#include "cassie_vision_sim_diagram.h"

#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/zero_order_hold.h>

#include <utility>


#include "dairlib/lcmt_robot_input.hpp"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "multibody/boxy_height_map.h"
#include "multibody/cube_height_map.h"
#include "multibody/stairs.h"
#include "systems/framework/geared_motor.h"
#include "systems/primitives/radio_parser.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/cameras/camera_utils.h"

#include "drake/geometry/drake_visualizer.h"
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
#include "drake/systems/sensors/rgbd_sensor.h"
#include "drake/geometry/render_vtk/render_engine_vtk_params.h"
#include "drake/geometry/render_vtk/factory.h"

namespace dairlib {
namespace examples {

using dairlib::systems::SubvectorPassThrough;
using drake::geometry::DrakeVisualizer;
using drake::geometry::SceneGraph;
using drake::math::RotationMatrix;
using drake::math::RigidTransform;
using drake::multibody::ContactResultsToLcmSystem;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

DepthCameraRenderNoTerrain::DepthCameraRenderNoTerrain(
    std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant,
    drake::math::RigidTransform<double>  camera_pose,
    const std::string& urdf) : cam_transform_(std::move(camera_pose)) {

  DiagramBuilder<double> builder;
  scene_graph_ = builder.AddSystem<SceneGraph>();
  scene_graph_->set_name("scene_graph");

  const std::string renderer_name = "multibody_renderer_vtk";
  scene_graph_->AddRenderer(renderer_name,
                            drake::geometry::MakeRenderEngineVtk(
                                drake::geometry::RenderEngineVtkParams()));

  plant_ = builder.AddSystem(std::move(plant));
  AddCassieMultibody(plant_, scene_graph_, true, urdf, true, true);
  plant_->Finalize();

  // Add camera model
  const auto& [color_camera, depth_camera] =
      camera::MakeDairD455CameraModel(renderer_name,
                                      camera::D455ImageSize::k640x480);
  const std::optional<drake::geometry::FrameId> parent_body_id =
      plant_->GetBodyFrameIdIfExists(
          plant_->GetFrameByName("pelvis").body().index());


  auto camera = builder.AddSystem<drake::systems::sensors::RgbdSensor>(
      parent_body_id.value(), cam_transform_, color_camera, depth_camera);


  // connect leaf systems
  builder.Connect(
      plant_->get_geometry_poses_output_port(),
      scene_graph_->get_source_pose_port(plant_->get_source_id().value()));
  builder.Connect(scene_graph_->get_query_output_port(),
                  plant_->get_geometry_query_input_port());
  builder.Connect(scene_graph_->get_query_output_port(),
                  camera->query_object_input_port());
  builder.ExportInput()
  builder.ExportOutput(camera->depth_image_32F_output_port(), "camera_output");
  builder.ExportOutput(camera->body_pose_in_world_output_port(), "camera pose");

  if (visualize) {
    DrakeVisualizer<double>::AddToBuilder(&builder, *scene_graph_);
  }
  builder.BuildInto(this);
  this->set_name("cassie_sim_diagram");
//  DrawAndSaveDiagramGraph(*this);
}
}  // namespace examples
}  // namespace dairlib