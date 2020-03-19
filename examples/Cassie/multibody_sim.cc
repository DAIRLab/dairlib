#include <memory>

#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <gflags/gflags.h>
#include "drake/geometry/geometry_visualization.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/constant_vector_source.h"

#include "drake/systems/primitives/signal_logger.h"

#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/plant/contact_results_to_lcm.h"

namespace dairlib {
using dairlib::systems::SubvectorPassThrough;
using drake::geometry::SceneGraph;
using drake::multibody::ContactResultsToLcmSystem;
using drake::multibody::MultibodyPlant;
using drake::multibody::RevoluteJoint;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using drake::math::RotationMatrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

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
              "The step size to use for compliant, ignored for time_stepping)");
DEFINE_double(penetration_allowance, 1e-5,
              "Penetration allowance for the contact model. Nearly equivalent"
              " to (m)");
DEFINE_double(end_time, std::numeric_limits<double>::infinity(),
              "End time for simulator");
DEFINE_double(publish_rate, 1000, "Publish rate for simulator");
DEFINE_double(init_height, 1.1,
              "Initial starting height of the pelvis above "
              "ground");

Eigen::VectorXd GetInitialState(const MultibodyPlant<double>& plant);

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Plant/System initialization
  DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double time_step = FLAGS_time_stepping ? FLAGS_dt : 0.0;
  MultibodyPlant<double>& plant = *builder.AddSystem<MultibodyPlant>(time_step);
  if (FLAGS_floating_base) {
    multibody::addFlatTerrain(&plant, &scene_graph, .8, .8);
  }
  addCassieMultibody(&plant, &scene_graph, FLAGS_floating_base,
                     "examples/Cassie/urdf/cassie_v2.urdf");

  plant.Finalize();
  plant.set_penetration_allowance(FLAGS_penetration_allowance);


  // Create lcm systems.
  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
  auto input_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          "CASSIE_INPUT", lcm));
  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(plant);
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(), 0,
      plant.get_actuation_input_port().size());
  auto state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          "CASSIE_STATE", lcm, 1.0 / FLAGS_publish_rate));
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(plant);

  // Contact Information
  ContactResultsToLcmSystem<double>& contact_viz =
      *builder.template AddSystem<ContactResultsToLcmSystem<double>>(plant);
  contact_viz.set_name("contact_visualization");
  auto& contact_results_publisher = *builder.AddSystem(
      LcmPublisherSystem::Make<drake::lcmt_contact_results_for_viz>(
          "CASSIE_CONTACT_RESULTS", lcm, 1.0 / FLAGS_publish_rate));
  contact_results_publisher.set_name("contact_results_publisher");

  // connect leaf systems
  builder.Connect(*input_sub, *input_receiver);
  builder.Connect(*input_receiver, *passthrough);
  builder.Connect(passthrough->get_output_port(),
                  plant.get_actuation_input_port());
  builder.Connect(plant.get_state_output_port(),
                  state_sender->get_input_port_state());
  builder.Connect(*state_sender, *state_pub);
  builder.Connect(
      plant.get_geometry_poses_output_port(),
      scene_graph.get_source_pose_port(plant.get_source_id().value()));
  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());
  builder.Connect(plant.get_contact_results_output_port(),
                  contact_viz.get_input_port(0));
  builder.Connect(contact_viz.get_output_port(0),
                  contact_results_publisher.get_input_port());

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set initial conditions of the simulation
  VectorXd q_init = GetInitialState(plant);
  plant.SetPositions(&plant_context, q_init);
  plant.SetVelocities(&plant_context, VectorXd::Zero(plant.num_velocities()));

  if (FLAGS_floating_base) {
    const drake::math::RigidTransformd transform(
        RotationMatrix<double>(), Eigen::Vector3d(0, 0, FLAGS_init_height));
    plant.SetFreeBodyPose(&plant_context, plant.GetBodyByName("pelvis"),
                          transform);
  }

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


Eigen::VectorXd GetInitialState(const MultibodyPlant<double>& plant) {
  int n_q = plant.num_positions();
  std::map<std::string, int> positions_map =
      multibody::makeNameToPositionsMap(plant);

  VectorXd q_ik_guess = VectorXd::Zero(n_q);
  Eigen::Vector4d quat(1, 0, 0, 0);
  q_ik_guess << quat.normalized(), 0.001, 0.001, 1.1, -0.01, 0.01, 0.0, 0.0,
      1.15, 1.15, -1.35, -1.35, 1.0, 1.0, 0.0, 0.0, 0.0, -M_PI / 2, 0.0,
      -M_PI / 2;

  double achilles_length = .5012;
  double feet_xpos_offset = -0.15;
  double eps = 1e-4;
  Vector3d pelvis_pos(0.0, 0.0, 1.0);
  Vector3d rear_contact_disp(-0.0457, 0.112, 0);
  Vector3d front_contact_disp(0.088, 0, 0);
  Vector3d left_toe_rear_pos(-0.02115 + feet_xpos_offset, 0.12, 0.00);
  Vector3d left_toe_front_pos(0.02115 + feet_xpos_offset, 0.12, 0.00);
  Vector3d right_toe_rear_pos(-0.02115 + feet_xpos_offset, -0.12, 0.00);
  Vector3d right_toe_front_pos(0.02115 + feet_xpos_offset, -0.12, 0.00);

  Vector3d rod_on_heel_spring;  // symmetric left and right
  rod_on_heel_spring << .11877, -.01, 0.0;
  Vector3d rod_on_thigh_left;
  rod_on_thigh_left << 0.0, 0.0, 0.045;
  Vector3d rod_on_thigh_right;
  rod_on_thigh_right << 0.0, 0.0, -0.045;

  const auto& world_frame = plant.world_frame();
  const auto& pelvis_frame = plant.GetFrameByName("pelvis");
  const auto& toe_left_frame = plant.GetFrameByName("toe_left");
  const auto& toe_right_frame = plant.GetFrameByName("toe_right");
  const auto& thigh_left_frame = plant.GetFrameByName("thigh_left");
  const auto& thigh_right_frame = plant.GetFrameByName("thigh_right");
  const auto& heel_spring_left_frame = plant.GetFrameByName("heel_spring_left");
  const auto& heel_spring_right_frame =
      plant.GetFrameByName("heel_spring_right");

  drake::multibody::InverseKinematics ik(plant);

  ik.AddPositionConstraint(pelvis_frame, Vector3d(0, 0, 0), world_frame,
                           pelvis_pos - eps * VectorXd::Ones(3),
                           pelvis_pos + eps * VectorXd::Ones(3));
  ik.AddOrientationConstraint(pelvis_frame, RotationMatrix<double>(),
                              world_frame, RotationMatrix<double>(), eps);
  ik.AddPositionConstraint(toe_left_frame, rear_contact_disp, world_frame,
                           left_toe_rear_pos, left_toe_rear_pos);
  ik.AddPositionConstraint(toe_left_frame, front_contact_disp, world_frame,
                           left_toe_front_pos, left_toe_front_pos);
  ik.AddPositionConstraint(toe_right_frame, rear_contact_disp, world_frame,
                           right_toe_rear_pos, right_toe_rear_pos);
  ik.AddPositionConstraint(toe_right_frame, front_contact_disp, world_frame,
                           right_toe_front_pos, right_toe_front_pos);
  ik.AddPointToPointDistanceConstraint(
      heel_spring_left_frame, rod_on_heel_spring, thigh_left_frame,
      rod_on_thigh_left, achilles_length, achilles_length);
  ik.AddPointToPointDistanceConstraint(
      heel_spring_right_frame, rod_on_heel_spring, thigh_right_frame,
      rod_on_thigh_right, achilles_length, achilles_length);

  ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_ik_guess);
  const auto result = Solve(ik.prog());
  const auto q_sol = result.GetSolution(ik.q());

  VectorXd q_sol_normd(n_q);
  q_sol_normd << q_sol.head(4).normalized(), q_sol.tail(n_q - 4);
  return q_sol_normd;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
