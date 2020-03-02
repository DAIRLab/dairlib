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
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

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


using drake::math::RotationMatrix;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;

// Simulation parameters.
DEFINE_bool(floating_base, true, "Fixed or floating base model");

DEFINE_double(target_realtime_rate, 1.0,
    "Desired rate relative to real time.  See documentation for "
    "Simulator::set_target_realtime_rate() for details.");
DEFINE_bool(time_stepping, false,
    "If 'true', the plant is modeled as a "
    "discrete system with periodic updates. "
    "If 'false', the plant is modeled as a continuous system.");
DEFINE_double(dt, 1e-4,
    "The step size to use for compliant, ignored for time_stepping)");
DEFINE_double(penetration_allowance, 1e-4,
    "Penetration allowance for the contact model. Nearly equivalent"
    " to (m)");
DEFINE_double(end_time, std::numeric_limits<double>::infinity(),
    "End time for simulator");
DEFINE_double(publish_rate, 1000, "Publish rate for simulator");
DEFINE_double(init_height, 0.2, "Initial starting height above ground");


Eigen::VectorXd GetInitialState(const MultibodyPlant<double>& plant,
                                double init_height);


int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm;

  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double time_step = FLAGS_time_stepping ? FLAGS_dt : 0.0;

  MultibodyPlant<double>& plant =
      *builder.AddSystem<MultibodyPlant>(time_step);

  if (FLAGS_floating_base) {
    multibody::addFlatTerrain(&plant, &scene_graph, .8, .8);
  }

  addCassieMultibody(&plant, &scene_graph, FLAGS_floating_base);

  plant.Finalize();

  plant.set_penetration_allowance(FLAGS_penetration_allowance);
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
  builder.Connect(plant.get_state_output_port(),
                  state_sender->get_input_port_state());

  builder.Connect(*state_sender, *state_pub);

  builder.Connect(
    plant.get_geometry_poses_output_port(),
    scene_graph.get_source_pose_port(plant.get_source_id().value()));

  builder.Connect(scene_graph.get_query_output_port(),
                  plant.get_geometry_query_input_port());

  auto diagram = builder.Build();


  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  VectorXd q_init = GetInitialState(plant, FLAGS_init_height);
  std::cout << q_init << std::endl;

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
    const drake::math::RigidTransformd transform(
        drake::math::RotationMatrix<double>(), Eigen::Vector3d(0, 0, 1.2));
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
  simulator.AdvanceTo(std::numeric_limits<double>::infinity());

  return 0;
}

Eigen::VectorXd GetInitialState(const MultibodyPlant<double>& plant,
                                double init_height) {
  int n_q = plant.num_positions();
  std::map<std::string, int> positions_map =
      multibody::makeNameToPositionsMap(plant);

  for (auto pair : positions_map) {
    std::cout << "name: " << pair.first << ": " << pair.second << std::endl;
  }

  VectorXd q_init_guess;
  VectorXd q_ik_guess = VectorXd::Zero(plant.num_positions());
  Eigen::Vector4d quat(2000.06, -0.339462, -0.609533, -0.760854);
  q_ik_guess << quat.normalized(), 0.000889849, 0.000626865, 1.0009, -0.0112109,
      0.00927845, -0.000600725, -0.000895805, 1.15086, 0.610808, -1.38608,
      -1.35926, 0.806192, 1.00716, -M_PI / 2, -M_PI / 2;

  double eps = 1e-3;
  Vector3d eps_vec = eps * VectorXd::Ones(3);
  Vector3d pelvis_pos(0.0, 0.0, 1.0 + init_height);
  Vector3d left_toe_pos(0.0, 0.12, 0.05 + init_height);
  Vector3d right_toe_pos(0.0, -0.12, 0.05 + init_height);

  const auto& world_frame = plant.world_frame();
  const auto& pelvis_frame = plant.GetFrameByName("pelvis");
  const auto& toe_left_frame = plant.GetFrameByName("toe_left");
  const auto& toe_right_frame = plant.GetFrameByName("toe_right");

  drake::multibody::InverseKinematics ik(plant);
  ik.AddPositionConstraint(pelvis_frame, Vector3d(0, 0, 0), world_frame,
      pelvis_pos - eps * VectorXd::Ones(3),
      pelvis_pos + eps * VectorXd::Ones(3));
  ik.AddOrientationConstraint(pelvis_frame, RotationMatrix<double>(),
      world_frame, RotationMatrix<double>(), eps);
  ik.AddPositionConstraint(toe_left_frame, Vector3d(0, 0, 0), world_frame,
      left_toe_pos - eps_vec, left_toe_pos + eps_vec);
  ik.AddPositionConstraint(toe_right_frame, Vector3d(0, 0, 0), world_frame,
      right_toe_pos - eps_vec, right_toe_pos + eps_vec);
  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("hip_yaw_left")) == 0);
  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("hip_yaw_right")) == 0);
  // Four bar linkage constraint (without spring)
  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("knee_left")) +
          (ik.q())(positions_map.at("ankle_joint_left")) ==
          M_PI * 13 / 180.0);
  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("knee_right")) +
          (ik.q())(positions_map.at("ankle_joint_right")) ==
          M_PI * 13 / 180.0);
  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("toe_left")) == -1.0);
  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("toe_right")) == -1.0);

  ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_ik_guess);
  const auto result = Solve(ik.prog());
  const auto q_sol = result.GetSolution(ik.q());
  // cout << "  q_sol = " << q_sol.transpose() << endl;
  // cout << "  q_sol.head(4).norm() = " << q_sol.head(4).norm() << endl;
  VectorXd q_sol_normd(n_q);
  q_sol_normd << q_sol.head(4).normalized(), q_sol.tail(n_q - 4);
  q_init_guess = q_sol_normd;
  //  return q_ik_guess;
  return q_init_guess;
}

}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
