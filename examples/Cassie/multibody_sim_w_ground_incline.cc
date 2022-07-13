#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include <drake/systems/primitives/multiplexer.h>
#include <gflags/gflags.h>

#include "dairlib/lcmt_cassie_out.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "examples/Cassie/cassie_utils.h"
#include "multibody/multibody_utils.h"
#include "solvers/constraint_factory.h"
#include "systems/primitives/subvector_pass_through.h"
#include "systems/robot_lcm_systems.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_contact_results_for_viz.hpp"
#include "drake/multibody/plant/contact_results_to_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/runge_kutta2_integrator.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/primitives/discrete_time_delay.h"

namespace dairlib {

using dairlib::systems::SubvectorPassThrough;
using drake::geometry::SceneGraph;
using drake::multibody::ContactResultsToLcmSystem;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiagramBuilder;
using drake::systems::Simulator;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;

using drake::math::RotationMatrix;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using std::pair;
using std::vector;

// Optimal ROM controller
DEFINE_bool(publish_at_initialization, true, "");
DEFINE_double(pause_second, 0, "pause after initialization");

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
DEFINE_bool(spring_model, true, "Use a URDF with or without legs springs");

DEFINE_string(radio_channel, "CASSIE_VIRTUAL_RADIO",
              "LCM channel for virtual radio command");
DEFINE_double(actuator_delay, 0.0,
              "Duration of actuator delay. Set to 0.0 by default.");
DEFINE_bool(publish_efforts, true, "Flag to publish the efforts.");

DEFINE_string(channel_u, "CASSIE_INPUT",
              "The name of the lcm channel that sends Cassie's state");

// Initial condition
DEFINE_double(pelvis_x_vel, 0, "external disturbance for testing");
DEFINE_double(pelvis_y_vel, 0.3, "for stability");

// Terrain
DEFINE_double(ground_incline, 0, "in radians. Positive is walking downhill");

class SimTerminator : public drake::systems::LeafSystem<double> {
 public:
  SimTerminator(const drake::multibody::MultibodyPlant<double>& plant,
                double update_period)
      : plant_(plant),
        context_(plant_.CreateDefaultContext()),
        toes_({LeftToeFront(plant), RightToeFront(plant)}) {
    this->set_name("termination");

    // Input/Output Setup
    this->DeclareVectorInputPort(
        "x",
        BasicVector<double>(plant.num_positions() + plant.num_velocities()));
    DeclarePeriodicDiscreteUpdateEvent(update_period, 0, &SimTerminator::Check);
  };

 private:
  void Check(const drake::systems::Context<double>& context,
             drake::systems::DiscreteValues<double>* discrete_state) const {
    drake::VectorX<double> x = this->EvalVectorInput(context, 0)->get_value();
    /*multibody::SetPositionsIfNew<double>(plant_,
       x.head(plant_.num_positions()), context_.get());*/
    plant_.SetPositions(context_.get(), x.head(plant_.num_positions()));

    drake::VectorX<double> pt_world(3);
    for (int i = 0; i < 2; i++) {
      plant_.CalcPointsPositions(*context_, toes_.at(i).second,
                                 Vector3d::Zero(), plant_.world_frame(),
                                 &pt_world);

      // Pelvis height wrt toe height
      DRAKE_DEMAND(x(6) - pt_world(2) > 0.2);
    }
  };

  const drake::multibody::MultibodyPlant<double>& plant_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  vector<pair<const Vector3d, const drake::multibody::Frame<double>&>> toes_;
};

int do_main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Ground direction
  DRAKE_DEMAND(abs(FLAGS_ground_incline) <= 0.3);
  Vector3d ground_normal(sin(FLAGS_ground_incline), 0,
                         cos(FLAGS_ground_incline));

  // Plant/System initialization
  DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  const double time_step = FLAGS_time_stepping ? FLAGS_dt : 0.0;
  drake::multibody::MultibodyPlant<double>& plant =
      *builder.AddSystem<drake::multibody::MultibodyPlant>(time_step);
  if (FLAGS_floating_base) {
    multibody::AddFlatTerrain(&plant, &scene_graph, .8, .8, ground_normal);
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
  auto input_sub =
      builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, lcm));
  auto input_receiver = builder.AddSystem<systems::RobotInputReceiver>(plant);
  auto passthrough = builder.AddSystem<SubvectorPassThrough>(
      input_receiver->get_output_port(0).size(), 0,
      plant.get_actuation_input_port().size());
  auto discrete_time_delay =
      builder.AddSystem<drake::systems::DiscreteTimeDelay>(
          1.0 / FLAGS_publish_rate, FLAGS_actuator_delay * FLAGS_publish_rate,
          plant.num_actuators());
  auto state_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_output>(
          "CASSIE_STATE_SIMULATION", lcm, 1.0 / FLAGS_publish_rate));
  auto state_sender = builder.AddSystem<systems::RobotOutputSender>(
      plant, FLAGS_publish_efforts);

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

  // Termination checker
  auto terminator =
      builder.AddSystem<SimTerminator>(plant, 1.0 / FLAGS_publish_rate);
  builder.Connect(plant.get_state_output_port(), terminator->get_input_port(0));

  // connect leaf systems
  builder.Connect(*input_sub, *input_receiver);
  builder.Connect(*input_receiver, *passthrough);
  builder.Connect(passthrough->get_output_port(),
                  discrete_time_delay->get_input_port());
  builder.Connect(discrete_time_delay->get_output_port(),
                  plant.get_actuation_input_port());
  builder.Connect(plant.get_state_output_port(),
                  state_sender->get_input_port_state());
  builder.Connect(discrete_time_delay->get_output_port(),
                  state_sender->get_input_port_effort());
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
  builder.Connect(radio_sub->get_output_port(),
                  sensor_aggregator.get_input_port_radio());
  builder.Connect(sensor_aggregator.get_output_port(0),
                  sensor_pub->get_input_port());

  auto diagram = builder.Build();

  // Create a context for this system:
  std::unique_ptr<Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram_context->EnableCaching();
  diagram->SetDefaultContext(diagram_context.get());
  Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());

  // Set initial conditions of the simulation
  VectorXd q_init, v_init, u_init, lambda_init;
  v_init = VectorXd::Zero(plant.num_velocities());
  double mu_fp = 0.5;         // 0
  double min_normal_fp = 10;  // 70
  double toe_spread = .15;
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
    VectorXd all_sol;
    CassieFixedPointSolver(plant_for_solver, FLAGS_init_height, mu_fp,
                           min_normal_fp, true, toe_spread, &q_init, &u_init,
                           &lambda_init, "", 0, &all_sol);
    CassieFixedPointSolver(plant_for_solver, FLAGS_init_height, mu_fp,
                           min_normal_fp, true, toe_spread, &q_init, &u_init,
                           &lambda_init, "", FLAGS_ground_incline, &all_sol);
    // std::cout << "q_init = \n" << q_init.transpose() << std::endl;
    // std::cout << "v_init = \n" << v_init.transpose() << std::endl;

    VectorXd pelvis_xy_vel(2);
    pelvis_xy_vel << FLAGS_pelvis_x_vel, FLAGS_pelvis_y_vel;
    CassieInitStateSolver(plant_for_solver, pelvis_xy_vel, FLAGS_init_height,
                          mu_fp, min_normal_fp, true, toe_spread,
                          FLAGS_ground_incline, q_init, u_init, lambda_init,
                          &q_init, &v_init, &u_init, &lambda_init);
    std::cout << "q_init = \n" << q_init.transpose() << std::endl;
    std::cout << "v_init = \n" << v_init.transpose() << std::endl;
  } else {
    CassieFixedBaseFixedPointSolver(plant_for_solver, &q_init, &u_init,
                                    &lambda_init);
    v_init = VectorXd::Zero(plant.num_velocities());
  }
  double theta = 0.0 / 180.0 * M_PI;
  q_init.head<4>() << cos(theta / 2), 0, 0, sin(theta / 2);
  plant.SetPositions(&plant_context, q_init);
  plant.SetVelocities(&plant_context, v_init);
  //  std::cout << "q_init = \n" << q_init.transpose() << std::endl;
  //  std::cout << "v_init = \n" << v_init.transpose() << std::endl;

  Simulator<double> simulator(*diagram, std::move(diagram_context));

  // Set the current time for testing
  //  auto& sim_diagram_context = simulator.get_mutable_context();
  //  sim_diagram_context.SetTime(13.1415926);

  if (!FLAGS_time_stepping) {
    // simulator.get_mutable_integrator()->set_maximum_step_size(0.01);
    // simulator.get_mutable_integrator()->set_target_accuracy(1e-1);
    // simulator.get_mutable_integrator()->set_fixed_step_mode(true);
    simulator.reset_integrator<drake::systems::RungeKutta2Integrator<double>>(
        FLAGS_dt);
  }

  simulator.set_publish_every_time_step(false);
  simulator.set_publish_at_initialization(FLAGS_publish_at_initialization);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();

  // pause a second for the planner to plan
  std::this_thread::sleep_for(
      std::chrono::milliseconds(int(FLAGS_pause_second * 1000)));

  simulator.AdvanceTo(FLAGS_end_time);

  return 0;
}

}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::do_main(argc, argv); }
