//
// Created by brian on 4/14/21.
//
#include<gflags/gflags.h>

#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/lcmt_drake_signal.hpp"

#include "koopman_mpc.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "common/find_resource.h"
#include "common/file_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/controllers/time_based_fsm.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/dairlib_signal_lcm_systems.h"

namespace dairlib {

using systems::DairlibSignalReceiver;
using systems::RobotOutputReceiver;
using systems::TimeBasedFiniteStateMachine;
using systems::LcmDrivenLoop;
using systems::OutputVector;


using std::cout;
using std::string;

using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::Frame;

using drake::lcm::DrakeLcm;

using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::TriggerTypeSet;
using drake::systems::Demultiplexer;
using drake::systems::ConstantVectorSource;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

DEFINE_string(channel_x, "PLANAR_STATE", "channel to publish/receive planar walker state");
DEFINE_string(channel_plan, "KOOPMAN_MPC_OUT", "channel to publish plan trajectory");
DEFINE_double(stance_time, 0.35, "duration of each stance phase");
DEFINE_bool(debug_mode, false, "Manually set MPC values to debug");

VectorXd poly_basis_1 (const VectorXd& x) {
  return x;
}

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Koopman parameters
  double dt = 0.025;

  DrakeLcm lcm_local;


  // diagram builder
  DiagramBuilder<double> builder;

  // Add MBP
  MultibodyPlant<double> plant(0.0);
  Parser parser(&plant);

  string plant_file_name = FindResourceOrThrow("examples/PlanarWalker/PlanarWalkerWithTorso.urdf");
  parser.AddModelFromFile(plant_file_name);

  plant.WeldFrames(
      plant.world_frame(), plant.GetFrameByName("base"), drake::math::RigidTransform<double>());

  plant.Finalize();

  Eigen::VectorXd x0 = VectorXd::Zero(plant.num_positions() + plant.num_velocities());
  auto map = multibody::makeNameToPositionsMap(plant);
  x0(map["planar_z"]) = 1.0;

  auto plant_context = plant.CreateDefaultContext();

  plant.SetPositionsAndVelocities(plant_context.get(), x0);

  Vector3d default_com = plant.CalcCenterOfMassPositionInWorld(*plant_context.get());

  std::cout << "Com_Position: \n" << default_com << std::endl;

  auto kmpc = builder.AddSystem<KoopmanMPC>(plant, plant_context.get(), dt, true, true, true);

  string folder_base = FindResourceOrThrow("examples/KoopmanMPC/koopman_models/planar_poly_1/");

  MatrixXd Al = MatrixXd::Zero(12, 12);
  MatrixXd Bl = MatrixXd::Zero(12, 6);
  MatrixXd bl = MatrixXd::Zero(12,1);
  MatrixXd Ar = MatrixXd::Zero(12, 12);
  MatrixXd Br = MatrixXd::Zero(12, 6);
  MatrixXd br = MatrixXd::Zero(12,1);

  KoopmanMPC::LoadDiscreteDynamicsFromFolder(folder_base, dt, &Al, &Bl, &bl, &Ar, &Br, &br);

  KoopmanDynamics left_stance_dynamics = {&poly_basis_1, Al, Bl, bl};
  KoopmanDynamics right_stance_dynamics = {&poly_basis_1, Ar, Br, br};

  kmpc->AddMode(left_stance_dynamics, koopMpcStance::kLeft, std::round(FLAGS_stance_time / dt));
  kmpc->AddMode(right_stance_dynamics, koopMpcStance::kRight, std::round(FLAGS_stance_time / dt));

  // add contact points
  auto left_pt = std::pair<const drake::multibody::BodyFrame<double> &, Eigen::Vector3d>(
          plant.GetBodyByName("left_lower_leg").body_frame(), Vector3d(0, 0, -0.5));

  auto right_pt = std::pair<const drake::multibody::BodyFrame<double> &, Eigen::Vector3d>(
      plant.GetBodyByName("right_lower_leg").body_frame(), Vector3d(0, 0, -0.5));

  kmpc->AddContactPoint(left_pt, koopMpcStance::kLeft);
  kmpc->AddContactPoint(right_pt, koopMpcStance::kRight);

  // Set kinematic reachability constraint
  std::vector<VectorXd> kin_nom = {0.9*Vector2d(-default_com(kmpc->saggital_idx()), -default_com(kmpc->vertical_idx())),
                                   0.9*Vector2d(-default_com(kmpc->saggital_idx()), -default_com(kmpc->vertical_idx()))};

  kmpc->SetReachabilityLimit(0.4*VectorXd::Ones(2), kin_nom);

  // add base pivot angle
  kmpc->AddJointToTrackBaseAngle("planar_roty", "planar_rotydot");

  // set mass
  std::vector<string> massive_bodies = {"torso_mass", "left_upper_leg_mass", "right_upper_leg_mass",
                                        "left_lower_leg_mass", "right_lower_leg_mass"};
  double mass = kmpc->SetMassFromListOfBodies(massive_bodies);

  // add tracking objective
  VectorXd x_des = VectorXd::Zero(kmpc->num_state_inflated());
  x_des(1) = 0.85 * default_com(kmpc->vertical_idx());
  x_des(3) = 0.25;
  x_des.tail(1) = 9.81 * mass * VectorXd::Ones(1);

  VectorXd q(kmpc->num_state_inflated());
  q << 0, 2, 2, 10, 1, 1, 0, 0, 0, 0, 0.0001, 0.0001;

  kmpc->AddTrackingObjective(x_des, q.asDiagonal());

    // add input cost
  kmpc->AddInputRegularization(0.0001 * VectorXd::Ones(6).asDiagonal());

  // set friction coeff
  kmpc->SetMu(0.8);
  kmpc->Build();
  std::cout << "Successfully built kmpc" << std::endl;

  auto xdes_source = builder.AddSystem<ConstantVectorSource<double>>(x_des);
  builder.Connect(xdes_source->get_output_port(), kmpc->get_x_des_input_port());

  std::vector<int> fsm_states = {koopMpcStance::kLeft, koopMpcStance::kRight};
  std::vector<double> state_durations = {FLAGS_stance_time, FLAGS_stance_time};

  auto fsm = builder.AddSystem<TimeBasedFiniteStateMachine>(
      plant, fsm_states, state_durations);

  // setup lcm messaging
  auto robot_out = builder.AddSystem<RobotOutputReceiver>(plant);
  auto koopman_mpc_out_publisher = builder.AddSystem(LcmPublisherSystem::Make<lcmt_saved_traj>(FLAGS_channel_plan, &lcm_local));

  builder.Connect(fsm->get_output_port(), kmpc->get_fsm_input_port());

  builder.Connect(robot_out->get_output_port(), fsm->get_input_port_state());

  builder.Connect(robot_out->get_output_port(),
      kmpc->get_state_input_port());

  builder.Connect(kmpc->get_output_port(),
      koopman_mpc_out_publisher->get_input_port());

  auto owned_diagram = builder.Build();
  owned_diagram->set_name("MPC");

  *owned_diagram;
  LcmDrivenLoop<lcmt_robot_output> loop(&lcm_local, std::move(owned_diagram),
      robot_out, FLAGS_channel_x, true);

  if (!FLAGS_debug_mode) {
    loop.Simulate();
  }  else {

  }

  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::DoMain(argc, argv);
}
