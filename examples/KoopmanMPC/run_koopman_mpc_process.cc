//
// Created by brian on 4/14/21.
//
#include<gflags/gflags.h>

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/lcmt_drake_signal.hpp"

#include "koopman_mpc_gains.h"
#include "model_utils.h"
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
using systems::DrakeSignalSender;
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

DEFINE_string(gains_filename, "examples/KoopmanMPC/koopman_mpc_gains.yaml", "koopman mpc gains file");
DEFINE_string(channel_x, "PLANAR_STATE", "channel to publish/receive planar walker state");
DEFINE_string(channel_plan, "KOOPMAN_MPC_OUT", "channel to publish plan trajectory");
DEFINE_string(channel_fsm, "FSM", "the name of the channel with the time-based fsm");
DEFINE_double(stance_time, 0.35, "duration of each stance phase");
DEFINE_bool(debug_mode, false, "Manually set MPC values to debug");
DEFINE_bool(use_com, false, "Use center of mass or a point to track CM location");
DEFINE_double(debug_time, 0.00, "time to simulate system at");
DEFINE_double(swing_ft_height, 0.05, "Swing foot height");
DEFINE_double(v_des, 0.4, "desired walking speed");
DEFINE_double(h_des, 1.1, "Desired CoM height");
DEFINE_double(dt, 0.01, "time step for koopman mpc");

VectorXd poly_basis_1 (const VectorXd& x) {
  return x;
}

int DoMain(int argc, char* argv[]) {

  MpcGains gains;
  const YAML::Node& root =
      YAML::LoadFile(FindResourceOrThrow(FLAGS_gains_filename));
  drake::yaml::YamlReadArchive(root).Accept(&gains);


  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Koopman parameters
  double dt = FLAGS_dt;

  DrakeLcm lcm_local;


  // diagram builder
  DiagramBuilder<double> builder;

  // Add MBP
  MultibodyPlant<double> plant(0.0);
  LoadPlanarWalkerFromFile(plant);
  plant.Finalize();

  Eigen::VectorXd x0 = VectorXd::Zero(plant.num_positions() + plant.num_velocities());
  auto map = multibody::makeNameToPositionsMap(plant);
  x0(map["planar_z"]) = 1.0;

  auto plant_context = plant.CreateDefaultContext();

  plant.SetPositionsAndVelocities(plant_context.get(), x0);


  Vector3d default_com = (FLAGS_use_com) ?
      plant.CalcCenterOfMassPositionInWorld(*plant_context.get()) :
      plant.GetBodyByName("torso_mass").EvalPoseInWorld(*plant_context.get()).translation();



  std::cout << "Com_Position: \n" << default_com << std::endl;

  auto kmpc = builder.AddSystem<KoopmanMPC>(plant, plant_context.get(), dt,
      FLAGS_swing_ft_height, true, true,  FLAGS_use_com);

  if (!FLAGS_use_com) {
    kmpc->AddBaseFrame("torso_mass", Vector3d::Zero());
  }
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
  default_com(kmpc->vertical_idx()) = FLAGS_h_des;
  std::vector<VectorXd> kin_nom = {
      Vector2d(default_com(kmpc->saggital_idx()), default_com(kmpc->vertical_idx())),
      Vector2d(default_com(kmpc->saggital_idx()), default_com(kmpc->vertical_idx()))};

  Vector2d kin_weights(1,2);
  kmpc->SetReachabilityLimit(
      gains.ReachabilityLim*kin_weights, kin_nom, gains.W_kin_reach);

  // add base pivot angle
  kmpc->AddJointToTrackBaseAngle("planar_roty", "planar_rotydot");

  // set mass
  std::vector<string> massive_bodies = {"torso_mass"};
  double mass = kmpc->SetMassFromListOfBodies(massive_bodies);

  // add tracking objective
  VectorXd x_des = VectorXd::Zero(kmpc->num_state_inflated());
  x_des(1) = FLAGS_h_des;
  x_des(3) = FLAGS_v_des;
  x_des.tail(1) = 9.81 * mass * VectorXd::Ones(1);

  std::cout << "x desired:\n" << x_des <<std::endl;

  kmpc->AddTrackingObjective(x_des, gains.q.asDiagonal());

  kmpc->SetFlatGroundSoftConstraint(gains.W_flat_ground);

    // add input cost
  kmpc->AddInputRegularization(gains.r.asDiagonal());

  // set friction coeff
  kmpc->SetMu(gains.mu);
  kmpc->Build();
  std::cout << "Successfully built kmpc" << std::endl;

  auto xdes_source = builder.AddSystem<ConstantVectorSource<double>>(x_des);
  builder.Connect(xdes_source->get_output_port(), kmpc->get_x_des_input_port());

  std::vector<int> fsm_states = {koopMpcStance::kLeft, koopMpcStance::kRight};
  std::vector<double> state_durations = {FLAGS_stance_time, FLAGS_stance_time};

  auto fsm = builder.AddSystem<TimeBasedFiniteStateMachine>(
      plant, fsm_states, state_durations);

  std::vector<std::string> signals = {"fsm"};
  auto fsm_send = builder.AddSystem<DrakeSignalSender>(signals, FLAGS_stance_time * 2);
  auto fsm_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_dairlib_signal>(FLAGS_channel_fsm, &lcm_local));


  // setup lcm messaging
  auto robot_out = builder.AddSystem<RobotOutputReceiver>(plant);
  auto koopman_mpc_out_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_saved_traj>(FLAGS_channel_plan, &lcm_local));

  // fsm connections
  builder.Connect(fsm->get_output_port(), kmpc->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(), fsm_send->get_input_port());
  builder.Connect(fsm_send->get_output_port(), fsm_pub->get_input_port());

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
    OutputVector<double> robot_out(x0.head(plant.num_positions()),
                                   x0.tail(plant.num_velocities()),
                                   VectorXd::Zero(plant.num_actuators()));

    robot_out.set_timestamp(FLAGS_debug_time);
    auto diagram_ptr = loop.get_diagram();
    auto& diagram_context = loop.get_diagram_mutable_context();

    diagram_context.SetTime(FLAGS_debug_time);

    auto& kmpc_context = diagram_ptr->GetMutableSubsystemContext(*kmpc, &diagram_context);

    kmpc->get_x_des_input_port().FixValue(&kmpc_context, x_des);
    kmpc->get_fsm_input_port().FixValue(&kmpc_context, VectorXd::Zero(1));
    kmpc->get_state_input_port().FixValue(&kmpc_context, robot_out);

    auto out = kmpc->AllocateOutput();
    kmpc->CalcOutput(kmpc_context, out.get());

    kmpc->print_initial_state_constraints();
    kmpc->print_dynamics_constraints();
    kmpc->print_state_knot_constraints();
  }

  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::DoMain(argc, argv);
}
