//
// Created by brian on 4/14/21.
//
#include <gflags/gflags.h>

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"

#include "dairlib/lcmt_robot_output.hpp"
#include "common/find_resource.h"
#include "common/file_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/controllers/time_based_fsm.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/dairlib_signal_lcm_systems.h"
#include "systems/system_utils.h"
#include "systems/controllers/mpc/srbd_cmpc.h"
#include "multibody/single_rigid_body_plant.h"
#include "examples/Cassie/mpc/cassie_srbd_cmpc_gains.h"
#include "examples/Cassie/cassie_utils.h"


namespace dairlib {

using systems::DairlibSignalReceiver;
using systems::DrakeSignalSender;
using systems::RobotOutputReceiver;
using systems::TimeBasedFiniteStateMachine;
using systems::LcmDrivenLoop;
using systems::OutputVector;
using multibody::SingleRigidBodyPlant;


using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::multibody::Frame;
using drake::lcm::DrakeLcm;

using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::TriggerTypeSet;
using drake::systems::ConstantVectorSource;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

DEFINE_string(gains_filename, "examples/Cassie/mpc/cassie_srbd_cmpc_gains.yaml", "convex mpc gains file");
DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION", "channel to publish/receive cassie state");
DEFINE_string(channel_plan, "SRBD_MPC_OUT", "channel to publish plan trajectory");
DEFINE_string(channel_fsm, "FSM", "the name of the channel with the time-based fsm");
DEFINE_double(stance_time, 0.3, "duration of each stance phase");
DEFINE_bool(debug_mode, false, "Manually set MPC values to debug");
DEFINE_bool(use_com, false, "Use center of mass or a point to track CM location");
DEFINE_bool(print_diagram, false, "print block diagram");
DEFINE_double(debug_time, 0.00, "time to simulate system at");
DEFINE_double(swing_ft_height, 0.01, "Swing foot height");
DEFINE_double(stance_width, 0.0, "stance width to use in dynamics linearization");
DEFINE_double(v_des, 0.4, "desired walking speed");
DEFINE_double(h_des, 0.75, "Desired pelvis height");
DEFINE_double(dt, 0.01, "time step for koopman mpc");

int DoMain(int argc, char* argv[]) {
  std::cout << "check 0" << std::endl;
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  SrbdMpcGains gains;
  const YAML::Node& root =
      YAML::LoadFile(FindResourceOrThrow(FLAGS_gains_filename));
  drake::yaml::YamlReadArchive(root).Accept(&gains);

  // mpc parameters
  double dt = FLAGS_dt;

  DrakeLcm lcm_local;
  DiagramBuilder<double> builder;

  // Add MBP
  MultibodyPlant<double> plant(0.0);
  addCassieMultibody(&plant, nullptr, true,
      "examples/Cassie/urdf/cassie_v2.urdf", true, false);
  plant.Finalize();
  auto plant_context = plant.CreateDefaultContext();
  auto x0 = plant.GetPositionsAndVelocities(*plant_context);
  auto srb_plant = SingleRigidBodyPlant(plant, plant_context.get(), false);

  std::cout << "Check 1" << std::endl;

  // Cassie SRBD model setup
  Vector3d com_offset = {0, 0, -0.128};
  Vector3d des_pelvis_pos = {0, 0, FLAGS_h_des};
  Vector3d des_com_pos = des_pelvis_pos + com_offset;

  Vector3d left_neutral_foot_pos = {0,  FLAGS_stance_width, 0};
  Vector3d left_safe_nominal_foot_pos = {0, 0.125, 0};
  Vector3d right_neutral_foot_pos = -left_neutral_foot_pos;
  Vector3d right_safe_nominal_foot_pos = -left_safe_nominal_foot_pos;
  Matrix3d I_rot;
  I_rot << 0.91, 0.04, 0.09, 0.04, 0.55, -0.001, 0.08, -0.001, 0.89;

  double mass = 30.0218;
  srb_plant.AddBaseFrame("pelvis", com_offset);

  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  Vector3d mid_contact_point = (left_toe.first + left_heel.first) / 2.0;

  // add contact points
  std::cout << "here1" << std::endl;
  auto left_pt = std::pair<Vector3d, const drake::multibody::BodyFrame<double> &>(
      mid_contact_point, plant.GetBodyByName("toe_left").body_frame());
  std::cout << "here2" << std::endl;
  auto right_pt = std::pair<Vector3d, const drake::multibody::BodyFrame<double> &>(
      mid_contact_point, plant.GetBodyByName("toe_right").body_frame());
  std::cout << "here3" << std::endl;
  srb_plant.SetMass(mass);
  std::cout << "before add_contact_point" << std::endl;
  srb_plant.AddContactPoint(left_pt, BipedStance::kLeft);
  srb_plant.AddContactPoint(right_pt, BipedStance::kRight);
  std::cout << "After add_contact_point" << std::endl;

  int nx = 12;
  int nu = 4;
  MatrixXd Al = MatrixXd::Zero(nx, nx+3);
  MatrixXd Bl = MatrixXd::Zero(nx, nu);
  VectorXd bl = VectorXd::Zero(nx);
  MatrixXd Ar = MatrixXd::Zero(nx, nx+3);
  MatrixXd Br = MatrixXd::Zero(nx, nu);
  VectorXd br = VectorXd::Zero(nx);

  srb_plant.CopyDiscreteLinearizedSrbDynamicsForMPC(
      dt, mass, 0, BipedStance::kLeft,
      I_rot, des_com_pos, left_neutral_foot_pos, &Al, &Bl, &bl);
  srb_plant.CopyDiscreteLinearizedSrbDynamicsForMPC(
      dt, mass, 0, BipedStance::kRight,
      I_rot, des_com_pos, right_neutral_foot_pos, &Ar, &Br, &br);

  LinearSrbdDynamics left_stance_dynamics = {Al, Bl, bl};
  LinearSrbdDynamics right_stance_dynamics = {Ar, Br, br};

  auto cmpc = builder.AddSystem<SrbdCMPC>(srb_plant, dt,
                                          FLAGS_swing_ft_height,
                                          false, true,  FLAGS_use_com);
  std::vector<VectorXd> kin_nom =
      {left_safe_nominal_foot_pos - des_com_pos,
       right_safe_nominal_foot_pos - des_com_pos};
  cmpc->SetReachabilityBoundingBox(gains.kin_reachability_lim,
                                   kin_nom);

  cmpc->AddMode(left_stance_dynamics, BipedStance::kLeft,
      MatrixXd::Identity(nx, nx), std::round(FLAGS_stance_time / dt));
  cmpc->AddMode(right_stance_dynamics, BipedStance::kRight,
      MatrixXd::Identity(nx, nx), std::round(FLAGS_stance_time / dt));
  cmpc->FinalizeModeSequence();

  // add tracking objective
  VectorXd x_des = VectorXd::Zero(nx);
  x_des(2) = des_com_pos(2);
  x_des(3) = FLAGS_v_des;
  MatrixXd qq = gains.q.asDiagonal();

  cmpc->AddTrackingObjective(x_des, gains.q.asDiagonal());
  cmpc->SetTerminalCost(gains.qf.asDiagonal());
  cmpc->AddInputRegularization(gains.r.asDiagonal());

  // set friction coeff
  cmpc->SetMu(gains.mu);
  cmpc->Build();

  std::cout << "Check 3" << std::endl;
  // Wire everything up
  auto xdes_source = builder.AddSystem<ConstantVectorSource<double>>(x_des);
  builder.Connect(xdes_source->get_output_port(), cmpc->get_x_des_input_port());

  std::vector<int> fsm_states = {BipedStance::kLeft, BipedStance::kRight};
  std::vector<double> state_durations = {FLAGS_stance_time, FLAGS_stance_time};

  auto fsm = builder.AddSystem<TimeBasedFiniteStateMachine>(
      plant, fsm_states, state_durations);

  std::vector<std::string> signals = {"fsm"};
  auto fsm_send = builder.AddSystem<DrakeSignalSender>(signals, FLAGS_stance_time * 2);
  auto fsm_pub = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_dairlib_signal>(FLAGS_channel_fsm, &lcm_local));


  // setup lcm messaging
  auto robot_out = builder.AddSystem<RobotOutputReceiver>(plant);
  auto mpc_out_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_saved_traj>(FLAGS_channel_plan, &lcm_local));

  // fsm connections
  builder.Connect(fsm->get_output_port(), cmpc->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(), fsm_send->get_input_port());
  builder.Connect(fsm_send->get_output_port(), fsm_pub->get_input_port());

  builder.Connect(robot_out->get_output_port(), fsm->get_input_port_state());
  builder.Connect(robot_out->get_output_port(),
                  cmpc->get_state_input_port());
  builder.Connect(cmpc->get_output_port(), mpc_out_publisher->get_input_port());

  auto owned_diagram = builder.Build();
  owned_diagram->set_name("MPC");

  *owned_diagram;
  LcmDrivenLoop<lcmt_robot_output> loop(&lcm_local, std::move(owned_diagram),
                                        robot_out, FLAGS_channel_x, true);

  if (FLAGS_print_diagram) {
    DrawAndSaveDiagramGraph(*loop.get_diagram());
  }

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

    auto& kmpc_context = diagram_ptr->GetMutableSubsystemContext(*cmpc, &diagram_context);

    cmpc->get_x_des_input_port().FixValue(&kmpc_context, x_des);
    cmpc->get_fsm_input_port().FixValue(&kmpc_context, VectorXd::Zero(1));
    cmpc->get_state_input_port().FixValue(&kmpc_context, robot_out);

    auto out = cmpc->AllocateOutput();
    cmpc->CalcOutput(kmpc_context, out.get());
  }

  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::DoMain(argc, argv);
}
