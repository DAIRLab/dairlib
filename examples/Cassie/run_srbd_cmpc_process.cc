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

#include "dairlib/lcmt_robot_output.hpp"
#include "common/find_resource.h"
#include "common/file_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/controllers/time_based_fsm.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/dairlib_signal_lcm_systems.h"
#include "examples/Cassie/mpc/cassie_srbd_cmpc_gains.h"
#include "examples/Cassie/cassie_utils.h"
#include "systems/controllers/mpc/srbd_cmpc.h"

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
using Eigen::Matrix3d;

DEFINE_string(gains_filename, "examples/Cassie/mpc/cassie_srbd_cmpc_gains.yaml", "convex mpc gains file");
DEFINE_string(channel_x, "CASSIE_STATE_DISPATCHER", "channel to publish/receive cassie state");
DEFINE_string(channel_plan, "SRBD_MPC_OUT", "channel to publish plan trajectory");
DEFINE_string(channel_fsm, "FSM", "the name of the channel with the time-based fsm");
DEFINE_double(stance_time, 0.3, "duration of each stance phase");
DEFINE_bool(debug_mode, false, "Manually set MPC values to debug");
DEFINE_bool(use_com, false, "Use center of mass or a point to track CM location");
DEFINE_double(debug_time, 0.00, "time to simulate system at");
DEFINE_double(swing_ft_height, 0.01, "Swing foot height");
DEFINE_double(v_des, 0.4, "desired walking speed");
DEFINE_double(h_des, 0.75, "Desired pelvis height");
DEFINE_double(dt, 0.01, "time step for koopman mpc");

// Code-gen of cross product basis

int DoMain(int argc, char* argv[]) {

  SrbdMpcGains gains;
  const YAML::Node& root =
      YAML::LoadFile(FindResourceOrThrow(FLAGS_gains_filename));
  drake::yaml::YamlReadArchive(root).Accept(&gains);


  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // mpc parameters
  double dt = FLAGS_dt;

  DrakeLcm lcm_local;


  // diagram builder
  DiagramBuilder<double> builder;

  // Add MBP
  MultibodyPlant<double> plant(0.0);
  addCassieMultibody(&plant, nullptr, true,
      "examples/Cassie/urdf/cassie_v2.urdf", true, false);
  plant.Finalize();

  auto map = multibody::makeNameToPositionsMap(plant);
  auto plant_context = plant.CreateDefaultContext();
  auto x0 = plant.GetPositionsAndVelocities(*plant_context);

  // Cassie SRBD model setup
  Vector3d des_com_pos = {0, 0, 0.82};
  Vector3d des_pelvis_pos = {0, 0, 0.95};
  Vector3d com_offset = {0, 0, -0.128};
  Vector3d left_neutral_foot_pos = {0,  0.05, 0};
  Vector3d left_safe_nominal_foot_pos = {0, 0.125, 0};
  Vector3d right_neutral_foot_pos = -left_neutral_foot_pos;
  Vector3d right_safe_nomonal_foot_pos = -left_safe_nominal_foot_pos;
  Matrix3d I_rot;
  I_rot << 0.91, 0.04, 0.09, 0.04, 0.55, -0.001, 0.08, -0.001, 0.89;
  double mass = 30.0218;

  auto cmpc = builder.AddSystem<SrbdCMPC>(plant, plant_context.get(), dt,
                                            FLAGS_swing_ft_height,
                                            false, false, true,  FLAGS_use_com);

  // add base to track position and orientation
  cmpc->AddBaseFrame("pelvis", com_offset);

  int nxi = 18;
  int nu = 10;
  MatrixXd Al = MatrixXd::Zero(nxi, nxi);
  MatrixXd Bl = MatrixXd::Zero(nxi, nu);
  VectorXd bl = VectorXd::Zero(nxi);
  MatrixXd Ar = MatrixXd::Zero(nxi, nxi);
  MatrixXd Br = MatrixXd::Zero(nxi, nu);
  VectorXd br = VectorXd::Zero(nxi);

  SrbdCMPC::CopyContinuous3dSrbDynamics(mass, 0.0, BipedStance::kLeft,
      I_rot, des_com_pos, left_neutral_foot_pos, &Al, &Bl, &bl);
  SrbdCMPC::CopyContinuous3dSrbDynamics(mass, 0.0, BipedStance::kRight,
      I_rot, des_com_pos, right_neutral_foot_pos, &Ar, &Br, &br);

  SrbdDynamics left_stance_dynamics = {Al, Bl, bl};
  SrbdDynamics right_stance_dynamics = {Ar, Br, br};

  cmpc->AddMode(left_stance_dynamics, BipedStance::kLeft, std::round(FLAGS_stance_time / dt));
  cmpc->AddMode(right_stance_dynamics, BipedStance::kRight, std::round(FLAGS_stance_time / dt));

  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  Vector3d mid_contact_point = (left_toe.first + left_heel.first) / 2.0;

  // add contact points
  auto left_pt = std::pair<const drake::multibody::BodyFrame<double> &, Eigen::Vector3d>(
      plant.GetBodyByName("toe_left").body_frame(), mid_contact_point);

  auto right_pt = std::pair<const drake::multibody::BodyFrame<double> &, Eigen::Vector3d>(
      plant.GetBodyByName("toe_right").body_frame(), mid_contact_point);

  cmpc->AddContactPoint(left_pt, BipedStance::kLeft);
  cmpc->AddContactPoint(right_pt, BipedStance::kRight);
  std::vector<VectorXd> kin_nom = {des_com_pos - left_safe_nominal_foot_pos, des_com_pos - right_safe_nomonal_foot_pos};
  cmpc->SetReachabilityLimit(gains.kin_reachability_lim, kin_nom, gains.W_kin_reach);

  // set mass
  cmpc->SetMass(mass);

  // add tracking objective
  VectorXd x_des = VectorXd::Zero(cmpc->num_state_inflated());
  x_des(2) = des_pelvis_pos(2);
  x_des(3) = FLAGS_v_des;

  std::cout << "x desired:\n" << x_des <<std::endl;

  MatrixXd qq = gains.q.asDiagonal();
  std::cout << "Gains: \n" << qq;

  cmpc->AddTrackingObjective(x_des, gains.q.asDiagonal());
  cmpc->SetTerminalCost(gains.qf.asDiagonal());
  cmpc->SetFlatGroundSoftConstraint(gains.W_flat_ground);

  // add input cost
  cmpc->AddInputRegularization(gains.r.asDiagonal());

  // set friction coeff
  cmpc->SetMu(gains.mu);
  cmpc->Build();
  std::cout << "Successfully built kmpc" << std::endl;

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
  auto koopman_mpc_out_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_saved_traj>(FLAGS_channel_plan, &lcm_local));

  // fsm connections
  builder.Connect(fsm->get_output_port(), cmpc->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(), fsm_send->get_input_port());
  builder.Connect(fsm_send->get_output_port(), fsm_pub->get_input_port());

  builder.Connect(robot_out->get_output_port(), fsm->get_input_port_state());
  builder.Connect(robot_out->get_output_port(),
                  cmpc->get_state_input_port());

  builder.Connect(cmpc->get_output_port(),
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

    auto& kmpc_context = diagram_ptr->GetMutableSubsystemContext(*cmpc, &diagram_context);

    cmpc->get_x_des_input_port().FixValue(&kmpc_context, x_des);
    cmpc->get_fsm_input_port().FixValue(&kmpc_context, VectorXd::Zero(1));
    cmpc->get_state_input_port().FixValue(&kmpc_context, robot_out);

    auto out = cmpc->AllocateOutput();
    cmpc->CalcOutput(kmpc_context, out.get());

    cmpc->print_initial_state_constraints();
    cmpc->print_dynamics_constraints();
    cmpc->print_state_knot_constraints();
  }

  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::DoMain(argc, argv);
}
