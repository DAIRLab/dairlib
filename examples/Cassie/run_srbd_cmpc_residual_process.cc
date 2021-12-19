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
#include "systems/system_utils.h"
#include "systems/controllers/mpc/srbd_cmpc.h"
#include "systems/controllers/mpc/lipm_warmstart_system.h"
#include "systems/controllers/fsm_event_time.h"
#include "multibody/single_rigid_body_plant.h"
#include "examples/Cassie/mpc/cassie_srbd_cmpc_gains.h"
#include "examples/Cassie/cassie_utils.h"
#include "examples/Cassie/cassie_fixed_point_solver.h"
#include "systems/srbd_residual_estimator.h"
#include "systems/controllers/mpc/mpc_trajectory_reciever.h"

namespace dairlib {

using systems::RobotOutputReceiver;
using systems::TimeBasedFiniteStateMachine;
using systems::LcmDrivenLoop;
using systems::OutputVector;
using systems::LipmWarmStartSystem;
using systems::FiniteStateMachineEventTime;
using systems::SRBDResidualEstimator;
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
DEFINE_double(stance_time, 0.35, "duration of each stance phase");
DEFINE_bool(debug_mode, false, "Manually set MPC values to debug");
DEFINE_bool(use_com, false, "Use center of mass or a point to track CM location");
DEFINE_bool(print_diagram, false, "print block diagram");
DEFINE_double(debug_time, 0.00, "time to simulate system at");
DEFINE_double(stance_width, 0.0, "stance width to use in dynamics linearization");
DEFINE_double(v_des, 0.0, "desired walking speed");
DEFINE_double(h_des, 0.82, "Desired pelvis height");
DEFINE_double(dt, 0.01, "time step for srbd mpc");

int DoMain(int argc, char* argv[]) {
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
  auto srb_plant = SingleRigidBodyPlant(plant, plant_context.get(), FLAGS_use_com);

  // Cassie SRBD model setup
  Vector3d com_offset = {0, 0, -0.128};
  Vector3d des_pelvis_pos = {0, 0, FLAGS_h_des};
  Vector3d des_com_pos = des_pelvis_pos + com_offset;

  Vector3d left_neutral_foot_pos = {0,  FLAGS_stance_width, 0};
  Vector3d left_safe_nominal_foot_pos = {0, 0.125, 0};
  Vector3d right_neutral_foot_pos = -left_neutral_foot_pos;
  Vector3d right_safe_nominal_foot_pos = -left_safe_nominal_foot_pos;
  Matrix3d I_rot = Vector3d(0.91, 0.55, 0.89).asDiagonal();
//  I_rot << 0.91, 0.04, 0.09, 0.04, 0.55, -0.001, 0.08, -0.001, 0.89;
  std::cout << "I:\n" << I_rot <<std::endl;

  double mass = 30.0218;
  srb_plant.AddBaseFrame("pelvis", com_offset);

  auto left_toe = LeftToeFront(plant);
  auto left_heel = LeftToeRear(plant);
  Vector3d mid_contact_point = (left_toe.first + left_heel.first) / 2.0;

  // add contact points
  auto left_pt = std::pair<Vector3d, const drake::multibody::BodyFrame<double> &>(
      mid_contact_point, plant.GetBodyByName("toe_left").body_frame());
  auto right_pt = std::pair<Vector3d, const drake::multibody::BodyFrame<double> &>(
      mid_contact_point, plant.GetBodyByName("toe_right").body_frame());
  srb_plant.SetMass(mass);
  srb_plant.AddContactPoint(left_pt, BipedStance::kLeft);
  srb_plant.AddContactPoint(right_pt, BipedStance::kRight);

  int nx = 12;
  int nu = 5;
  MatrixXd Al = MatrixXd::Zero(nx, nx+3);
  MatrixXd Bl = MatrixXd::Zero(nx, nu);
  VectorXd bl = VectorXd::Zero(nx);
  MatrixXd Ar = MatrixXd::Zero(nx, nx+3);
  MatrixXd Br = MatrixXd::Zero(nx, nu);
  VectorXd br = VectorXd::Zero(nx);

  srb_plant.CopyContinuousLinearized3dSrbDynamicsForMPC(
      mass, 0, BipedStance::kLeft,
      I_rot, des_com_pos, left_neutral_foot_pos, &Al, &Bl, &bl);
  srb_plant.CopyContinuousLinearized3dSrbDynamicsForMPC(
      mass, 0, BipedStance::kRight,
      I_rot, des_com_pos, right_neutral_foot_pos, &Ar, &Br, &br);

  LinearSrbdDynamics left_stance_dynamics = {Al, Bl, bl};
  LinearSrbdDynamics right_stance_dynamics = {Ar, Br, br};

  auto cmpc = builder.AddSystem<SrbdCMPC>(srb_plant, dt, false, true, true);
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
  x_des(6) = FLAGS_v_des;
  std::cout << "xd:\n" << x_des << std::endl;

  cmpc->AddTrackingObjective(x_des, gains.q.asDiagonal());
  cmpc->SetTerminalCost(gains.qf.asDiagonal());
  cmpc->AddInputRegularization(gains.r.asDiagonal());
  cmpc->AddFootPlacementRegularization(0.1*Eigen::Matrix3d::Identity());

  // set friction coeff
  cmpc->SetMu(gains.mu);
  cmpc->Build();

  // Wire everything up
  auto xdes_source = builder.AddSystem<ConstantVectorSource<double>>(x_des);
  builder.Connect(xdes_source->get_output_port(), cmpc->get_x_des_input_port());

  std::vector<int> fsm_states = {BipedStance::kLeft, BipedStance::kRight};
  std::vector<BipedStance> fsm_stances = {BipedStance::kLeft, BipedStance::kRight};
  std::vector<double> state_durations = {FLAGS_stance_time, FLAGS_stance_time};

  auto fsm = builder.AddSystem<TimeBasedFiniteStateMachine>(
      plant, fsm_states, state_durations);

//  auto warmstarter = builder.AddSystem<LipmWarmStartSystem>(
//      srb_plant, FLAGS_h_des, FLAGS_stance_time,
//      FLAGS_dt, fsm_states, fsm_stances);
//
//  auto liftoff_event_time =
//      builder.AddSystem<FiniteStateMachineEventTime>(plant, fsm_states);

  // setup lcm messaging
  auto robot_out = builder.AddSystem<RobotOutputReceiver>(plant);
  auto mpc_out_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<lcmt_saved_traj>(FLAGS_channel_plan, &lcm_local));

  auto lstsq_sys = builder.AddSystem<SRBDResidualEstimator>(
      srb_plant, 0.01, 200, true, 1.0/2000.0, true);
  lstsq_sys->AddMode(left_stance_dynamics, BipedStance::kLeft,
                     MatrixXd::Identity(nx, nx), std::round(FLAGS_stance_time / dt));
  lstsq_sys->AddMode(right_stance_dynamics, BipedStance::kRight,
                     MatrixXd::Identity(nx, nx), std::round(FLAGS_stance_time / dt));

  builder.Connect(fsm->get_output_port(), lstsq_sys->get_fsm_input_port());
  builder.Connect(robot_out->get_output_port(), lstsq_sys->get_state_input_port());
  builder.Connect(cmpc->get_output_port(), lstsq_sys->get_mpc_input_port());
  builder.Connect(lstsq_sys->get_residual_output_port(),
                  cmpc->get_residual_input_port());
  builder.Connect(fsm->get_output_port(), cmpc->get_fsm_input_port());
//  builder.Connect(fsm->get_output_port(), warmstarter->get_input_port_fsm());
//  builder.Connect(fsm->get_output_port(),
//                  liftoff_event_time->get_input_port_fsm());
//  builder.Connect(robot_out->get_output_port(),
//                  liftoff_event_time->get_input_port_state());
  builder.Connect(robot_out->get_output_port(), fsm->get_input_port_state());
//  builder.Connect(robot_out->get_output_port(), warmstarter->get_input_port_state());
  builder.Connect(robot_out->get_output_port(),
                  cmpc->get_state_input_port());
//  builder.Connect(liftoff_event_time->get_output_port_event_time(),
//                  warmstarter->get_input_port_touchdown_time());
//  builder.Connect(xdes_source->get_output_port(),
//                  warmstarter->get_xdes_input_port());
//  builder.Connect(warmstarter->get_output_port_lipm_from_current(),
//                  cmpc->get_warmstart_input_port());
//  builder.Connect(warmstarter->get_output_port_foot_target(),
//                  cmpc->get_foot_target_input_port());

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
    drake::multibody::MultibodyPlant<double> plant_for_solver(0.0);
    addCassieMultibody(&plant_for_solver, nullptr, true,
                       "examples/Cassie/urdf/cassie_v2.urdf", true, false);
    plant_for_solver.Finalize();
    VectorXd q_init, u_init, lambda_init;
    CassieFixedPointSolver(plant_for_solver, FLAGS_h_des, gains.mu,
                           70, true, FLAGS_stance_width, &q_init, &u_init, &lambda_init);

    OutputVector<double> rbt_out_msg(q_init,
                                     VectorXd::Zero(plant.num_velocities()),
                                     VectorXd::Zero(plant.num_actuators()));

    rbt_out_msg.set_timestamp(FLAGS_debug_time);
    auto diagram_ptr = loop.get_diagram();
    auto& diagram_context = loop.get_diagram_mutable_context();

    diagram_context.SetTime(FLAGS_debug_time);

    auto& cmpc_context = diagram_ptr->GetMutableSubsystemContext(*cmpc, &diagram_context);
    auto& mpc_pub_context = diagram_ptr->GetMutableSubsystemContext(*mpc_out_publisher, &diagram_context);

    cmpc->get_x_des_input_port().FixValue(&cmpc_context, x_des);
    cmpc->get_fsm_input_port().FixValue(&cmpc_context, VectorXd::Zero(1));
    cmpc->get_state_input_port().FixValue(&cmpc_context, rbt_out_msg);
    mpc_out_publisher->Publish(mpc_pub_context);

  }
  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::DoMain(argc, argv);
}
