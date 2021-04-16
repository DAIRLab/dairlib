//
// Created by brian on 4/14/21.
//

#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/lcmt_drake_signal.hpp"

#include "koopman_mpc.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "common/find_resource.h"
#include "common/file_utils.h"
#include "systems/robot_lcm_systems.h"
#include "systems/controllers/time_based_fsm.h"
#include "systems/framework/lcm_driven_loop.h"

namespace dairlib {

using systems::RobotOutputReceiver;
using systems::TimeBasedFiniteStateMachine;
using systems::LcmDrivenLoop;

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

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

VectorXd poly_basis_1 (const VectorXd& x) {
  return x;
}

int DoMain(int argc, char* argv[]) {
  // Koopman parameters
  double dt = 0.025;
  double stance_time = 0.35;

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

  VectorXd x0 = VectorXd::Zero(plant.num_positions() + plant.num_velocities());
  auto plant_context = plant.CreateDefaultContext();

  plant.SetPositionsAndVelocities(plant_context.get(), x0);

  Vector3d default_com = plant.CalcCenterOfMassPositionInWorld(*plant_context.get());

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

  kmpc->AddMode(left_stance_dynamics, koopMpcStance::kLeft, std::round(stance_time / dt));
  kmpc->AddMode(right_stance_dynamics, koopMpcStance::kRight, std::round(stance_time / dt));

  // add contact points
  auto left_pt = std::pair<const drake::multibody::BodyFrame<double> &, Eigen::Vector3d>(
          plant.GetBodyByName("left_lower_leg").body_frame(), Vector3d(0, 0, -0.5));

  auto right_pt = std::pair<const drake::multibody::BodyFrame<double> &, Eigen::Vector3d>(
      plant.GetBodyByName("right_lower_leg").body_frame(), Vector3d(0, 0, -0.5));

  kmpc->AddContactPoint(left_pt, koopMpcStance::kLeft);
  kmpc->AddContactPoint(right_pt, koopMpcStance::kRight);

  // Set kinematic reachability constraint
  std::vector<VectorXd> kin_nom = {Vector2d(-default_com(kmpc->saggital_idx()), -default_com(kmpc->vertical_idx())),
                                   Vector2d(-default_com(kmpc->saggital_idx()), -default_com(kmpc->vertical_idx()))};

  kmpc->SetReachabilityLimit(0.4*VectorXd::Ones(2), kin_nom);

  // add base pivot angle
  kmpc->AddJointToTrackBaseAngle("hip_pin", "hip_pindot");

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
  kmpc->AddInputRegularization(0.00001 * VectorXd::Ones(6).asDiagonal());

  // set friction coeff
  kmpc->SetMu(0.4);
  kmpc->Build();
  std::cout << "Successfully built kmpc" << std::endl;

  // Setup fsm
  std::vector<int> fsm_states = {koopMpcStance::kLeft, koopMpcStance::kRight};
  std::vector<double> state_durations = {dt, dt};
  auto fsm = builder.AddSystem<TimeBasedFiniteStateMachine>(plant, fsm_states, state_durations);

  builder.Connect(fsm->get_output_port(), kmpc->get_fsm_input_port());

  // setup lcm messaging
  auto robot_out = builder.AddSystem<RobotOutputReceiver>(plant);
  auto dispatcher_out_subscriber = builder.AddSystem(LcmSubscriberSystem::Make<lcmt_robot_output>("PLANAR_DISPATCHER_OUT", &lcm_local));
  auto koopman_mpc_out_publisher = builder.AddSystem(LcmPublisherSystem::Make<lcmt_saved_traj>("KOOPMAN_MPC_OUT", &lcm_local));

  builder.Connect(dispatcher_out_subscriber->get_output_port(),
      robot_out->get_input_port());

  builder.Connect(robot_out->get_output_port(),
      kmpc->get_state_input_port());

  builder.Connect(kmpc->get_output_port(),
      koopman_mpc_out_publisher->get_input_port());

  auto owned_diagram = builder.Build();
  owned_diagram->set_name("MPC");

  *owned_diagram;
  LcmDrivenLoop<lcmt_robot_output> loop(&lcm_local, std::move(owned_diagram),
      dispatcher_out_subscriber, "PLANAR_DISPATCHER_OUT", false);

  loop.Simulate();
  return 0;
}

}

int main(int argc, char* argv[]) {
  return dairlib::DoMain(argc, argv);
}
