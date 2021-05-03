#include <drake/multibody/parsing/parser.h>
#include <gflags/gflags.h>
#include <drake/systems/lcm/lcm_subscriber_system.h>

#include "common/find_resource.h"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "lcm/lcm_trajectory.h"

#include "multibody/multibody_utils.h"

#include "model_utils.h"
#include "lipm_walking_speed_control.h"

#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/osc_tracking_data.h"
#include "systems/controllers/lipm_traj_gen.h"
#include "systems/controllers/swing_ft_traj_gen.h"
#include "systems/controllers/time_based_fsm.h"
#include "systems/controllers/fsm_event_time.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"

#include "examples/KoopmanMPC/osc_walking_gains.h"

#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/primitives/constant_value_source.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

enum stance {
  kLeft = 0,
  kRight = 1
};

namespace dairlib {

using std::map;
using std::pair;
using std::string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector2d;
using Eigen::VectorXd;

using drake::Value;
using drake::geometry::SceneGraph;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::TriggerTypeSet;
using drake::trajectories::PiecewisePolynomial;
using drake::systems::ConstantVectorSource;
using drake::systems::ConstantValueSource;

using systems::controllers::ComTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;
using systems::controllers::JointSpaceTrackingData;
using systems::LIPMTrajGenerator;
using systems::SwingFootTrajGenerator;
using systems::TimeBasedFiniteStateMachine;
using systems::FiniteStateMachineEventTime;

using koopman_examples::LipmWalkingSpeedControl;

namespace examples {

DEFINE_string(channel_x, "PLANAR_STATE","The name of the channel which receives state");
DEFINE_string(channel_u, "PLANAR_INPUT","The name of the channel which publishes command");
DEFINE_string(gains_filename,"examples/KoopmanMPC/osc_walking_gains.yaml","Filepath containing gains");
DEFINE_double(z_com, 1.05, "Desired CoM height");
DEFINE_double(stance_time, 0.35, "stance time");
DEFINE_double(mid_ft_height, 0.025, "mid swing foot height");
DEFINE_double(final_ft_height, -0.01, "end of swing foot height");
DEFINE_double(kin_reach, 0.3, "allowable footstep distance");
DEFINE_double(v_des, 0.25, "desired com velocity");
DEFINE_double(k_ff, 0, "Feed forward raibert gain");
DEFINE_double(k_fb, 0.2, "Feedback raibert gain");
DEFINE_bool(track_com, false,"use com tracking data (otherwise uses trans space)");
DEFINE_bool(print_osc_debug, false, "print osc_debug to the terminal");

void print_gains(const OSCWalkingGains& gains);

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build the controller diagram
  DiagramBuilder<double> builder;

  // Built the MBP
  drake::multibody::MultibodyPlant<double> plant(0.0);

  SceneGraph<double>& scene_graph = *(builder.AddSystem<SceneGraph>());
  LoadPlanarWalkerFromFile(plant, &scene_graph);
  plant.Finalize();

  auto left_pt = std::pair<const Vector3d, const drake::multibody::BodyFrame<double>&>(
      Vector3d(0, 0, -0.5),plant.GetBodyByName("left_lower_leg").body_frame());

  auto right_pt = std::pair<const Vector3d, const drake::multibody::BodyFrame<double> &>(
       Vector3d(0, 0, -0.5), plant.GetBodyByName("right_lower_leg").body_frame());

  auto plant_context = plant.CreateDefaultContext();

  // Get contact frames and position (doesn't matter whether we use
  // plant or plant_wo_springs because the contact frames exit in both
  // plants)
  int nv = plant.num_velocities();

  // Create maps for joints
  map<string, int> pos_map = multibody::makeNameToPositionsMap(plant);
  map<string, int> vel_map = multibody::makeNameToVelocitiesMap(plant);
  map<string, int> act_map = multibody::makeNameToActuatorsMap(plant);

  /**** Convert the gains from the yaml struct to Eigen Matrices ****/
  OSCWalkingGains gains;
  const YAML::Node& root =
      YAML::LoadFile(FindResourceOrThrow(FLAGS_gains_filename));
  drake::yaml::YamlReadArchive(root).Accept(&gains);


  std::vector<int> fsm_states = {stance::kLeft, stance::kRight};
  std::vector<double> state_durations = {FLAGS_stance_time, FLAGS_stance_time};

  std::vector<std::vector<std::pair<const Vector3d, const Frame<double>&>>> fsm_pts;
  fsm_pts.push_back({left_pt});
  fsm_pts.push_back({right_pt});

  std::vector<std::pair<const Vector3d, const Frame<double>&>> left_right_pts =
      {left_pt, right_pt};



  /**** Initialize all the leaf systems ****/
  drake::lcm::DrakeLcm lcm_local;

  auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(plant);

  auto fsm = builder.AddSystem<TimeBasedFiniteStateMachine>(
      plant, fsm_states, state_durations);

  auto touchdown_event_time = builder.AddSystem<FiniteStateMachineEventTime>();

  auto command_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
          FLAGS_channel_u, &lcm_local, TriggerTypeSet({TriggerType::kForced})));

  auto command_sender = builder.AddSystem<systems::RobotCommandSender>(plant);

  auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
      plant, plant, plant_context.get(), plant_context.get(), true, FLAGS_print_osc_debug);

  auto osc_debug_pub =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_osc_output>(
          "OSC_DEBUG_WALKING", &lcm_local, TriggerTypeSet({TriggerType::kForced})));

  auto base_traj_gen = builder.AddSystem<LIPMTrajGenerator>(
      plant, plant_context.get(), FLAGS_z_com, fsm_states, state_durations,
      fsm_pts, "torso_mass", FLAGS_track_com);

  auto walking_speed_control = builder.AddSystem<LipmWalkingSpeedControl>(
      plant, plant_context.get(), FLAGS_k_ff, FLAGS_k_fb, "torso_mass", state_durations.at(0));

  auto swing_ft_traj_gen = builder.AddSystem<SwingFootTrajGenerator>(
      plant, plant_context.get(), fsm_states, state_durations, left_right_pts,
      "torso_mass", FLAGS_mid_ft_height, FLAGS_final_ft_height,
      0, FLAGS_kin_reach, 0, 0, true, false, true, 1);

  PiecewisePolynomial<double> orientation =
      PiecewisePolynomial<double>::ZeroOrderHold(
          Vector2d(0, std::numeric_limits<double>::infinity()),
          Eigen::RowVector2d::Zero());

  auto orientation_traj = builder.AddSystem(
      std::make_unique<ConstantValueSource<double>>(
          Value<drake::trajectories::Trajectory<double>>(orientation)));

  auto des_vel = builder.AddSystem<ConstantVectorSource<double>>(
      VectorXd::Ones(1));

  /**** OSC setup ****/
  // Cost
  MatrixXd Q_accel = gains.w_accel * MatrixXd::Identity(nv, nv);
  osc->SetAccelerationCostForAllJoints(Q_accel);

  // Soft constraint on contacts
  osc->SetWeightOfSoftContactConstraint(gains.w_soft_constraint);

  // Contact information for OSC
  osc->SetContactFriction(gains.mu);

  Vector3d foot_contact_disp(0, 0, 0);

  auto left_foot_evaluator = multibody::WorldPointEvaluator(plant,
      left_pt.first, left_pt.second, Matrix3d::Identity(),
      foot_contact_disp,{0, 2});

  auto right_foot_evaluator = multibody::WorldPointEvaluator(plant,
      right_pt.first, right_pt.second,  Matrix3d::Identity(),
      foot_contact_disp, {0, 2});



  /*** tracking data ***/
  TransTaskSpaceTrackingData swing_foot_traj("swing_ft_traj",
      gains.K_p_swing_foot, gains.K_d_swing_foot, gains.W_swing_foot, plant, plant);

  swing_foot_traj.AddStateAndPointToTrack(kLeft, "right_lower_leg", right_pt.first);
  swing_foot_traj.AddStateAndPointToTrack(kRight, "left_lower_leg", left_pt.first);

  osc->AddTrackingData(&swing_foot_traj);

  gains.W_com(0,0) = 0;
  ComTrackingData com_traj("com_traj", gains.K_p_com,
                           gains.K_d_com, gains.W_com, plant, plant);

  TransTaskSpaceTrackingData torso_traj("com_traj", gains.K_p_com,
                                        gains.K_d_com, gains.W_com, plant, plant);
  torso_traj.AddPointToTrack("torso_mass");

  if (FLAGS_track_com) {
    osc->AddTrackingData(&com_traj);
  } else {
    osc->AddTrackingData(&torso_traj);
  }

  JointSpaceTrackingData angular_traj("base_angle", gains.K_p_orientation,
      gains.K_d_orientation, gains.W_orientation, plant, plant);

  angular_traj.AddJointToTrack("planar_roty", "planar_rotydot");

  osc->AddTrackingData(&angular_traj);

  // Build OSC problem
  osc->AddStateAndContactPoint(kLeft, &left_foot_evaluator);
  osc->AddStateAndContactPoint(kRight, &right_foot_evaluator);
  osc->Build();
  std::cout << "Built OSC" << std::endl;

  /*****Connect ports*****/

  // connect robot out receiver state out port
  builder.Connect(state_receiver->get_output_port(),
                  fsm->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(),
                  base_traj_gen->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(),
                  swing_ft_traj_gen->get_input_port_state());
  builder.Connect(state_receiver->get_output_port(),
                  osc->get_robot_output_input_port());
  builder.Connect(state_receiver->get_output_port(),
                  walking_speed_control->get_input_port_state());

  // fsm output port connections
  builder.Connect(fsm->get_output_port(), osc->get_fsm_input_port());
  builder.Connect(fsm->get_output_port(), touchdown_event_time->get_input_port_fsm());
  builder.Connect(fsm->get_output_port(), base_traj_gen->get_input_port_fsm());
  builder.Connect(fsm->get_output_port(), swing_ft_traj_gen->get_input_port_fsm());

  // fsm switch time output port connections
  builder.Connect(touchdown_event_time->get_output_port_event_time(),
                  base_traj_gen->get_input_port_fsm_switch_time());
  builder.Connect(touchdown_event_time->get_output_port_event_time(),
                  swing_ft_traj_gen->get_input_port_fsm_switch_time());
  builder.Connect(touchdown_event_time->get_output_port_event_time(),
                  walking_speed_control->get_input_port_fsm_switch_time());

  // connect lipm traj gens
  builder.Connect(base_traj_gen->get_output_port_lipm_from_current(),
                  swing_ft_traj_gen->get_input_port_com());
  builder.Connect(base_traj_gen->get_output_port_lipm_from_current(),
                  walking_speed_control->get_input_port_com());
  builder.Connect(walking_speed_control->get_output_port(),
                  swing_ft_traj_gen->get_input_port_sc());
  builder.Connect(des_vel->get_output_port(),
      walking_speed_control->get_input_port_des_vel());


  // connect tracking datas
  builder.Connect(base_traj_gen->get_output_port_lipm_from_touchdown(),
                  osc->get_tracking_data_input_port("com_traj"));
  builder.Connect(orientation_traj->get_output_port(),
                    osc->get_tracking_data_input_port("base_angle"));
  builder.Connect(swing_ft_traj_gen->get_output_port(),
                  osc->get_tracking_data_input_port("swing_ft_traj"));

  // Publisher connections
  builder.Connect(osc->get_osc_output_port(),
                  command_sender->get_input_port(0));
  builder.Connect(command_sender->get_output_port(0),
                  command_pub->get_input_port());
  builder.Connect(osc->get_osc_debug_port(), osc_debug_pub->get_input_port());

  // Run lcm-driven simulation
  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("id_walking_controller"));

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x, true);

  loop.Simulate();


  return 0;
}

void print_gains(const OSCWalkingGains& gains) {
  std::cout <<"======== OSC WALKING GAINS ==========\n";
}

}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::examples::DoMain(argc, argv);
}
