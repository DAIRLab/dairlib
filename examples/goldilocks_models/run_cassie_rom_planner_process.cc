#include <string>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_trajectory_block.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/goldilocks_models/controller/cassie_rom_planner_system.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/lcmt_drake_signal.hpp"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"

namespace dairlib::goldilocks_models {

using std::cout;
using std::endl;
using std::string;
using std::to_string;
using std::vector;

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using drake::MatrixX;
using drake::VectorX;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::systems::DiagramBuilder;
using drake::systems::TriggerType;
using drake::systems::lcm::LcmPublisherSystem;
using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::TriggerTypeSet;
using drake::trajectories::PiecewisePolynomial;

using systems::OutputVector;

using multibody::JwrtqdotToJwrtv;

// Planner settings
DEFINE_int32(robot_option, 1, "0: plannar robot. 1: cassie_fixed_spring");
DEFINE_int32(rom_option, 4, "See find_goldilocks_models.cc");
DEFINE_int32(iter, 20, "The iteration # of the theta that you use");
DEFINE_int32(sample, 4, "The sample # of the initial condition that you use");

DEFINE_int32(n_step, 3, "Number of foot steps in rom traj opt");
DEFINE_double(final_position, 2, "The final position for the robot");

DEFINE_int32(knots_per_mode, 24, "Number of knots per mode in rom traj opt");
DEFINE_bool(fix_duration, false, "Fix the total time");
DEFINE_bool(equalize_timestep_size, true, "Make all timesteps the same size");
DEFINE_bool(zero_touchdown_impact, true, "Zero impact at foot touchdown");
DEFINE_double(opt_tol, 1e-4, "");
DEFINE_double(feas_tol, 1e-4, "");

DEFINE_bool(log_solver_info, false,
            "Log snopt output to a file or ipopt to terminal");
DEFINE_bool(use_ipopt, false, "use ipopt instead of snopt");

// Flag for debugging
DEFINE_bool(debug_mode, false, "Only run the traj opt once locally");

// LCM channels (non debug mode)
DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");
DEFINE_string(
    channel_fsm_t, "FSM_T",
    "LCM channel for receiving fsm and time of latest liftoff event. ");
DEFINE_string(channel_y, "MPC_OUTPUT",
              "The name of the channel which publishes command");

// (for non debug mode)
DEFINE_string(init_file, "", "Initial Guess for Planning Optimization");
DEFINE_double(init_phase, 0,
              "The phase where the initial FOM pose is throughout the single "
              "support period. This is used to prepare ourselves for MPC");
DEFINE_bool(start_with_left_stance, true,
            "The starting stance of the robot. This is used to prepare "
            "ourselves for MPC");
DEFINE_double(disturbance, 0, "Disturbance to FoM initial state");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_feedback(0.0);
  addCassieMultibody(&plant_feedback, nullptr, true /*floating base*/,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant_feedback.Finalize();
  // Build fix-spring Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_controls(0.0);
  addCassieMultibody(&plant_controls, nullptr, true,
                     "examples/Cassie/urdf/cassie_fixed_springs.urdf", false,
                     false);
  plant_controls.Finalize();

  // Parameters for the traj opt
  PlannerSetting param;
  param.rom_option = FLAGS_rom_option;
  param.iter = FLAGS_iter;
  param.sample = FLAGS_sample;
  param.n_step = FLAGS_n_step;
  param.knots_per_mode = FLAGS_knots_per_mode;
  param.final_position_x = FLAGS_final_position;
  param.zero_touchdown_impact = FLAGS_zero_touchdown_impact;
  param.equalize_timestep_size = FLAGS_equalize_timestep_size;
  param.fix_duration = FLAGS_fix_duration;
  param.feas_tol = FLAGS_feas_tol;
  param.opt_tol = FLAGS_opt_tol;
  param.use_ipopt = FLAGS_use_ipopt;
  param.log_solver_info = FLAGS_log_solver_info;
  param.w_Q = 1;
  param.w_R = 1;
  param.dir_model =
      "../dairlib_data/goldilocks_models/planning/robot_1/models/";
  param.dir_data = "../dairlib_data/goldilocks_models/planning/robot_1/data/";
  param.init_file = FLAGS_init_file;

  if (FLAGS_debug_mode) {
    if (!CreateFolderIfNotExist(param.dir_model)) return 0;
    if (!CreateFolderIfNotExist(param.dir_data)) return 0;
  }

  // Read in initial robot state
  VectorXd x_init;  // we assume that solution from files are in left stance
  if (FLAGS_debug_mode) {
    string model_dir_n_pref = param.dir_model + to_string(FLAGS_iter) +
                              string("_") + to_string(FLAGS_sample) +
                              string("_");
    cout << "model_dir_n_pref = " << model_dir_n_pref << endl;
    int n_sample_raw =
        readCSV(model_dir_n_pref + string("time_at_knots.csv")).size();
    x_init = readCSV(model_dir_n_pref + string("state_at_knots.csv"))
                 .col(int(n_sample_raw * FLAGS_init_phase));
    // Mirror x_init if it's right stance
    if (!FLAGS_start_with_left_stance) {
      // Create mirror maps
      StateMirror state_mirror(
          MirrorPosIndexMap(plant_controls, OptimalRomPlanner::ROBOT),
          MirrorPosSignChangeSet(plant_controls, OptimalRomPlanner::ROBOT),
          MirrorVelIndexMap(plant_controls, OptimalRomPlanner::ROBOT),
          MirrorVelSignChangeSet(plant_controls, OptimalRomPlanner::ROBOT));
      // Mirror the state
      x_init.head(plant_controls.num_positions()) =
          state_mirror.MirrorPos(x_init.head(plant_controls.num_positions()));
      x_init.tail(plant_controls.num_velocities()) =
          state_mirror.MirrorVel(x_init.tail(plant_controls.num_velocities()));
    }

    if (FLAGS_disturbance != 0) {
      //    x_init(9) += FLAGS_disturbance / 1;
    }

    // Testing
    std::vector<string> name_list = {"base_qw",
                                     "base_qx",
                                     "base_qy",
                                     "base_qz",
                                     "base_x",
                                     "base_y",
                                     "base_z",
                                     "hip_roll_left",
                                     "hip_roll_right",
                                     "hip_yaw_left",
                                     "hip_yaw_right",
                                     "hip_pitch_left",
                                     "hip_pitch_right",
                                     "knee_left",
                                     "knee_right",
                                     "ankle_joint_left",
                                     "ankle_joint_right",
                                     "toe_left",
                                     "toe_right"};
    std::map<string, int> positions_map =
        multibody::makeNameToPositionsMap(plant_controls);
    /*for (auto name : name_list) {
      cout << name << ", " << init_state(positions_map.at(name)) << endl;
    }*/
    // TODO: find out why the initial left knee position is not within the joint
    //  limits.
    //  Could be that the constraint tolerance is too high in rom optimization
  }

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  // Create state receiver.
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_feedback);

  // Create Lcm subscriber for fsm and latest lift off time
  auto fsm_and_liftoff_time_receiver =
      builder.AddSystem(LcmSubscriberSystem::Make<drake::lcmt_drake_signal>(
          FLAGS_channel_fsm_t, &lcm_local));

  // Create mpc traj publisher
  auto traj_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_trajectory_block>(
          FLAGS_channel_y, &lcm_local, TriggerTypeSet({TriggerType::kForced})));

  // Create optimal rom trajectory generator
  // TODO(yminchen): need to centralize the fsm state
  int left_stance_state = 0;
  int right_stance_state = 1;
  double stride_period = 0.37;  // TODO(yminchen): this value should change
  std::vector<int> ss_fsm_states = {left_stance_state, right_stance_state};
  auto rom_planner = builder.AddSystem<OptimalRomPlanner>(
      plant_feedback, plant_controls, ss_fsm_states, stride_period, param,
      FLAGS_debug_mode);
  builder.Connect(state_receiver->get_output_port(0),
                  rom_planner->get_input_port_state());
  builder.Connect(fsm_and_liftoff_time_receiver->get_output_port(),
                  rom_planner->get_input_port_fsm_and_lo_time());
  builder.Connect(rom_planner->get_output_port(0),
                  traj_publisher->get_input_port());

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name("MPC");

  // Run lcm-driven simulation
  systems::LcmDrivenLoop<dairlib::lcmt_robot_output> loop(
      &lcm_local, std::move(owned_diagram), state_receiver, FLAGS_channel_x,
      true);
  if (!FLAGS_debug_mode) {
    loop.Simulate();
  } else {
    // TODO: finish this
    //...

    // Store data
    writeCSV(param.dir_data + string("n_step.csv"),
             param.n_step * VectorXd::Ones(1));
    writeCSV(param.dir_data + string("nodes_per_step.csv"),
             param.knots_per_mode * VectorXd::Ones(1));
  }

  return 0;
}

}  // namespace dairlib::goldilocks_models

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::DoMain(argc, argv);
}
