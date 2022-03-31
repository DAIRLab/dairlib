#include <stdlib.h>
#include <unistd.h>  // sleep/usleep

#include <cmath>
#include <fstream>
#include <string>

#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "dairlib/lcmt_dairlib_signal.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/goldilocks_models/controller/cassie_rom_planner_system.h"
#include "examples/goldilocks_models/controller/control_parameters.h"
#include "examples/goldilocks_models/controller/planner_preprocessing.h"
#include "examples/goldilocks_models/rom_walking_gains.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "multibody/multipose_visualizer.h"
#include "multibody/pinocchio_plant.h"
#include "systems/dairlib_signal_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"
#include "systems/system_utils.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
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

DEFINE_bool(broadcast, false,
            "broadcast between controller thread and planner thread. only used "
            "on hardware");
/*DEFINE_bool(create_new_data_folder, true,
            "create new folder to prevent overwriting; only for hardware");*/

// Planner settings
DEFINE_int32(rom_option, -1, "See find_goldilocks_models.cc");
DEFINE_int32(iter, -1, "The iteration # of the theta that you use");
DEFINE_int32(sample, -1,
             "The sample # of the trajopt file that we used for 1) the "
             "regularization terms of both ROM and FOM, and 2) initial guess");

DEFINE_int32(n_step, 3, "Number of foot steps in rom traj opt");
DEFINE_int32(n_step_lipm, 0, "Number of foot steps of lipm cascased");
// TODO: We can probably remove final_position
DEFINE_double(final_position, 2, "The final position for the robot");
DEFINE_double(stride_length, -10000, "set constant walking stride length");
DEFINE_double(stride_length_scaling, 1.0, "");

DEFINE_int32(knots_per_mode, 24, "Number of knots per mode in rom traj opt");
DEFINE_bool(fix_duration, true,
            "Fix the total time. (could lead to faster solve but possibly "
            "worse solution)");
DEFINE_bool(equalize_timestep_size, true, "Make all timesteps the same size");
DEFINE_bool(zero_touchdown_impact, false, "Zero impact at foot touchdown");
DEFINE_bool(use_double_contact_points, true, "");
DEFINE_double(opt_tol, 1e-2, "");
DEFINE_double(feas_tol, 1e-2, "");
DEFINE_int32(max_iter, 10000, "Maximum iteration for the solver");

// Time limit
DEFINE_double(time_limit, 0, "time limit for the solver.");
DEFINE_double(realtime_rate_for_time_limit, 1, "");

// Solver options
DEFINE_bool(use_ipopt, false, "use ipopt instead of snopt");
DEFINE_bool(switch_to_snopt_after_first_loop, true,
            "use snopt after the first loop");

// Logging
DEFINE_bool(log_solver_info, true,
            "Log snopt output to a file or ipopt to terminal");
DEFINE_bool(log_data, true, "Save the planner data into files");
DEFINE_int32(print_level, 1, "");

// Flag for debugging
DEFINE_bool(run_one_loop_to_get_init_file, false, "");
DEFINE_bool(debug_mode, false, "Only run the traj opt once locally");
DEFINE_int32(solve_idx_for_read_from_file, -1,
             "Files index for input port values");
DEFINE_string(dir_data, "", "data directory; for convenience when testing");

// LCM channels (non debug mode)
DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");
DEFINE_string(
    channel_fsm_t, "FSM_T",
    "LCM channel for receiving fsm and time of latest liftoff event.");
DEFINE_string(channel_y, "MPC_OUTPUT",
              "The name of the channel which publishes command");

// Simulated robot
DEFINE_bool(spring_model, true, "Use a URDF with or without legs springs");

// (for non debug mode)
DEFINE_string(dir_and_prefix_FOM, "",
              "file location and prefix for FOM poses (used in planner's "
              "regularization cost)");
DEFINE_string(init_file, "", "Initial Guess for Planning Optimization");
DEFINE_double(init_phase, 0,
              "The phase where the initial FOM pose is throughout the single "
              "support period. This is used to prepare ourselves for MPC");
DEFINE_bool(start_with_left_stance, true,
            "The starting stance of the robot. This is used to prepare "
            "ourselves for MPC");
DEFINE_double(xy_disturbance, 0,
              "Disturbance to FoM initial state. Range from 0 to 1");
DEFINE_double(yaw_disturbance, 0,
              "Disturbance to FoM initial state. Range from 0 to 1");

// Testing
DEFINE_string(lcm_url_port, "7667", "port number. Should be > 1024");
DEFINE_string(path_wait_identifier, "", "");

DEFINE_bool(completely_use_trajs_from_model_opt_as_target, false, "");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_broadcast) {
    DRAKE_DEMAND(FLAGS_lcm_url_port == "7667");
  }

  // Read-in the parameters
  RomWalkingGains gains;
  const YAML::Node& root = YAML::LoadFile(FindResourceOrThrow(GAINS_FILENAME));
  drake::yaml::YamlReadArchive(root).Accept(&gains);

  // Note that 0 <= phase < 1, but we allows the phase to be 1 here for testing
  // purposes
  DRAKE_DEMAND(0 <= FLAGS_init_phase && FLAGS_init_phase <= 1);

  DRAKE_DEMAND(0 <= FLAGS_xy_disturbance && FLAGS_xy_disturbance <= 1);
  DRAKE_DEMAND(0 <= FLAGS_yaw_disturbance && FLAGS_yaw_disturbance <= 1);

  if (!FLAGS_debug_mode) {
    // When using with controller, we need to align the steps with the FSM
    DRAKE_DEMAND(FLAGS_fix_duration);
    DRAKE_DEMAND(FLAGS_equalize_timestep_size);
  }
  if (FLAGS_solve_idx_for_read_from_file >= 0) {
    DRAKE_DEMAND(FLAGS_debug_mode);
    // We manually assign the init file to be: "%d_init_file.csv" % solve_idx
    DRAKE_DEMAND(FLAGS_init_file.empty());
  }
  if (FLAGS_run_one_loop_to_get_init_file) {
    DRAKE_DEMAND(FLAGS_log_data);
  }

  if (gains.use_virtual_radio) {
    cout << "Set `use_radio` to true because `use_virtual_radio` = true\n\n";
    gains.use_radio = true;
  }

  if (FLAGS_stride_length > -100) {
    gains.set_constant_walking_speed = true;
    gains.constant_step_length_x = FLAGS_stride_length;
  }
  gains.constant_step_length_x *= FLAGS_stride_length_scaling;

  if (FLAGS_completely_use_trajs_from_model_opt_as_target) {
    DRAKE_DEMAND(FLAGS_dir_and_prefix_FOM.empty());
  }
  if (!FLAGS_dir_and_prefix_FOM.empty()) {
    cout << "dir_and_prefix_FOM is specified, so we won't have any target ROM "
            "traj in the regularization term\n";
  }

  // We only create new data folders for hardware experiment (broadcast case)
  /*if (FLAGS_broadcast) {
    bool create_new_data_folder = FLAGS_create_new_data_folder;
    if (!FLAGS_init_file.empty()) {
      // DRAKE_DEMAND(!FLAGS_create_new_data_folder);
      create_new_data_folder = false;
    }

    // Check if we want to use a new folder
    string temp_path = gains.dir_data;
    temp_path.pop_back();
    bool assigned_a_folder = false;
    int n_max_folders = 200;
    if (create_new_data_folder) {
      for (int i = 0; i < n_max_folders; i++) {
        if (!folder_exist(temp_path + "_" + to_string(i), false)) {
          gains.dir_data = temp_path + "_" + to_string(i) + "/";
          assigned_a_folder = true;
          break;
        }
      }
    } else {
      DRAKE_DEMAND(
          !folder_exist(temp_path + "_" + to_string(n_max_folders), false));
      for (int i = n_max_folders; i >= 0; i--) {
        if (folder_exist(temp_path + "_" + to_string(i), false)) {
          gains.dir_data = temp_path + "_" + to_string(i) + "/";
          assigned_a_folder = true;
          break;
        }
      }
    }
    // DRAKE_DEMAND(assigned_a_folder);
    if (!assigned_a_folder) {
      throw std::runtime_error(
          "Too many data folders! Delete some or increase the limit.");
    }
  }*/

  if (!FLAGS_dir_data.empty()) gains.dir_data = FLAGS_dir_data;

  // Create data folder if it doesn't exist
  cout << "data directory = " << gains.dir_data << endl;
  if (!CreateFolderIfNotExist(gains.dir_data, false)) return 0;

  // Parameters for the traj opt
  PlannerSetting param;
  param.rom_option =
      (FLAGS_rom_option >= 0) ? FLAGS_rom_option : gains.rom_option;
  param.iter = (FLAGS_iter >= 0) ? FLAGS_iter : gains.model_iter;
  param.sample = (FLAGS_sample >= 0) ? FLAGS_sample : gains.sample_idx;
  param.n_step = FLAGS_n_step;
  param.n_step_lipm = FLAGS_n_step_lipm;
  param.knots_per_mode = FLAGS_knots_per_mode;
  param.zero_touchdown_impact = FLAGS_zero_touchdown_impact;
  param.use_double_contact_points = FLAGS_use_double_contact_points;
  param.equalize_timestep_size = FLAGS_equalize_timestep_size;
  param.fix_duration = FLAGS_fix_duration;
  param.feas_tol = FLAGS_feas_tol;
  param.opt_tol = FLAGS_opt_tol;
  param.max_iter = FLAGS_max_iter;
  param.use_ipopt = FLAGS_use_ipopt;
  param.switch_to_snopt_after_first_loop =
      FLAGS_switch_to_snopt_after_first_loop;
  param.log_solver_info = FLAGS_log_solver_info;
  param.time_limit = FLAGS_time_limit;
  param.realtime_rate_for_time_limit = FLAGS_realtime_rate_for_time_limit;
  param.dir_model = gains.dir_model;
  param.dir_data = gains.dir_data;
  param.init_file = FLAGS_init_file;
  param.dir_and_prefix_FOM = FLAGS_dir_and_prefix_FOM;
  param.solve_idx_for_read_from_file = FLAGS_solve_idx_for_read_from_file;
  param.gains = gains;
  if (FLAGS_solve_idx_for_read_from_file >= 0) {
    param.rom_option = readCSV(param.dir_data + "rom_option.csv")(0, 0);
    param.iter = readCSV(param.dir_data + "model_iter.csv")(0, 0);
    param.sample = readCSV(param.dir_data + "sample_idx.csv")(0, 0);
    param.init_file =
        to_string(FLAGS_solve_idx_for_read_from_file) + "_init_file.csv";
  }

  // Store data
  writeCSV(param.dir_data + string("n_step.csv"),
           param.n_step * VectorXd::Ones(1));
  writeCSV(param.dir_data + string("nodes_per_step.csv"),
           param.knots_per_mode * VectorXd::Ones(1));

  //
  PiecewisePolynomial<double> x_traj;
  MatrixXd x_samples0;
  if (FLAGS_completely_use_trajs_from_model_opt_as_target) {
    std::string model_dir_n_pref = param.dir_model + to_string(param.iter) +
                                   string("_") + to_string(param.sample) +
                                   string("_");
    DRAKE_DEMAND(file_exist(model_dir_n_pref + "t_breaks0.csv"));
    DRAKE_DEMAND(file_exist(model_dir_n_pref + "x_samples0.csv"));
    DRAKE_DEMAND(file_exist(model_dir_n_pref + "xdot_samples0.csv"));
    x_traj = PiecewisePolynomial<double>::CubicHermite(
        readCSV(model_dir_n_pref + string("t_breaks0.csv")).col(0),
        readCSV(model_dir_n_pref + string("x_samples0.csv")),
        readCSV(model_dir_n_pref + string("xdot_samples0.csv")));
    x_samples0 = readCSV(model_dir_n_pref + string("x_samples0.csv"));
  }

  // Build Cassie MBP
  std::string urdf = FLAGS_spring_model
                         ? "examples/Cassie/urdf/cassie_v2.urdf"
                         : "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  drake::multibody::MultibodyPlant<double> plant_feedback(0.0);
  addCassieMultibody(&plant_feedback, nullptr, true /*floating base*/, urdf,
                     FLAGS_spring_model, false /*loop closure*/);
  plant_feedback.Finalize();
  // Build fix-spring Cassie MBP
  std::string fixed_spring_urdf =
      "examples/Cassie/urdf/cassie_fixed_springs.urdf";
  //  drake::multibody::MultibodyPlant<double> plant_control(0.0);
  multibody::PinocchioPlant<double> plant_control(0.0, fixed_spring_urdf);
  addCassieMultibody(&plant_control, nullptr, true, fixed_spring_urdf, false,
                     false);
  plant_control.Finalize();

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:" + FLAGS_lcm_url_port +
                                 "?ttl=0");
  drake::lcm::DrakeLcm lcm_network("udpm://239.255.76.67:7667?ttl=1");

  // We probably cannot have two lcmsubsribers listening to the same channel?
  // https://github.com/RobotLocomotion/drake/blob/master/lcm/drake_lcm_interface.h#L242

  // Create state receiver.
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_feedback);

  // Create Lcm receiver for fsm and latest lift off time (translate the lcm to
  // BasicVector)
  int lcm_vector_size = 5;
  auto controller_signal_receiver =
      builder.AddSystem<systems::DairlibSignalReceiver>(lcm_vector_size);

  // Create mpc traj publisher
  auto traj_publisher = builder.AddSystem(
      LcmPublisherSystem::Make<dairlib::lcmt_timestamped_saved_traj>(
          FLAGS_channel_y, FLAGS_broadcast ? &lcm_network : &lcm_local,
          TriggerTypeSet({TriggerType::kForced})));

  // Create a block that gets the stance leg
  std::vector<int> ss_fsm_states = {gains.left_support, gains.right_support};
  auto stance_foot_getter = builder.AddSystem<CurrentStanceFoot>(ss_fsm_states);
  builder.Connect(controller_signal_receiver->get_output_port(0),
                  stance_foot_getter->get_input_port_fsm_and_lo_time());

  // Create a block that compute the phase of the first mode
  double stride_period =
      gains.left_support_duration + gains.double_support_duration;
  auto init_phase_calculator =
      builder.AddSystem<PhaseInFirstMode>(plant_feedback, stride_period);
  builder.Connect(state_receiver->get_output_port(0),
                  init_phase_calculator->get_input_port_state());
  builder.Connect(controller_signal_receiver->get_output_port(0),
                  init_phase_calculator->get_input_port_fsm_and_lo_time());

  // Create a block that compute target position for the planner
  Eigen::Vector2d global_target_pos(gains.global_target_position_x,
                                    gains.global_target_position_y);
  PlannerFinalPosition* planner_final_pos;
  if (gains.use_radio) {
    planner_final_pos = builder.AddSystem<PlannerFinalPosition>(
        plant_feedback, stride_period, param.n_step);
    builder.Connect(controller_signal_receiver->get_output_port(0),
                    planner_final_pos->get_input_port_fsm_and_lo_time());
  } else if (FLAGS_completely_use_trajs_from_model_opt_as_target) {
    // The implementation currently assume y is close to 0
    if (!(abs(x_samples0(5, 0) - 0) < 1e-3)) {
      cout << "WARNING!! x_samples0(5, 0) = " << x_samples0(5, 0) << endl;
    }
    // DRAKE_DEMAND(abs(x_samples0(5, 0) - 0) < 1e-3);

    planner_final_pos = builder.AddSystem<PlannerFinalPosition>(
        plant_feedback, Eigen::Vector2d(x_samples0.rightCols(1)(4), 0),
        param.n_step);
  } else {
    planner_final_pos = gains.set_constant_walking_speed
                            ? builder.AddSystem<PlannerFinalPosition>(
                                  plant_feedback,
                                  Eigen::Vector2d(gains.constant_step_length_x,
                                                  gains.constant_step_length_y),
                                  param.n_step)
                            : builder.AddSystem<PlannerFinalPosition>(
                                  plant_feedback, global_target_pos);
  }
  builder.Connect(state_receiver->get_output_port(0),
                  planner_final_pos->get_input_port_state());
  builder.Connect(init_phase_calculator->get_output_port(0),
                  planner_final_pos->get_input_port_init_phase());

  // Create a block that computes the initial state for the planner
  auto x_init_calculator = builder.AddSystem<InitialStateForPlanner>(
      plant_feedback, plant_control, param.n_step, stride_period,
      FLAGS_spring_model);
  if (FLAGS_completely_use_trajs_from_model_opt_as_target) {
    x_init_calculator->completely_use_trajs_from_model_opt_as_target(x_traj);
  }
  builder.Connect(stance_foot_getter->get_output_port(0),
                  x_init_calculator->get_input_port_stance_foot());
  builder.Connect(state_receiver->get_output_port(0),
                  x_init_calculator->get_input_port_state());
  builder.Connect(init_phase_calculator->get_output_port(0),
                  x_init_calculator->get_input_port_init_phase());
  builder.Connect(controller_signal_receiver->get_output_port(0),
                  x_init_calculator->get_input_port_fsm_and_lo_time());

  // Create optimal rom trajectory generator
  auto rom_planner = builder.AddSystem<CassiePlannerWithMixedRomFom>(
      plant_control, stride_period, param, FLAGS_debug_mode, FLAGS_log_data,
      FLAGS_print_level);
  if (FLAGS_completely_use_trajs_from_model_opt_as_target)
    rom_planner->completely_use_trajs_from_model_opt_as_target();
  builder.Connect(stance_foot_getter->get_output_port(0),
                  rom_planner->get_input_port_stance_foot());
  builder.Connect(init_phase_calculator->get_output_port(0),
                  rom_planner->get_input_port_init_phase());
  builder.Connect(x_init_calculator->get_output_port_adjusted_state(),
                  rom_planner->get_input_port_state());
  builder.Connect(x_init_calculator->get_output_port_adjustment(),
                  rom_planner->get_input_port_quat_xyz_shift());
  builder.Connect(planner_final_pos->get_output_port(0),
                  rom_planner->get_input_port_planner_final_pos());
  builder.Connect(controller_signal_receiver->get_output_port(0),
                  rom_planner->get_input_port_fsm_and_lo_time());
  builder.Connect(rom_planner->get_output_port(0),
                  traj_publisher->get_input_port());

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name("MPC");

  DrawAndSaveDiagramGraph(*owned_diagram);

  // Run lcm-driven simulation
  std::vector<const drake::systems::LeafSystem<double>*> lcm_parsers = {
      controller_signal_receiver, state_receiver};
  std::vector<std::string> input_channels = {FLAGS_channel_fsm_t,
                                             FLAGS_channel_x};
  systems::TwoLcmDrivenLoop<dairlib::lcmt_dairlib_signal,
                            dairlib::lcmt_robot_output>
      loop(FLAGS_broadcast ? &lcm_network : &lcm_local,
           std::move(owned_diagram), lcm_parsers, input_channels, true,
           FLAGS_run_one_loop_to_get_init_file
               ? 1
               : std::numeric_limits<int>::infinity());
  if (!FLAGS_debug_mode) {
    // Create the file to indicate that the planner thread is listening
    if (!FLAGS_path_wait_identifier.empty()) {
      std::system(("touch " + FLAGS_path_wait_identifier).c_str());
      cout << "Created " << FLAGS_path_wait_identifier << endl;
    }

    loop.Simulate();
  } else {
    // Manually set the input ports of CassiePlannerWithMixedRomFom and evaluate
    // the output (we do not run the LcmDrivenLoop)

    // Initialize some values
    double init_phase;
    double is_right_stance;
    double current_time;
    double global_fsm_idx;
    VectorXd quat_xyz_shift(7);
    VectorXd final_position(2);
    if (FLAGS_solve_idx_for_read_from_file >= 0) {
      init_phase = readCSV(param.dir_data +
                           to_string(FLAGS_solve_idx_for_read_from_file) +
                           "_init_phase.csv")(0, 0);
      is_right_stance = readCSV(param.dir_data +
                                to_string(FLAGS_solve_idx_for_read_from_file) +
                                "_is_right_stance.csv")(0, 0);
      current_time = readCSV(param.dir_data +
                             to_string(FLAGS_solve_idx_for_read_from_file) +
                             "_current_time.csv")(0, 0);
      // writeCSV(param.dir_data + "testing_" + string("init_phase.csv"),
      //         init_phase * VectorXd::Ones(1), true);
      // writeCSV(param.dir_data + "testing_" + string("is_right_stance.csv"),
      //         is_right_stance * VectorXd::Ones(1), true);
      // writeCSV(param.dir_data + "testing_" + string("current_time.csv"),
      //         current_time * VectorXd::Ones(1), true);
      quat_xyz_shift = readCSV(param.dir_data +
                               to_string(FLAGS_solve_idx_for_read_from_file) +
                               "_quat_xyz_shift.csv")
                           .col(0);
      final_position = readCSV(param.dir_data +
                               to_string(FLAGS_solve_idx_for_read_from_file) +
                               "_final_position.csv")
                           .col(0);
      global_fsm_idx = readCSV(param.dir_data +
                               to_string(FLAGS_solve_idx_for_read_from_file) +
                               "_global_fsm_idx.csv")(0, 0);
    } else {
      init_phase = FLAGS_init_phase;
      is_right_stance = !FLAGS_start_with_left_stance;
      current_time = 0;
      quat_xyz_shift << 1, 0, 0, 0, 0, 0, 0;
      final_position << 0, 0;
      global_fsm_idx = 0;
    }

    ///
    /// Read in initial robot state
    ///
    VectorXd x_init;  // we assume that solution from files are in left stance
    if (FLAGS_solve_idx_for_read_from_file >= 0) {
      // Testing -- read x_init directly from a file
      x_init = readCSV(param.dir_data +
                       to_string(FLAGS_solve_idx_for_read_from_file) +
                       "_x_init.csv");
      // writeCSV(param.dir_data + "testing_" + string("x_init.csv"), x_init,
      // true);

      cout << "x_init = " << x_init.transpose() << endl;
    } else {
      string model_dir_n_pref = param.dir_model + to_string(param.iter) +
                                string("_") + to_string(param.sample) +
                                string("_");
      cout << "model_dir_n_pref = " << model_dir_n_pref << endl;
      int n_sample_raw =
          readCSV(model_dir_n_pref + string("time_at_knots.csv")).size();
      x_init = readCSV(model_dir_n_pref + string("x_samples.csv"))
                   .col(int(round((n_sample_raw - 1) * init_phase)));

      // Mirror x_init if it's right stance
      if (!FLAGS_start_with_left_stance) {
        // Create mirror maps
        StateMirror state_mirror(
            MirrorPosIndexMap(plant_control,
                              CassiePlannerWithMixedRomFom::ROBOT),
            MirrorPosSignChangeSet(plant_control,
                                   CassiePlannerWithMixedRomFom::ROBOT),
            MirrorVelIndexMap(plant_control,
                              CassiePlannerWithMixedRomFom::ROBOT),
            MirrorVelSignChangeSet(plant_control,
                                   CassiePlannerWithMixedRomFom::ROBOT));
        // Mirror the state
        x_init.head(plant_control.num_positions()) =
            state_mirror.MirrorPos(x_init.head(plant_control.num_positions()));
        x_init.tail(plant_control.num_velocities()) =
            state_mirror.MirrorVel(x_init.tail(plant_control.num_velocities()));
      }

      cout << "x_init = " << x_init.transpose() << endl;

      // Perturbing the initial floating base configuration for testing trajopt
      srand((unsigned int)time(0));
      if (FLAGS_yaw_disturbance > 0) {
        double theta = M_PI * VectorXd::Random(1)(0) * FLAGS_yaw_disturbance;
        Vector3d vec(0, 0, 1);
        x_init.head(4) << cos(theta / 2), sin(theta / 2) * vec.normalized();
      }
      if (FLAGS_xy_disturbance > 0) {
        x_init.segment<2>(4) = 10 * VectorXd::Random(2) * FLAGS_xy_disturbance;
      }

      cout << "x_init = " << x_init.transpose() << endl;
    }

    // Visualize the initial pose
    /*multibody::MultiposeVisualizer visualizer =
    multibody::MultiposeVisualizer(
        FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"),
        1);
    visualizer.DrawPoses(x_init);*/

    ///
    /// Set input ports
    ///

    // fsm_state is currently not used in the leafsystem
    double fsm_state = 0;

    // Construct robot output (init state and current time)
    double prev_lift_off_time = 0;
    OutputVector<double> robot_output(
        x_init.head(plant_control.num_positions()),
        x_init.tail(plant_control.num_velocities()),
        VectorXd::Zero(plant_control.num_actuators()));
    // The current time is not set in the robot output anymore
    robot_output.set_timestamp(-1);

    // Get contexts
    auto diagram_ptr = loop.get_diagram();
    auto& diagram_context = loop.get_diagram_mutable_context();
    diagram_context.SetTime(current_time);
    auto& planner_context =
        diagram_ptr->GetMutableSubsystemContext(*rom_planner, &diagram_context);

    // Set input port value
    rom_planner->get_input_port_stance_foot().FixValue(&planner_context,
                                                       is_right_stance);
    rom_planner->get_input_port_init_phase().FixValue(&planner_context,
                                                      init_phase);
    rom_planner->get_input_port_state().FixValue(&planner_context,
                                                 robot_output);
    rom_planner->get_input_port_quat_xyz_shift().FixValue(&planner_context,
                                                          quat_xyz_shift);
    rom_planner->get_input_port_planner_final_pos().FixValue(&planner_context,
                                                             final_position);
    rom_planner->get_input_port_fsm_and_lo_time().FixValue(
        &planner_context,
        drake::systems::BasicVector({fsm_state, prev_lift_off_time,
                                     global_fsm_idx, 0.0, 0.0, current_time}));

    ///
    /// Eval output port and store data
    ///

    // Calc output
    auto output = rom_planner->AllocateOutput();
    rom_planner->CalcOutput(planner_context, output.get());
  }

  return 0;
}

}  // namespace dairlib::goldilocks_models

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::DoMain(argc, argv);
}
