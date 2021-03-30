#include <stdlib.h>
#include <unistd.h>  // sleep/usleep
#include <cmath>
#include <fstream>
#include <string>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/goldilocks_models/controller/cassie_rom_planner_system.h"
#include "examples/goldilocks_models/controller/control_parameters.h"
#include "examples/goldilocks_models/controller/osc_rom_walking_gains.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "multibody/multipose_visualizer.h"
#include "systems/controllers/osc/osc_utils.h"
#include "systems/dairlib_signal_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/robot_lcm_systems.h"

#include "dairlib/lcmt_dairlib_signal.hpp"
#include "examples/goldilocks_models/controller/planner_preprocessing.h"
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

// Planner settings
DEFINE_int32(rom_option, -1, "See find_goldilocks_models.cc");
DEFINE_int32(iter, -1, "The iteration # of the theta that you use");
DEFINE_int32(sample, -1, "The sample # of the initial condition that you use");

DEFINE_int32(n_step, 3, "Number of foot steps in rom traj opt");
DEFINE_double(final_position, 2, "The final position for the robot");

DEFINE_double(w_Q, 1, "");
DEFINE_double(w_R, 1, "");
DEFINE_double(w_rom_reg, 1, "cost weight for the ROM state regularization");

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

DEFINE_bool(use_ipopt, false, "use ipopt instead of snopt");
DEFINE_bool(log_solver_info, true,
            "Log snopt output to a file or ipopt to terminal");
DEFINE_double(time_limit, 0, "time limit for the solver.");

// Flag for debugging
DEFINE_bool(debug_mode, false, "Only run the traj opt once locally");
DEFINE_int32(solve_idx_for_read_from_file, -1,
             "Files index for input port values");

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

// (for non debug mode)
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

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Read-in the parameters
  OSCRomWalkingGains gains;
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
  }

  // Build Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_feedback(0.0);
  addCassieMultibody(&plant_feedback, nullptr, true /*floating base*/,
                     "examples/Cassie/urdf/cassie_v2.urdf",
                     true /*spring model*/, false /*loop closure*/);
  plant_feedback.Finalize();
  // Build fix-spring Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_control(0.0);
  addCassieMultibody(&plant_control, nullptr, true,
                     "examples/Cassie/urdf/cassie_fixed_springs.urdf", false,
                     false);
  plant_control.Finalize();

  // Parameters for the traj opt
  PlannerSetting param;
  param.rom_option =
      (FLAGS_rom_option >= 0) ? FLAGS_rom_option : gains.rom_option;
  param.iter = (FLAGS_iter >= 0) ? FLAGS_iter : gains.model_iter;
  param.sample = (FLAGS_sample >= 0) ? FLAGS_sample : gains.sample_idx;
  param.n_step = FLAGS_n_step;
  param.knots_per_mode = FLAGS_knots_per_mode;
  // TODO: temporarily commented out FLAGS_final_position for testing
  //  param.final_position_x = FLAGS_final_position;
  param.final_position_x = gains.stride_length * FLAGS_n_step;
  param.zero_touchdown_impact = FLAGS_zero_touchdown_impact;
  param.use_double_contact_points = FLAGS_use_double_contact_points;
  param.equalize_timestep_size = FLAGS_equalize_timestep_size;
  param.fix_duration = FLAGS_fix_duration;
  param.feas_tol = FLAGS_feas_tol;
  param.opt_tol = FLAGS_opt_tol;
  param.max_iter = FLAGS_max_iter;
  param.use_ipopt = FLAGS_use_ipopt;
  param.log_solver_info = FLAGS_log_solver_info;
  param.time_limit = FLAGS_time_limit;
  param.w_Q = gains.w_Q;
  param.w_R = gains.w_R;
  param.w_rom_reg = gains.w_rom_reg;
  param.w_reg_quat_ = gains.w_reg_quat;
  param.w_reg_xy_ = gains.w_reg_xy;
  param.w_reg_z_ = gains.w_reg_z;
  param.w_reg_joints_ = gains.w_reg_joints;
  param.dir_model =
      "../dairlib_data/goldilocks_models/planning/robot_1/models/";
  param.dir_data = "../dairlib_data/goldilocks_models/planning/robot_1/data/";
  param.init_file = FLAGS_init_file;
  param.solve_idx_for_read_from_file = FLAGS_solve_idx_for_read_from_file;
  param.gains = gains;

  if (FLAGS_debug_mode) {
    if (!CreateFolderIfNotExist(param.dir_model)) return 0;
    if (!CreateFolderIfNotExist(param.dir_data)) return 0;
  }

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  // We probably cannot have two lcmsubsribers listening to the same channel?
  // https://github.com/RobotLocomotion/drake/blob/master/lcm/drake_lcm_interface.h#L242

  // Create state receiver.
  auto state_receiver =
      builder.AddSystem<systems::RobotOutputReceiver>(plant_feedback);

  // Create Lcm receiver for fsm and latest lift off time (translate the lcm to
  // BasicVector)
  int lcm_vector_size = 2;
  auto fsm_and_liftoff_time_receiver =
      builder.AddSystem<systems::DairlibSignalReceiver>(lcm_vector_size);

  // Create mpc traj publisher
  auto traj_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_saved_traj>(
          FLAGS_channel_y, &lcm_local, TriggerTypeSet({TriggerType::kForced})));

  // Create a block that gets the stance leg
  std::vector<int> ss_fsm_states = {LEFT_STANCE, RIGHT_STANCE};
  auto stance_foot_getter = builder.AddSystem<CurrentStanceFoot>(ss_fsm_states);
  builder.Connect(fsm_and_liftoff_time_receiver->get_output_port(0),
                  stance_foot_getter->get_input_port_fsm_and_lo_time());

  // Create a block that compute the phase of the first mode
  double stride_period =
      gains.left_support_duration + gains.double_support_duration;
  auto init_phase_calculator =
      builder.AddSystem<PhaseInFirstMode>(plant_feedback, stride_period);
  builder.Connect(state_receiver->get_output_port(0),
                  init_phase_calculator->get_input_port_state());
  builder.Connect(fsm_and_liftoff_time_receiver->get_output_port(0),
                  init_phase_calculator->get_input_port_fsm_and_lo_time());

  // Create a block that computes the initial state for the planner
  auto x_init_calculator = builder.AddSystem<InitialStateForPlanner>(
      plant_feedback, plant_control, param.final_position_x, param.n_step);
  builder.Connect(state_receiver->get_output_port(0),
                  x_init_calculator->get_input_port_state());
  builder.Connect(init_phase_calculator->get_output_port(0),
                  x_init_calculator->get_input_port_init_phase());
  builder.Connect(fsm_and_liftoff_time_receiver->get_output_port(0),
                  x_init_calculator->get_input_port_fsm_and_lo_time());

  // Create optimal rom trajectory generator
  auto rom_planner = builder.AddSystem<CassiePlannerWithMixedRomFom>(
      plant_control, stride_period, gains.double_support_duration, param,
      FLAGS_debug_mode);
  builder.Connect(stance_foot_getter->get_output_port(0),
                  rom_planner->get_input_port_stance_foot());
  builder.Connect(init_phase_calculator->get_output_port(0),
                  rom_planner->get_input_port_init_phase());
  builder.Connect(x_init_calculator->get_output_port_adjusted_state(),
                  rom_planner->get_input_port_state());
  builder.Connect(x_init_calculator->get_output_port_adjustment(),
                  rom_planner->get_input_port_quat_xyz_shift());
  //  builder.Connect(fsm_and_liftoff_time_receiver->get_output_port(0),
  //                  rom_planner->get_input_port_fsm_and_lo_time());
  builder.Connect(rom_planner->get_output_port(0),
                  traj_publisher->get_input_port());

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name("MPC");

  CreateDiagramFigure(*owned_diagram);

  // Run lcm-driven simulation
  std::vector<const drake::systems::LeafSystem<double>*> lcm_parsers = {
      fsm_and_liftoff_time_receiver, state_receiver};
  std::vector<std::string> input_channels = {FLAGS_channel_fsm_t,
                                             FLAGS_channel_x};
  systems::TwoLcmDrivenLoop<dairlib::lcmt_dairlib_signal,
                            dairlib::lcmt_robot_output>
      loop(&lcm_local, std::move(owned_diagram), lcm_parsers, input_channels,
           true);
  if (!FLAGS_debug_mode) {
    loop.Simulate();
  } else {
    // Manually set the input ports of CassiePlannerWithMixedRomFom and evaluate
    // the output (we do not run the LcmDrivenLoop)

    // Initialize some values
    double init_phase;
    double is_right_stance;
    double current_time;
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
    } else {
      init_phase = FLAGS_init_phase;
      is_right_stance = !FLAGS_start_with_left_stance;
      current_time = 0;
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

      // Perturbing the initial floating base configuration for testing
      // trajopt
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
    multibody::MultiposeVisualizer visualizer = multibody::MultiposeVisualizer(
        FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"),
        1);
    visualizer.DrawPoses(x_init);

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
        multibody::makeNameToPositionsMap(plant_control);
    /*for (auto name : name_list) {
      cout << name << ", " << init_state(positions_map.at(name)) << endl;
    }*/
    // TODO: find out why the initial left knee position is not within the
    //  joint limits.
    //  Could be that the constraint tolerance is too high in rom optimization

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
    rom_planner->get_input_port_fsm_and_lo_time().FixValue(
        &planner_context,
        drake::systems::BasicVector({fsm_state, prev_lift_off_time}));

    ///
    /// Eval output port and store data
    ///

    // Calc output
    auto output = rom_planner->AllocateOutput();
    rom_planner->CalcOutput(planner_context, output.get());

    // Store data
    writeCSV(param.dir_data + string("n_step.csv"),
             param.n_step * VectorXd::Ones(1));
    writeCSV(param.dir_data + string("nodes_per_step.csv"),
             param.knots_per_mode * VectorXd::Ones(1));

    // Testing - checking the planner output
    const auto* abstract_value = output->get_data(0);
    const dairlib::lcmt_saved_traj& traj_msg =
        abstract_value->get_value<dairlib::lcmt_saved_traj>();
    LcmTrajectory traj_data(traj_msg);
    cout << "\nFirst-mode trajectory in the lcmt_saved_traj:\n";
    string traj_name_0 = traj_data.GetTrajectoryNames()[0];
    cout << "time_vector = \n"
         << traj_data.GetTrajectory(traj_name_0).time_vector.transpose()
         << endl;
    cout << "datapoints = \n"
         << traj_data.GetTrajectory(traj_name_0).datapoints << endl;
  }

  return 0;
}

}  // namespace dairlib::goldilocks_models

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::DoMain(argc, argv);
}
