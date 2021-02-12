#include <stdlib.h>
#include <cmath>
#include <string>
#include <gflags/gflags.h>

#include "common/eigen_utils.h"
#include "dairlib/lcmt_dairlib_signal.hpp"
#include "dairlib/lcmt_saved_traj.hpp"
#include "dairlib/lcmt_timestamped_vector.hpp"
#include "examples/Cassie/cassie_utils.h"
#include "examples/goldilocks_models/controller/control_parameters.h"
#include "examples/goldilocks_models/controller/osc_rom_walking_gains.h"
#include "examples/goldilocks_models/goldilocks_utils.h"
#include "examples/goldilocks_models/reduced_order_models.h"
#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "multibody/multipose_visualizer.h"
#include "systems/controllers/fsm_event_time.h"
#include "systems/dairlib_signal_lcm_systems.h"
#include "systems/framework/lcm_driven_loop.h"
#include "systems/framework/output_vector.h"
#include "systems/robot_lcm_systems.h"

#include "examples/goldilocks_models/controller/rom_inverse_kinematics.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/yaml/yaml_read_archive.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/primitives/multiplexer.h"

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
DEFINE_int32(rom_option, 4, "See find_goldilocks_models.cc");
DEFINE_int32(iter, -1, "The iteration # of the theta that you use");

DEFINE_int32(knots_per_mode, 24, "Number of knots per mode in rom traj opt");
DEFINE_double(opt_tol, 1e-2, "");
DEFINE_double(feas_tol, 1e-2, "");

DEFINE_bool(use_ipopt, false, "use ipopt instead of snopt");
DEFINE_bool(log_solver_info, true,
            "Log snopt output to a file or ipopt to terminal");

// Flag for debugging
DEFINE_bool(debug_mode, false, "Only run the traj opt once locally");
DEFINE_bool(read_from_file, false, "Files for input port values");

// LCM channels (non debug mode)
DEFINE_string(channel_x, "CASSIE_STATE_SIMULATION",
              "LCM channel for receiving state. "
              "Use CASSIE_STATE_SIMULATION to get state from simulator, and "
              "use CASSIE_STATE_DISPATCHER to get state from state estimator");
DEFINE_string(
    channel_fsm_t, "FSM_T",
    "LCM channel for receiving fsm and time of latest liftoff event.");
DEFINE_string(channel_y, "MPC_OUTPUT", "");
DEFINE_string(channel_ik, "IK_OUTPUT", "");

// (for non debug mode)
DEFINE_string(init_file, "", "Initial Guess for ik");
DEFINE_bool(start_with_left_stance, true,
            "The starting stance of the robot. This is used to prepare "
            "ourselves for MPC");

int DoMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  // Read-in the parameters
  OSCRomWalkingGains gains;
  const YAML::Node& root = YAML::LoadFile(FindResourceOrThrow(GAINS_FILENAME));
  drake::yaml::YamlReadArchive(root).Accept(&gains);

  // Build Cassie MBP
  //  drake::multibody::MultibodyPlant<double> plant_feedback(0.0);
  //  addCassieMultibody(&plant_feedback, nullptr, true /*floating base*/,
  //                     "examples/Cassie/urdf/cassie_v2.urdf",
  //                     true /*spring model*/, false /*loop closure*/);
  // plant_feedback.Finalize();
  // Build fix-spring Cassie MBP
  drake::multibody::MultibodyPlant<double> plant_control(0.0);
  addCassieMultibody(&plant_control, nullptr, true,
                     "examples/Cassie/urdf/cassie_fixed_springs.urdf", false,
                     false);
  plant_control.Finalize();

  // Parameters for the traj opt
  // TODO: some of the fields might be unnecessary
  IKSetting param;
  param.rom_option = FLAGS_rom_option;
  param.iter = (FLAGS_iter >= 0) ? FLAGS_iter : gains.model_iter;
  param.knots_per_mode = FLAGS_knots_per_mode;
  param.feas_tol = FLAGS_feas_tol;
  param.opt_tol = FLAGS_opt_tol;
  param.use_ipopt = FLAGS_use_ipopt;
  param.log_solver_info = FLAGS_log_solver_info;
  param.dir_model =
      "../dairlib_data/goldilocks_models/planning/robot_1/models/";
  param.dir_data = "../dairlib_data/goldilocks_models/ik/robot_1/data/";
  param.init_file = FLAGS_init_file;

  if (FLAGS_debug_mode) {
    if (!CreateFolderIfNotExist(param.dir_model)) return 0;
    if (!CreateFolderIfNotExist(param.dir_data)) return 0;
  }

  // Build the controller diagram
  DiagramBuilder<double> builder;

  drake::lcm::DrakeLcm lcm_local("udpm://239.255.76.67:7667?ttl=0");

  // We probably cannot have two lcmsubsribers listening to the same channel?
  // https://github.com/RobotLocomotion/drake/blob/master/lcm/drake_lcm_interface.h#L242

  // Create Lcm receiver for fsm and latest lift off time (translate the lcm to
  // BasicVector)
  int lcm_vector_size = 2;
  auto fsm_and_liftoff_time_receiver =
      builder.AddSystem<systems::DairlibSignalReceiver>(lcm_vector_size);

  // Create traj publisher
  auto q_traj_publisher =
      builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_saved_traj>(
          FLAGS_channel_ik, &lcm_local,
          TriggerTypeSet({TriggerType::kForced})));

  // Create optimal rom trajectory generator
  auto ik_block = builder.AddSystem<RomInverseKinematics>(plant_control, param,
                                                          FLAGS_debug_mode);
  builder.Connect(ik_block->get_output_port(0),
                  q_traj_publisher->get_input_port());

  // Create the diagram
  auto owned_diagram = builder.Build();
  owned_diagram->set_name("IK");

  // Run lcm-driven simulation
  std::vector<const drake::systems::LeafSystem<double>*> lcm_parsers = {
      fsm_and_liftoff_time_receiver, ik_block};
  std::vector<std::string> input_channels = {FLAGS_channel_fsm_t,
                                             FLAGS_channel_y};
  systems::TwoLcmDrivenLoop<dairlib::lcmt_dairlib_signal,
                            dairlib::lcmt_saved_traj>
      loop(&lcm_local, std::move(owned_diagram), lcm_parsers, input_channels,
           true);
  if (!FLAGS_debug_mode) {
    loop.Simulate();
  } else {
    // TODO: finish the manual part

    // Manually set the input ports of CassiePlannerWithMixedRomFom and evaluate
    // the output (we do not run the LcmDrivenLoop)

    /*// Initialize some values
    double is_right_stance;
    if (FLAGS_read_from_file) {
      is_right_stance =
          readCSV(param.dir_data + "is_right_stance_test.csv")(0, 0);
    } else {
      is_right_stance = !FLAGS_start_with_left_stance;
    }

    ///
    /// Read in initial robot state
    ///
    VectorXd x_init;

    // Visualize the initial pose
    multibody::MultiposeVisualizer visualizer = multibody::MultiposeVisualizer(
        FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf"),
        1);
    visualizer.DrawPoses(x_init);

    ///
    /// Set input ports
    ///

    // fsm_state is currently not used in the leafsystem
    double fsm_state = 0;

    // Construct robot output (init state and current time)
    double prev_lift_off_time = 0;
    double current_time = 0;
    OutputVector<double> robot_output(
        x_init.head(plant_control.num_positions()),
        x_init.tail(plant_control.num_velocities()),
        VectorXd::Zero(plant_control.num_actuators()));
    robot_output.set_timestamp(current_time);

    // Get contexts
    auto diagram_ptr = loop.get_diagram();
    auto& diagram_context = loop.get_diagram_mutable_context();
    auto& planner_context =
        diagram_ptr->GetMutableSubsystemContext(*ik_block, &diagram_context);

    // Set input port value
    ik_block->get_input_port_stance_foot().FixValue(&planner_context,
                                                    is_right_stance);
    ik_block->get_input_port_init_phase().FixValue(&planner_context,
                                                   init_phase);
    ik_block->get_input_port_state().FixValue(&planner_context, robot_output);
    ik_block->get_input_port_fsm_and_lo_time().FixValue(
        &planner_context,
        drake::systems::BasicVector({fsm_state, prev_lift_off_time}));

    ///
    /// Eval output port and store data
    ///

    // Calc output
    auto output = ik_block->AllocateOutput();
    ik_block->CalcOutput(planner_context, output.get());

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
    cout << "first trajectory in the lcmt_saved_traj:\n";
    string traj_name_0 = traj_data.GetTrajectoryNames()[0];
    cout << "time_vector = \n"
         << traj_data.GetTrajectory(traj_name_0).time_vector.transpose()
         << endl;
    cout << "datapoints = \n"
         << traj_data.GetTrajectory(traj_name_0).datapoints << endl;*/
  }

  return 0;
}

}  // namespace dairlib::goldilocks_models

int main(int argc, char* argv[]) {
  return dairlib::goldilocks_models::DoMain(argc, argv);
}
