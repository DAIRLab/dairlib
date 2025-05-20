#include <lcm/lcm-cpp.hpp>
#include <gflags/gflags.h>
#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>
#include <unistd.h>
#include <limits.h>
#include "dairlib/lcmt_c3_state.hpp"
#include "dairlib/lcmt_object_state.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "dairlib/lcmt_radio_out.hpp"
#include <Eigen/Dense>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/framework/diagram_builder.h>
#include "systems/controllers/sampling_based_c3_controller.h"
#include "examples/sampling_c3/parameter_headers/franka_c3_controller_params.h"
#include "examples/sampling_c3/parameter_headers/sampling_c3_options.h"
#include "examples/sampling_c3/parameter_headers/franka_sim_params.h"
#include "systems/system_utils.h"
#include "systems/framework/timestamped_vector.h"
#include "solvers/c3_output.h"
#include "solvers/lcs_factory.h"

// To use this file, run as follows based on the demo name: 
// bazel-bin/examples/sampling_c3/test/lcm_log_loader --demo_name=jacktoy /mnt/data2/sharanya/logs/2025/05_07_25/000000 1
// bazel-bin/examples/sampling_c3/test/lcm_log_loader --demo_name=push_t /mnt/data2/sharanya/logs/2025/05_07_25/000001 1
// bazel-bin/examples/sampling_c3/test/lcm_log_loader --demo_name=box_topple /mnt/data2/sharanya/logs/2025/05_07_25/000002 1 

/////// WARNING!! //////
// This version of the log loader will not work for logs collected post March 31st, 2025 due to the changes made to 
// franka_c3_controller_params.h as per this commit where the safe params were removed:
// https://github.com/DAIRLab/dairlib/commit/c381e5cd89500cfd3d702ada6d3c659fe8bbb43e#diff-a6f0508e6ca4588155b0f75ba718f66042ed9d05ee1b7727a963820c6fcf0cca
// This is because the log loader is trying to read a vector of c3_options files which is no longer the case.

DEFINE_string(demo_name,
  "unkown",
  "Name for the demo, used when building filepaths for output.");

// Uncomment this line to output cost information for evenly spaced samples.
// #define DO_SAMPLE_VISUALIZATIONS

// Sample grid locations.
#define N_VERTICAL 100
#define N_HORIZONTAL 100

#define N_Q 10
#define N_V 9


namespace dairlib {

using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using drake::multibody::AddMultibodyPlantSceneGraph;
using dairlib::systems::TimestampedVector;
using drake::multibody::Parser;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RigidTransform;
using solvers::LCSFactory;


// Declare function that will generate samples around T location.
std::vector<Eigen::VectorXd> GenerateEvenlySpacedSamplesAroundT(
    const Eigen::VectorXd& x_lcs,
    const SamplingC3SamplingParams& sampling_params,
    const int& num_vertical, const int& num_horizontal
  );


// Declare function that will generate samples around jack location.
std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::Vector2d>> 
  GenerateEvenlySpacedSamplesAroundJack(
    const Eigen::VectorXd& x_lcs,
    const SamplingC3SamplingParams& sampling_params,
    const int& num_vertical, const int& num_horizontal
  );

// Declare helper function that will print a vector of Eigen::VectorXd in a
// Python-friendly format to define a numpy array.
void PythonFriendlyVectorOfVectorXdPrint(
  char* name, std::vector<Eigen::VectorXd> some_vector);

// Declare helper function that will write a vector of Eigen::VectorXd to a txt
// file such that it can be loaded from a python file via np.loadtxt.
void PythonFriendlyVectorOfVectorXdToFile(
  char* name, std::vector<Eigen::VectorXd> some_vector);


int DoMain(int argc,  char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::string base_path = "examples/sampling_c3/" + FLAGS_demo_name + "/";
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] <<" --demo_name={DEMO} <log_folder> <time_into_log>" <<
      std::endl;
    return 1;
  }
  std::cout<<"LOG LOADER FOR PUSH T EXAMPLE" << std::endl;
  const std::string& log_folder = std::string(argv[1]);
  const double& time_into_log = std::stod(std::string(argv[2]));

  // Turn the folder into a file path.
  std::string log_number = log_folder.substr(log_folder.find_last_of("/")+1, 6);
  std::string log_filepath = log_folder + "/simlog-" + log_number;
  std::cout<<"Parsing log at: " << log_filepath << std::endl;

  // Set the start time.
  // This time is how many seconds into the log we want to start processing
  // information. Convert to microseconds.
  const int64_t time_into_log_in_microsecs = time_into_log*1e6;

  std::cout << "time into log in seconds: " << time_into_log << std::endl;
  std::cout << "  in microseconds: " << time_into_log_in_microsecs << std::endl;

  // Load the recorded parameters:  (1/4) Controller parameters.
  std::string franka_c3_controller_params_path = log_filepath;
  std::string to_replace = "simlog-";
  std::string franka_c3_controller_params_path_replacement =
    "franka_c3_controller_params_";
  franka_c3_controller_params_path.replace(
    franka_c3_controller_params_path.find(to_replace),
    to_replace.length(), franka_c3_controller_params_path_replacement);
  FrankaC3ControllerParams controller_params =
    drake::yaml::LoadYamlFile<FrankaC3ControllerParams>(
      franka_c3_controller_params_path + ".yaml");
  std::cout<<"path of franka_c3_controller_params loaded: "<<
    franka_c3_controller_params_path << std::endl;

  // (2/4) C3 parameters.
  std::string c3_gains_path = log_filepath;
  std::string c3_gains_path_replacement = "c3_gains_";
  c3_gains_path.replace(c3_gains_path.find(to_replace), to_replace.length(),
                        c3_gains_path_replacement);
  C3Options c3_options;
  c3_options = drake::yaml::LoadYamlFile<C3Options>(c3_gains_path + ".yaml");
    
  // Sampling c3 options.
  // WARNING: ALL LOGS PRIOR TO MAY 15, 2025 WILL NOT HAVE A SAMPLING C3 OPTIONS.
  // PLEASE USE AN OLDER VERSION OF THIS FILE TO LOAD LOGS COLLECTED PRIOR TO MAY 15, 2025.
  std::string sampling_c3_options_path = log_filepath;
  std::string sampling_c3_options_path_replacement = "sampling_c3_options_";
  sampling_c3_options_path.replace(
    sampling_c3_options_path.find(to_replace), to_replace.length(),
    sampling_c3_options_path_replacement);
  SamplingC3Options sampling_c3_options =
    drake::yaml::LoadYamlFile<SamplingC3Options>(sampling_c3_options_path + ".yaml");
  // NOTE:  can temporarily hard code many more ADMM iterations or other
  // changes here, e.g.:
  // c3_options.admm_iter = 8;
  
  //example of updating Q to test effect of changing cost weights on C3 solve
  // std::vector<double> new_q {0.01, 0.01, 0.01,
  //          0.1, 0.1, 0.1, 0.1,
  //          250, 250, 250,
  //          2.5, 2.5, 2.5,
  //          0.05, 0.05, 0.05,
  //          0.05, 0.05, 0.05};
  // c3_options.q_vector_position_and_orientation = new_q;
  // Eigen::VectorXd q_position_and_orientation = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
  //       c3_options.q_vector_position_and_orientation.data(), c3_options.q_vector_position_and_orientation.size());
  // c3_options.Q_position_and_orientation = c3_options.w_Q * q_position_and_orientation.asDiagonal();
  // std::cout<<"Updated Q"<<std::endl;

  // (3/4) Sim parameters.
  std::string sim_params_path = log_filepath;
  std::string sim_params_path_replacement = "sim_params_";
  sim_params_path.replace(sim_params_path.find(to_replace), to_replace.length(),
                          sim_params_path_replacement);
  FrankaSimParams sim_params =
    drake::yaml::LoadYamlFile<FrankaSimParams>(sim_params_path + ".yaml");

  // (4/4) Sampling parameters.
  std::string sampling_params_path = log_filepath;
  std::string sampling_params_path_replacement = "sampling_params_";
  sampling_params_path.replace(
    sampling_params_path.find(to_replace), to_replace.length(),
    sampling_params_path_replacement);
  SamplingC3SamplingParams sampling_params = \
    drake::yaml::LoadYamlFile<SamplingC3SamplingParams>(
      sampling_params_path + ".yaml");
  // NOTE:  hard code the number of additional samples to be 0, since this
  // script is just to debug a single C3 solve.
  sampling_params.num_additional_samples_c3 = 0;
  sampling_params.num_additional_samples_repos = 0;


  // Create an instance of the LCM log handler.
  lcm::LCM lcm;
  lcm::LogFile log_file(log_filepath, "r");
  // Keep track of the timestamp of the first message in the log.
  int64_t u_init_time = 0;
  const lcm::LogEvent* first_event = log_file.readNextEvent();
  if (first_event != nullptr) {
      u_init_time = first_event->timestamp;
      std::cout<<"Initial event timestamp (s): "<<u_init_time/1e6 << std::endl;
  } else {
      std::cerr<<"Error: No events in the log!" << std::endl;
      return 1;  // Exit if no events are found
  }

  // Now seek to the time we want to start processing the log. That is,
  // time_into_log_in_microseconds offset from the first message in the log.
  log_file.seekToTimestamp(time_into_log_in_microsecs + u_init_time);

  const lcm::LogEvent* event;

  // Read the log until the first C3_ACTUAL message after the start time.
  // Prepare to grab all desired message information.
  std::set<std::string> channels_to_check = {
    "C3_ACTUAL", "C3_TARGET", "C3_FINAL_TARGET",
    "DYNAMICALLY_FEASIBLE_CURR_PLAN", "DYNAMICALLY_FEASIBLE_CURR_ACTOR_PLAN",
    "C3_DEBUG_CURR", "IS_C3_MODE", "SAMPLE_LOCATIONS", "SAMPLE_COSTS", "DYNAMICALLY_FEASIBLE_BEST_PLAN", "DYNAMICALLY_FEASIBLE_BEST_ACTOR_PLAN"};

  Eigen::VectorXd x_lcs_actual = Eigen::VectorXd::Zero(19);
  Eigen::VectorXd x_lcs_desired = Eigen::VectorXd::Zero(19);
  Eigen::VectorXd x_lcs_final_desired = Eigen::VectorXd::Zero(19);
  Eigen::MatrixXd dyn_feas_curr_plan_obj_pos = Eigen::MatrixXd::Zero(3, c3_options.N+1);
  Eigen::MatrixXd dyn_feas_curr_plan_ee_pos = Eigen::MatrixXd::Zero(3, c3_options.N+1);
  Eigen::MatrixXd dyn_feas_curr_plan_obj_orientation = Eigen::MatrixXd::Zero(4, c3_options.N+1);
  Eigen::MatrixXd dyn_feas_best_plan_obj_pos = Eigen::MatrixXd::Zero(3, c3_options.N+1);
  Eigen::MatrixXd dyn_feas_best_plan_ee_pos = Eigen::MatrixXd::Zero(3, c3_options.N+1);
  Eigen::MatrixXd dyn_feas_best_plan_obj_orientation = Eigen::MatrixXd::Zero(4, c3_options.N+1);

  Eigen::MatrixXd u_sol = Eigen::MatrixXd::Zero(3, c3_options.N);
  Eigen::MatrixXd x_sol = Eigen::MatrixXd::Zero(19, c3_options.N);
  Eigen::MatrixXd lambda_sol = Eigen::MatrixXd::Zero(16, c3_options.N);
  Eigen::MatrixXd w_sol = Eigen::MatrixXd::Zero(38, c3_options.N);
  Eigen::MatrixXd delta_sol = Eigen::MatrixXd::Zero(38, c3_options.N);

  // Collect the sample locations
  std::vector<Eigen::VectorXd> sample_locations_in_log;
  std::vector<Eigen::VectorXd> sample_costs_in_log;

  bool is_c3_mode = false;
  bool is_c3_mode_set = false;

  while ((event = log_file.readNextEvent()) != nullptr) {
    // Offset the time stamp by the initial time for better readability.
    int64_t adjusted_utimestamp = event->timestamp - u_init_time;

    if (event->channel == "C3_ACTUAL") {
      if(event->timestamp > time_into_log_in_microsecs + u_init_time) {
        dairlib::lcmt_c3_state message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received C3_ACTUAL message in seconds utime: " <<
            (message.utime)/1e6 << " and event timestamp " <<
            adjusted_utimestamp/1e6 << std::endl;
          for (int i=0; i<19; i++) {
            x_lcs_actual(i) = static_cast<double>(message.state[i]);
          }
        } else {
          std::cerr << "Failed to decode C3_ACTUAL message" << std::endl;
        }
      }
    }
    else if (event->channel == "C3_TARGET") {
      if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
        dairlib::lcmt_c3_state message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received C3_TARGET message in seconds utime: " <<
            (message.utime)/1e6 << " and event timestamp " <<
            adjusted_utimestamp/1e6 << std::endl;
          for (int i=0; i<19; i++) {
            x_lcs_desired(i) = static_cast<double>(message.state[i]);
          }
        } else {
          std::cerr << "Failed to decode C3_TARGET message" << std::endl;
        }
      }
    }
    else if (event->channel == "C3_FINAL_TARGET") {
      if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
        dairlib::lcmt_c3_state message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received C3_FINAL_TARGET message in seconds utime: " <<
            (message.utime)/1e6 << " and event timestamp " <<
            adjusted_utimestamp/1e6 << std::endl;
          for (int i=0; i<19; i++) {
            x_lcs_final_desired(i) = static_cast<double>(message.state[i]);
          }
        } else {
          std::cerr << "Failed to decode C3_FINAL_TARGET message" << std::endl;
        }
      }
    }
    else if (event->channel == "DYNAMICALLY_FEASIBLE_CURR_PLAN") {
      if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
        dairlib::lcmt_timestamped_saved_traj message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received DYNAMICALLY_FEASIBLE_CURR_PLAN message in " <<
            "seconds utime: " << (message.utime)/1e6 << " and event " <<
            "timestamp " << adjusted_utimestamp/1e6 << std::endl;
          for (int i=0; i<4; i++) {
            for (int j=0; j<c3_options.N+1; j++) {
              dyn_feas_curr_plan_obj_orientation(i,j) =
                message.saved_traj.trajectories[0].datapoints[i][j];
            }
          }
          for (int i=0; i<3; i++) {
            for (int j=0; j<c3_options.N+1; j++) {
              dyn_feas_curr_plan_obj_pos(i,j) =
                message.saved_traj.trajectories[1].datapoints[i][j];
            }
          }
        } else {
          std::cerr << "Failed to decode DYNAMICALLY_FEASIBLE_CURR_PLAN " <<
            "message" << std::endl;
        }
      }
    }
    else if (event->channel == "DYNAMICALLY_FEASIBLE_CURR_ACTOR_PLAN"){
      if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
        dairlib::lcmt_timestamped_saved_traj message;
        if (message.decode(event->data, 0, event->datalen) > 0){
            std::cout << "Received DYNAMICALLY_FEASIBLE_CURR_ACTOR_PLAN " <<
              "message in seconds utime: " << (message.utime)/1e6 <<
              " and event timestamp " << adjusted_utimestamp/1e6 << std::endl;
          for (int i = 0; i < 3; i++) {
            for (int j = 0; j < c3_options.N+1; j++) {
              dyn_feas_curr_plan_ee_pos(i,j) =
                message.saved_traj.trajectories[0].datapoints[i][j];
            }
          }
        } else {
          std::cerr << "Failed to decode DYNAMICALLY_FEASIBLE_CURR_ACTOR_PLAN"
            << " message" << std::endl;
        }
      }
    }
    else if (event->channel == "DYNAMICALLY_FEASIBLE_BEST_PLAN"){
      if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
        dairlib::lcmt_timestamped_saved_traj message;
        if (message.decode(event->data, 0, event->datalen) > 0){
            std::cout << "Received DYNAMICALLY_FEASIBLE_BEST_PLAN " <<
              "message in seconds utime: " << (message.utime)/1e6 <<
              " and event timestamp " << adjusted_utimestamp/1e6 << std::endl;
          for (int i = 0; i < 4; i++) {
            for (int j = 0; j < c3_options.N+1; j++) {
              dyn_feas_best_plan_obj_orientation(i,j) =
                message.saved_traj.trajectories[0].datapoints[i][j];
            }
          }
          for (int i = 0; i < 3; i++) {
            for (int j = 0; j < c3_options.N+1; j++) {
              dyn_feas_best_plan_obj_pos(i,j) =
                message.saved_traj.trajectories[1].datapoints[i][j];
            }
          }
        } else {
          std::cerr << "Failed to decode DYNAMICALLY_FEASIBLE_BEST_PLAN" <<
            " message" << std::endl;
        }
      }
    }
    else if (event->channel == "DYNAMICALLY_FEASIBLE_BEST_ACTOR_PLAN"){
      if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
        dairlib::lcmt_timestamped_saved_traj message;
        if (message.decode(event->data, 0, event->datalen) > 0){
            std::cout << "Received DYNAMICALLY_FEASIBLE_BEST_ACTOR_PLAN " <<
              "message in seconds utime: " << (message.utime)/1e6 <<
              " and event timestamp " << adjusted_utimestamp/1e6 << std::endl;
          for (int i = 0; i < 3; i++) {
            for (int j = 0; j < c3_options.N+1; j++) {
              dyn_feas_best_plan_ee_pos(i,j) =
                message.saved_traj.trajectories[0].datapoints[i][j];
            }
          }
        } else {
          std::cerr << "Failed to decode DYNAMICALLY_FEASIBLE_BEST_ACTOR_PLAN"
            << " message" << std::endl;
        }
      }
    }
    else if (event->channel == "C3_DEBUG_CURR") {
      if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
        dairlib::lcmt_c3_output message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received C3_DEBUG_CURR message in seconds utime: " <<
            (message.utime)/1e6 << " and event timestamp " <<
            adjusted_utimestamp/1e6 << std::endl;
          for (int i = 0; i < 3; i++) {
            for (int j = 0; j < c3_options.N; j++) {
              u_sol(i,j) = static_cast<double>(message.c3_solution.u_sol[i][j]);
            }
          }
          for(int i = 0; i < 19; i++){
            for(int j = 0; j < c3_options.N; j++){
              x_sol(i,j) = static_cast<double>(message.c3_solution.x_sol[i][j]);
            }
          }
          for(int i = 0; i < 16; i++){
            for(int j = 0; j < c3_options.N; j++){
              lambda_sol(i,j) = static_cast<double>(message.c3_solution.lambda_sol[i][j]);
            }
          }
          for(int i = 0; i < 38; i++){
            for(int j = 0; j < c3_options.N; j++){
              w_sol(i,j) = static_cast<double>(message.c3_intermediates.w_sol[i][j]);
            }
          }
          for(int i = 0; i < 38; i++){
            for(int j = 0; j < c3_options.N; j++){
              delta_sol(i,j) = static_cast<double>(message.c3_intermediates.delta_sol[i][j]);
            }
          }
        } else {
          std::cerr << "Failed to decode C3_DEBUG_CURR message" << std::endl;
        }
      }
    }
    else if (event->channel == "IS_C3_MODE") {
      if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
        dairlib::lcmt_timestamped_saved_traj message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received IS_C3_MODE message in " <<
            "seconds utime: " << (message.utime)/1e6 << " and event " <<
            "timestamp " << adjusted_utimestamp/1e6 << std::endl;
          is_c3_mode = message.saved_traj.trajectories[0].datapoints[0][0];
          is_c3_mode_set = true;
        } else {
          std::cerr << "Failed to decode IS_C3_MODE message" << std::endl;
        }
      }
    }
    else if (event->channel == "SAMPLE_LOCATIONS") {
      if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
        dairlib::lcmt_timestamped_saved_traj message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received SAMPLE_LOCATIONS message in " <<
            "seconds utime: " << (message.utime)/1e6 << " and event " <<
            "timestamp " << adjusted_utimestamp/1e6 << std::endl;
          for (int i = 0; i < message.saved_traj.trajectories[0].datapoints[0].size(); i++) {
            Eigen::VectorXd sample_location = Eigen::VectorXd::Zero(3);
            for (int j = 0; j < 3; j++) {
              sample_location(j) = message.saved_traj.trajectories[0].datapoints[j][i];
            }
            sample_locations_in_log.push_back(sample_location);
          }
        } else {
          std::cerr << "Failed to decode SAMPLE_LOCATIONS message" << std::endl;
        }
      }
    }
    else if (event->channel == "SAMPLE_COSTS") {
      if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
        dairlib::lcmt_timestamped_saved_traj message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received SAMPLE_COSTS message in " <<
            "seconds utime: " << (message.utime)/1e6 << " and event " <<
            "timestamp " << adjusted_utimestamp/1e6 << std::endl;
          for (int i = 0; i < message.saved_traj.trajectories[0].datapoints[0].size(); i++) {
            sample_costs_in_log.push_back(
              Eigen::VectorXd::Constant(1, message.saved_traj.trajectories[0].datapoints[0][i]));
          }
        } else {
          std::cerr << "Failed to decode SAMPLE_COSTS message" << std::endl;
        }
      }
    }

    // Break out of loop if we have one message for every desired channel.
    if (channels_to_check.find(event->channel) != channels_to_check.end()) {
      if ((x_lcs_actual != Eigen::VectorXd::Zero(19)) &&
          (x_lcs_desired != Eigen::VectorXd::Zero(19)) &&
          (x_lcs_final_desired != Eigen::VectorXd::Zero(19)) &&
          (dyn_feas_curr_plan_ee_pos != Eigen::MatrixXd::Zero(3, c3_options.N+1)) &&
          (dyn_feas_curr_plan_obj_pos != Eigen::MatrixXd::Zero(3, c3_options.N+1)) &&
          (dyn_feas_curr_plan_obj_orientation != Eigen::MatrixXd::Zero(4, c3_options.N+1)) &&
          (u_sol != Eigen::MatrixXd::Zero(3, c3_options.N)) &&
          (is_c3_mode_set) &&
          (sample_locations_in_log.size() > 0) &&
          (sample_costs_in_log.size() > 0)) {
        break;
      }
    }
  }

  std::cout<<"x_lcs_actual from log : "<<x_lcs_actual.transpose()<<"\n"<<std::endl;
  std::cout<<"Available sample locations from log and corresponding costs: " <<std::endl;
  std::cout<<"Number of sample locations in log: "<<sample_locations_in_log.size()<<std::endl;
  for (int i = 0; i < sample_locations_in_log.size(); i++) {
    if (i < sample_costs_in_log.size()) {
      std::cout <<"sample "<<i<<" : " << sample_locations_in_log[i].transpose() << " cost is "<< sample_costs_in_log[i]<<std::endl;
      if(i == 0){
        std::cout<<"Dynamically feasible plan is: "<<std::endl;
        std::cout<<"End effector position: "<<dyn_feas_curr_plan_ee_pos.transpose()<<std::endl;
        std::cout<<"Object orientation: "<<dyn_feas_curr_plan_obj_orientation.transpose()<<std::endl;
        std::cout<<"Object position: "<<dyn_feas_curr_plan_obj_pos.transpose()<<std::endl;
      }
      if(i == 1){
        std::cout<<"Dynamically feasible best plan is: "<<std::endl;
        std::cout<<"End effector position: "<<dyn_feas_best_plan_ee_pos.transpose()<<std::endl;
        std::cout<<"Object orientation: "<<dyn_feas_best_plan_obj_orientation.transpose()<<std::endl;
        std::cout<<"Object position: "<<dyn_feas_best_plan_obj_pos.transpose()<<std::endl;
      }
    } else {
      std::cout <<"sample "<<i<<" : " << sample_locations_in_log[i].transpose() << " cost is "<< "N/A"<<std::endl;
    }
  }

  std::cout << "\nFuture timestamps:" << std::endl;
  int more_msgs_to_print = 5;
  while(((event = log_file.readNextEvent()) != nullptr) &&
        (more_msgs_to_print > 0)) {
    if (event->channel == "C3_ACTUAL") {
      std::cout << "Received C3_ACTUAL message in seconds: " <<
        (event->timestamp - u_init_time)/1e6<< std::endl;
      more_msgs_to_print--;
    }
  }
  std::cout<<""<<std::endl;

  // Create the plant for the LCS.
  DiagramBuilder<double> plant_builder;
  auto [plant_for_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_builder, 0.0);

  Parser parser_for_lcs(&plant_for_lcs);
  parser_for_lcs.SetAutoRenaming(true);

  // Load simple model of end effector (just a sphere) for the lcs plant.
  parser_for_lcs.AddModels(controller_params.end_effector_simple_model);
  parser_for_lcs.AddModels(controller_params.jack_model);
  parser_for_lcs.AddModels(controller_params.ground_model);	
	RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  Eigen::Vector3d p_world_to_ground = sim_params.p_world_to_franka +
                                      sim_params.p_franka_to_ground;
  RigidTransform<double> X_W_G =
      RigidTransform<double>(drake::math::RotationMatrix<double>(),
                             p_world_to_ground);
  plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
                           plant_for_lcs.GetFrameByName("base_link"), X_WI);
  plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
                           plant_for_lcs.GetFrameByName("ground"),
                           X_W_G);
  plant_for_lcs.Finalize();
  std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_for_lcs_autodiff =
      drake::systems::System<double>::ToAutoDiffXd(plant_for_lcs);

  auto plant_diagram = plant_builder.Build();
  std::unique_ptr<drake::systems::Context<double>> diagram_context =
      plant_diagram->CreateDefaultContext();
  auto& plant_for_lcs_context = plant_diagram->GetMutableSubsystemContext(
      plant_for_lcs, diagram_context.get());
  auto plant_for_lcs_context_ad = plant_for_lcs_autodiff->CreateDefaultContext();


  std::vector<std::vector<SortedPair<GeometryId>>>
  contact_pairs;  // Exact list depends on c3_options.num_contacts_index,
                  // but e.g. num_contacts_index = 0 means this will have
                  // [[(ee,cap1), (ee,cap2), (ee_cap3)],
                  //  [(ground,cap1sph1), (ground,cap1sph2),
                  //   (ground,cap2sph1), (ground,cap2sph2),
                  //   (ground,cap3sph1), (ground,cap3sph2)]]
                  // and the first list (of 3) will get resolved to a single
                  // ee-jack contact, and the second list (of 6) will get
                  // resolved to 3 ground-jack contacts.

  //   Creating a map of contact geoms
  std::unordered_map<std::string, drake::geometry::GeometryId> contact_geoms;

  //   Change contact geoms based on the demo_name
  if(FLAGS_demo_name == "push_t") {

      drake::geometry::GeometryId ee_contact_points =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("end_effector_simple"))[0];
      drake::geometry::GeometryId horizontal_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("horizontal_link"))[0];
      drake::geometry::GeometryId vertical_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("vertical_link"))[0];

      drake::geometry::GeometryId corner_nxynz_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("corner_nxynz"))[0];
      drake::geometry::GeometryId corner_nxnynz_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("corner_nxnynz"))[0];
      drake::geometry::GeometryId corner_xynz_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("corner_xynz"))[0];

      drake::geometry::GeometryId ground_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("ground"))[0];

      //   Creating a map of contact geoms
      contact_geoms["EE"] = ee_contact_points;
      contact_geoms["horizontal_link"] = horizontal_geoms;
      contact_geoms["vertical_link"] = vertical_geoms;
      contact_geoms["corner_nxynz"] = corner_nxynz_geoms;
      contact_geoms["corner_nxnynz"] = corner_nxnynz_geoms;
      contact_geoms["corner_xynz"] = corner_xynz_geoms;
      contact_geoms["GROUND"] = ground_geoms;

      std::vector<SortedPair<GeometryId>> ee_contact_pairs;

      //   Creating a list of contact pairs for the end effector and the jack to
      //   hand over to lcs factory in the controller to resolve
      ee_contact_pairs.push_back(
          SortedPair(contact_geoms["EE"], contact_geoms["horizontal_link"]));
      ee_contact_pairs.push_back(
          SortedPair(contact_geoms["EE"], contact_geoms["vertical_link"]));

      //   Creating a list of contact pairs for the jack and the ground
          SortedPair<GeometryId> ground_contact_1{
          SortedPair(contact_geoms["corner_nxynz"], contact_geoms["GROUND"])};
          SortedPair<GeometryId> ground_contact_2{
          SortedPair(contact_geoms["corner_nxnynz"], contact_geoms["GROUND"])};
          SortedPair<GeometryId> ground_contact_3{
          SortedPair(contact_geoms["corner_xynz"], contact_geoms["GROUND"])};

      contact_pairs.push_back(ee_contact_pairs);

      if(sampling_c3_options.num_contacts_index == 2 || sampling_c3_options.num_contacts_index == 3){
          // If num_contacts_index is 2 or 3, we add an additional contact pair 
          // between the end effector and the ground.
          std::vector<SortedPair<GeometryId>> ee_ground_contact{
          SortedPair(contact_geoms["EE"], contact_geoms["GROUND"])};
          contact_pairs.push_back(ee_ground_contact);
      }
      std::vector<SortedPair<GeometryId>> ground_object_contact_pairs;
      ground_object_contact_pairs.push_back(ground_contact_1);
      ground_object_contact_pairs.push_back(ground_contact_2);
      ground_object_contact_pairs.push_back(ground_contact_3);
      contact_pairs.push_back(ground_object_contact_pairs);
  }
  else if(FLAGS_demo_name == "box_topple") {
      drake::geometry::GeometryId ee_contact_points =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("end_effector_simple"))[0];
      drake::geometry::GeometryId box_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("box"))[0];

      drake::geometry::GeometryId corner_xynz_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("corner_xynz"))[0];
      drake::geometry::GeometryId corner_xnynz_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("corner_xnynz"))[0];
      drake::geometry::GeometryId corner_nxynz_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("corner_nxynz"))[0];
      drake::geometry::GeometryId corner_nxnynz_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("corner_nxnynz"))[0];
      drake::geometry::GeometryId corner_xyz_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("corner_xyz"))[0];
      drake::geometry::GeometryId corner_xnyz_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("corner_xnyz"))[0];
      drake::geometry::GeometryId corner_nxyz_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("corner_nxyz"))[0];
      drake::geometry::GeometryId corner_nxnyz_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("corner_nxnyz"))[0];

      drake::geometry::GeometryId ground_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("ground"))[0];

      //   Creating a map of contact geoms
      // std::unordered_map<std::string, drake::geometry::GeometryId> contact_geoms;
      contact_geoms["EE"] = ee_contact_points;
      contact_geoms["box"] = box_geoms;
      contact_geoms["corner_xynz"] = corner_xynz_geoms;
      contact_geoms["corner_xnynz"] = corner_xnynz_geoms;
      contact_geoms["corner_nxynz"] = corner_nxynz_geoms;
      contact_geoms["corner_nxnynz"] = corner_nxnynz_geoms;
      contact_geoms["corner_xyz"] = corner_xyz_geoms;
      contact_geoms["corner_xnyz"] = corner_xnyz_geoms;
      contact_geoms["corner_nxyz"] = corner_nxyz_geoms;
      contact_geoms["corner_nxnyz"] = corner_nxnyz_geoms;
      contact_geoms["GROUND"] = ground_geoms;

      std::vector<SortedPair<GeometryId>> ee_contact_pairs;

      //   Creating a list of contact pairs for the end effector and the jack to
      //   hand over to lcs factory in the controller to resolve
      ee_contact_pairs.push_back(
          SortedPair(contact_geoms["EE"], contact_geoms["box"]));

      //   Creating a list of contact pairs for the jack and the ground
          SortedPair<GeometryId> ground_contact_1{
          SortedPair(contact_geoms["corner_xynz"], contact_geoms["GROUND"])};
          SortedPair<GeometryId> ground_contact_2{
          SortedPair(contact_geoms["corner_xnynz"], contact_geoms["GROUND"])};
          SortedPair<GeometryId> ground_contact_3{
          SortedPair(contact_geoms["corner_nxynz"], contact_geoms["GROUND"])};
          SortedPair<GeometryId> ground_contact_4{
          SortedPair(contact_geoms["corner_nxnynz"], contact_geoms["GROUND"])};
          SortedPair<GeometryId> ground_contact_5{
          SortedPair(contact_geoms["corner_xyz"], contact_geoms["GROUND"])};
          SortedPair<GeometryId> ground_contact_6{
          SortedPair(contact_geoms["corner_xnyz"], contact_geoms["GROUND"])};
          SortedPair<GeometryId> ground_contact_7{
          SortedPair(contact_geoms["corner_nxyz"], contact_geoms["GROUND"])};
          SortedPair<GeometryId> ground_contact_8{
          SortedPair(contact_geoms["corner_nxnyz"], contact_geoms["GROUND"])};

      // std::vector<std::vector<SortedPair<GeometryId>>>
      //     contact_pairs;  // Exact list depends on c3_options.num_contacts_index,
                          // but e.g. num_contacts_index = 0 means this will have
                          // [[(ee,cap1), (ee,cap2), (ee_cap3)],
                          //  [(ground,cap1sph1), (ground,cap1sph2),
                          //   (ground,cap2sph1), (ground,cap2sph2),
                          //   (ground,cap3sph1), (ground,cap3sph2)]]
                          // and the first list (of 3) will get resolved to a single
                          // ee-jack contact, and the second list (of 6) will get
                          // resolved to 3 ground-jack contacts.
      contact_pairs.push_back(ee_contact_pairs);

      if(sampling_c3_options.num_contacts_index == 2 || sampling_c3_options.num_contacts_index == 3){
          // If num_contacts_index is 2 or 3, we add an additional contact pair 
          // between the end effector and the ground.
          std::vector<SortedPair<GeometryId>> ee_ground_contact{
          SortedPair(contact_geoms["EE"], contact_geoms["GROUND"])};
          contact_pairs.push_back(ee_ground_contact);
      }
      std::vector<SortedPair<GeometryId>> ground_object_contact_pairs;
      ground_object_contact_pairs.push_back(ground_contact_1);
      ground_object_contact_pairs.push_back(ground_contact_2);
      ground_object_contact_pairs.push_back(ground_contact_3);
      ground_object_contact_pairs.push_back(ground_contact_4);
      ground_object_contact_pairs.push_back(ground_contact_5);
      ground_object_contact_pairs.push_back(ground_contact_6);
      ground_object_contact_pairs.push_back(ground_contact_7);
      ground_object_contact_pairs.push_back(ground_contact_8);
      contact_pairs.push_back(ground_object_contact_pairs);
  }
  else if(FLAGS_demo_name == "jacktoy") {
          drake::geometry::GeometryId ee_contact_points =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("end_effector_simple"))[0];
      drake::geometry::GeometryId capsule1_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("capsule_1"))[0];
      drake::geometry::GeometryId capsule2_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("capsule_2"))[0];
      drake::geometry::GeometryId capsule3_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("capsule_3"))[0];

      drake::geometry::GeometryId capsule1_sphere1_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("capsule_1"))[1];
      drake::geometry::GeometryId capsule1_sphere2_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("capsule_1"))[2];
      drake::geometry::GeometryId capsule2_sphere1_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("capsule_2"))[1];
      drake::geometry::GeometryId capsule2_sphere2_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("capsule_2"))[2];
      drake::geometry::GeometryId capsule3_sphere1_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("capsule_3"))[1];
      drake::geometry::GeometryId capsule3_sphere2_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("capsule_3"))[2];

      drake::geometry::GeometryId ground_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("ground"))[0];

      //   Creating a map of contact geoms
      // std::unordered_map<std::string, drake::geometry::GeometryId> contact_geoms;
      contact_geoms["EE"] = ee_contact_points;
      contact_geoms["CAPSULE_1"] = capsule1_geoms;
      contact_geoms["CAPSULE_2"] = capsule2_geoms;
      contact_geoms["CAPSULE_3"] = capsule3_geoms;
      contact_geoms["CAPSULE_1_SPHERE_1"] = capsule1_sphere1_geoms;
      contact_geoms["CAPSULE_1_SPHERE_2"] = capsule1_sphere2_geoms;
      contact_geoms["CAPSULE_2_SPHERE_1"] = capsule2_sphere1_geoms;
      contact_geoms["CAPSULE_2_SPHERE_2"] = capsule2_sphere2_geoms;
      contact_geoms["CAPSULE_3_SPHERE_1"] = capsule3_sphere1_geoms;
      contact_geoms["CAPSULE_3_SPHERE_2"] = capsule3_sphere2_geoms;
      contact_geoms["GROUND"] = ground_geoms;

      std::vector<SortedPair<GeometryId>> ee_contact_pairs;

      //   Creating a list of contact pairs for the end effector and the jack to
      //   hand over to lcs factory in the controller to resolve
      ee_contact_pairs.push_back(
          SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_1"]));
      ee_contact_pairs.push_back(
          SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_2"]));
      ee_contact_pairs.push_back(
          SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_3"]));

      //   Creating a list of contact pairs for the jack and the ground
      SortedPair<GeometryId> ground_contact_1_1{
          SortedPair(contact_geoms["CAPSULE_1_SPHERE_1"], contact_geoms["GROUND"])};
      SortedPair<GeometryId> ground_contact_1_2{
          SortedPair(contact_geoms["CAPSULE_1_SPHERE_2"], contact_geoms["GROUND"])};
      SortedPair<GeometryId> ground_contact_2_1{
          SortedPair(contact_geoms["CAPSULE_2_SPHERE_1"], contact_geoms["GROUND"])};
      SortedPair<GeometryId> ground_contact_2_2{
          SortedPair(contact_geoms["CAPSULE_2_SPHERE_2"], contact_geoms["GROUND"])};
      SortedPair<GeometryId> ground_contact_3_1{
          SortedPair(contact_geoms["CAPSULE_3_SPHERE_1"], contact_geoms["GROUND"])};
      SortedPair<GeometryId> ground_contact_3_2{
          SortedPair(contact_geoms["CAPSULE_3_SPHERE_2"], contact_geoms["GROUND"])};

      // std::vector<std::vector<SortedPair<GeometryId>>>
      //     contact_pairs;  // Exact list depends on c3_options.num_contacts_index,
                          // but e.g. num_contacts_index = 0 means this will have
                          // [[(ee,cap1), (ee,cap2), (ee_cap3)],
                          //  [(ground,cap1sph1), (ground,cap1sph2),
                          //   (ground,cap2sph1), (ground,cap2sph2),
                          //   (ground,cap3sph1), (ground,cap3sph2)]]
                          // and the first list (of 3) will get resolved to a single
                          // ee-jack contact, and the second list (of 6) will get
                          // resolved to 3 ground-jack contacts.
      contact_pairs.push_back(ee_contact_pairs);

      if(sampling_c3_options.num_contacts_index == 2 || sampling_c3_options.num_contacts_index == 3){
      // If num_contacts_index is 2 or 3, we add an additional contact pair 
      // between the end effector and the ground.
      std::vector<SortedPair<GeometryId>> ee_ground_contact{
          SortedPair(contact_geoms["EE"], contact_geoms["GROUND"])};
      contact_pairs.push_back(ee_ground_contact);
      }
      std::vector<SortedPair<GeometryId>> ground_object_contact_pairs;
      ground_object_contact_pairs.push_back(ground_contact_1_1);
      ground_object_contact_pairs.push_back(ground_contact_1_2);
      ground_object_contact_pairs.push_back(ground_contact_2_1);
      ground_object_contact_pairs.push_back(ground_contact_2_2);
      ground_object_contact_pairs.push_back(ground_contact_3_1);
      ground_object_contact_pairs.push_back(ground_contact_3_2);
      contact_pairs.push_back(ground_object_contact_pairs);
  }
  else if(FLAGS_demo_name == "ball_rolling"){
      drake::geometry::GeometryId ee_contact_points =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("end_effector_simple"))[0];
      drake::geometry::GeometryId object_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("sphere"))[0];
      drake::geometry::GeometryId ground_geoms =
          plant_for_lcs.GetCollisionGeometriesForBody(
              plant_for_lcs.GetBodyByName("ground"))[0];

      //   Creating a map of contact geoms
      contact_geoms["EE"] = ee_contact_points;
      contact_geoms["SPHERE"] = object_geoms;
      contact_geoms["GROUND"] = ground_geoms;

      std::vector<SortedPair<GeometryId>> ee_contact_pairs;

      //   Creating a list of contact pairs for the end effector and the jack to
      //   hand over to lcs factory in the controller to resolve
      ee_contact_pairs.push_back(
          SortedPair(contact_geoms["EE"], contact_geoms["SPHERE"]));
      //   Creating a list of contact pairs for the jack and the ground
      std::vector<SortedPair<GeometryId>> ground_contact{
          SortedPair(contact_geoms["SPHERE"], contact_geoms["GROUND"])};

      // will have [[(ee,sphere)], [(sphere, ground)]]
      contact_pairs.push_back(ee_contact_pairs);

      if (sampling_c3_options.num_contacts_index == 1){
          // If num_contacts_index is 2 or 3, we add an additional contact pair 
          // between the end effector and the ground.
          std::vector<SortedPair<GeometryId>> ee_ground_contact{
          SortedPair(contact_geoms["EE"], contact_geoms["GROUND"])};
          contact_pairs.push_back(ee_ground_contact);
      }
      contact_pairs.push_back(ground_contact);
  }

  plant_diagram->set_name(("franka_c3_plant"));
  DrawAndSaveDiagramGraph(
    *plant_diagram, base_path + "test/franka_c3_plant_in_log_loader");

  // Create the controller.  The last bool argument is the verbose flag.
  bool verbose = true;
  #ifdef DO_SAMPLE_VISUALIZATIONS
    verbose = false;
    c3_options.use_predicted_x0_c3 = false;
  #endif
  DiagramBuilder<double> builder;
  auto controller = builder.AddSystem<dairlib::systems::SamplingC3Controller>(
    plant_for_lcs, &plant_for_lcs_context, *plant_for_lcs_autodiff,
    plant_for_lcs_context_ad.get(), contact_pairs, c3_options,
    sampling_c3_options,
    sampling_params, verbose);
  auto controller_context = controller->CreateDefaultContext();

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_c3_controller"));

  DrawAndSaveDiagramGraph(
    *owned_diagram, base_path + "test/franka_c3_controller_in_log_loader");

  // Fix static input ports.
  controller->get_input_port_radio().FixValue(
    controller_context.get(), drake::Value<dairlib::lcmt_radio_out>{});
  controller->get_input_port_target().FixValue(
    controller_context.get(), x_lcs_desired);
  controller->get_input_port_final_target().FixValue(
    controller_context.get(), x_lcs_final_desired);

  std::vector<Eigen::VectorXd> x_lcs_actuals;
  // x_lcs_actuals.push_back(x_lcs_actual);
  
  // Testing sample location 0 and sample location 1 from the log from all sample locations.
  // This was done to verify the anolmaly seen as a result of using cost type 3 in the push_T case.
  // Eigen::VectorXd curr_sample_in_log = x_lcs_actual;
  // curr_sample_in_log.head(3) = sample_locations_in_log[0];
  // x_lcs_actuals.push_back(curr_sample_in_log);
  Eigen::VectorXd best_sample_in_log = x_lcs_actual;
  best_sample_in_log.head(3) = sample_locations_in_log[1];
  x_lcs_actuals.push_back(best_sample_in_log);

  #ifdef DO_SAMPLE_VISUALIZATIONS
    std::cout<<"DO_SAMPLE_VISUALIZATIONS"<<std::endl;

    std::vector<Eigen::VectorXd> x_lcs_samples;
    std::vector<Eigen::Vector2d> angles;


    // Check the demo name and call the corresponding function directly.
    if (FLAGS_demo_name == "push_t" || FLAGS_demo_name == "box_topple") {
      // Directly call the GenerateEvenlySpacedSamplesAroundT function
      x_lcs_samples = GenerateEvenlySpacedSamplesAroundT(x_lcs_actual, sampling_params, N_VERTICAL, N_HORIZONTAL);
    } else if (FLAGS_demo_name == "jacktoy" || FLAGS_demo_name == "ball_rolling") {
        // Directly call the GenerateEvenlySpacedSamplesAroundJack function
        auto [samples, angle_data] = GenerateEvenlySpacedSamplesAroundJack(x_lcs_actual, sampling_params, N_VERTICAL, N_HORIZONTAL);
        x_lcs_samples = samples;
        angles = angle_data;
    }

    x_lcs_actuals.clear();
    x_lcs_actuals = x_lcs_samples;

    // Prepare to print out the costs.
    Eigen::VectorXd costs;

    std::cout << "DO_SAMPLE_VISUALIZATIONS" << std::endl;
    if (FLAGS_demo_name == "jacktoy" || FLAGS_demo_name == "ball_rolling") {
      costs = Eigen::VectorXd::Zero(angles.size());  // If angles are available
    } else if (FLAGS_demo_name == "push_t" || FLAGS_demo_name == "box_topple") {
        costs = Eigen::VectorXd::Zero(x_lcs_samples.size());  // For the other case
    }
  #endif

  // Remember that doing this in a loop actually triggers the x_pred calculation
  // in the controller. Ensure this is something you want to do. 
  for (int i = 0; i < x_lcs_actuals.size(); i++) {
    Eigen::VectorXd x_lcs_actual_i = x_lcs_actuals[i];

    // Set plant states, fix input port values.
    plant_for_lcs.SetPositionsAndVelocities(
      &plant_for_lcs_context, x_lcs_actual_i);
    controller->get_input_port_lcs_state().FixValue(
      controller_context.get(), TimestampedVector<double>(x_lcs_actual_i));

    // Do forced publish.
    auto discrete_state = controller->AllocateDiscreteVariables();
    controller->CalcForcedDiscreteVariableUpdate(
      *controller_context, discrete_state.get());
    auto sample_costs_inside = controller->get_output_port_all_sample_costs(
        ).Eval<std::vector<double>>(*controller_context);
    controller->ForcedPublish(*controller_context);

    #ifdef DO_SAMPLE_VISUALIZATIONS
      // Get the cost of the sample by querying the sample costs output port.
      auto sample_costs = controller->get_output_port_all_sample_costs(
        ).Eval<std::vector<double>>(*controller_context);
      costs(i) = sample_costs[0];
      if (i % 10 == 0) {
        std::cout << "Finished " << i << " of " << x_lcs_actuals.size() <<
          " samples." << std::endl;
      }
    #endif
  }
  #ifdef DO_SAMPLE_VISUALIZATIONS
    // Write values like positions and costs to files which can be loaded with
    // np.loadtxt() cleanly from a python script.  If the outputs are small
    // enough, can swap the file writing for printing to the console via
    // PythonFriendlyVectorOfVectorXdPrint.
    PythonFriendlyVectorOfVectorXdToFile("x_lcs_samples", x_lcs_samples);
    PythonFriendlyVectorOfVectorXdToFile("costs", {costs});
    PythonFriendlyVectorOfVectorXdToFile("x_lcs_desired", {x_lcs_desired});
    PythonFriendlyVectorOfVectorXdToFile(
      "p_world_to_franka", {sim_params.p_world_to_franka});
    PythonFriendlyVectorOfVectorXdToFile(
      "p_franka_to_ground", {sim_params.p_franka_to_ground});

    std::cout << "\nee_urdf = op.join(DAIRLIB_DIR, '" <<
      controller_params.end_effector_simple_model << "')" << std::endl;
    std::cout << "jack_urdf = op.join(DAIRLIB_DIR, '" <<
      sim_params.jack_model << "')" << std::endl;
  #endif

  std::cout << "Finished ForcedPublish" << std::endl;

  // Example of getting the C3 solution by evaluating the output port.
  // auto c3_solution = controller->get_output_port_c3_solution_curr_plan(
  //   ).Eval<C3Output::C3Solution>(*controller_context);
  // std::cout << "\nc3_solution.x_sol_ = " << c3_solution.x_sol_ << std::endl;
  // std::cout << "\nc3_solution.lambda_sol_ = " << c3_solution.lambda_sol_
  //   << std::endl;
  // std::cout << "\nc3_solution.u_sol_ = " << c3_solution.u_sol_ << std::endl;
  // std::cout << "\nc3_solution.time_vector_ = " << c3_solution.time_vector_
  //   << std::endl;

  return 0;
}


/* Define function that will generate samples around jack location.

  Args:
    x_lcs_actual
    sampling_params
    num_vertical
    num_horizontal

  Returns:
    - A std::vector of x_lcs samples
    - A std::vector of 2D (theta, elevation_theta) positions from which the
      samples were derived.
*/
std::vector<Eigen::VectorXd> GenerateEvenlySpacedSamplesAroundT(
    const Eigen::VectorXd& x_lcs,
    const SamplingC3SamplingParams& sampling_params,
    const int& num_vertical, const int& num_horizontal
  ) {
  
  // Extract sampling parameters.
  double sampling_height = 0.00;

  // the sampling region has an outer x and y limit. These are expressed in body frame.
  double outer_x_limit = 0.16;
  double outer_y_limit = 0.16;

  // Generate samples on a grid around the T
  std::vector<Eigen::VectorXd> x_lcs_samples;

  // Compute the step size for the grid.
  // num_vertical means how many rows of samples we want to take. That is x changes across each row.
  // num_horizontal means how many columns of samples we want to take. That is y changes across each column.
  double step_size_x = 2*outer_x_limit/(num_vertical-1);
  double step_size_y = 2*outer_y_limit/(num_horizontal-1);
  std::cout << "step_size_x: " << step_size_x << std::endl;
  std::cout << "step_size_y: " << step_size_y << std::endl;

  std::vector<Eigen::VectorXd> unfiltered_samples_in_body_frame;
  // Set the dummy to be the same as the actual lcs value.
  Eigen::VectorXd x_lcs_sample = x_lcs;
  std::cout << "x_lcs: " << x_lcs.transpose() << std::endl;
  Eigen::Quaterniond quat_object(x_lcs(3), x_lcs(4), x_lcs(5), x_lcs(6));
  Eigen::Vector3d object_position = x_lcs.segment(7, 3);

  // Generate uniform samples on a grid. Replace the end effector values in the dummy with the sample values.
  for (int i = 0; i < num_vertical; i++) {
    for (int j = 0; j < num_horizontal; j++) {
      x_lcs_sample(0) = -outer_x_limit + i*step_size_x;
      x_lcs_sample(1) = -outer_y_limit + j*step_size_y;
      x_lcs_sample(2) = sampling_height;
      unfiltered_samples_in_body_frame.push_back(x_lcs_sample);
    }
  }

  // Define limits for sample validity.
  double ee_radius = 0; //0.0145;
  double clearance = 0; //0.002;     // Maintain a clearance of 2mm from the T.
  double x_bottom_lim_1 = 0.08 + ee_radius + clearance;
  double x_top_lim_1 = -0.08 + ee_radius + clearance;
  double x_top_lim_2 = -0.08 - 0.04 - ee_radius - clearance;
  double y_lim_1 = 0.04 + ee_radius + clearance;
  double y_lim_2 = 0.08 + ee_radius + clearance;

  // Remove invalid samples.
  // std::vector<Eigen::VectorXd> filtered_samples_in_body_frame = remove_invalid_samples(
  //   unfiltered_samples_in_body_frame, x_bottom_lim_1, x_top_lim_1, x_top_lim_2,
  //   y_lim_1, y_lim_2);

  // Currently setting filtering off. 
  std::vector<Eigen::VectorXd> filtered_samples_in_body_frame = unfiltered_samples_in_body_frame;

  // Convert the samples to world frame.
  std::vector<Eigen::VectorXd> filtered_samples_in_world_frame;
  for (const auto& sample : filtered_samples_in_body_frame) {
    Eigen::VectorXd sample_in_world_frame = sample;
    // Convert end effector position to world frame.
    sample_in_world_frame.head(3) = quat_object*sample.head(3) + object_position;
    filtered_samples_in_world_frame.push_back(sample_in_world_frame);
  }

  return filtered_samples_in_world_frame;
}

/* Define function that will generate samples around jack location.

  Args:
    x_lcs_actual
    sampling_params
    num_vertical
    num_horizontal

  Returns:
    - A std::vector of x_lcs samples
    - A std::vector of 2D (theta, elevation_theta) positions from which the
      samples were derived.
*/
std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::Vector2d>> 
  GenerateEvenlySpacedSamplesAroundJack(
    const Eigen::VectorXd& x_lcs,
    const SamplingC3SamplingParams& sampling_params,
    const int& num_vertical, const int& num_horizontal
  ) {
  // Grab sampling parameters.
  double sampling_radius = sampling_params.sampling_radius;
  double min_angle_from_vertical = sampling_params.min_angle_from_vertical;
  double max_angle_from_vertical = sampling_params.max_angle_from_vertical;

  // Pull out the q and v from the LCS state.  The end effector location and
  // velocity of this state will be changed for the sample.
  Eigen::VectorXd q = x_lcs.head(N_Q);
  Eigen::VectorXd v = x_lcs.tail(N_V);

  // Center the sampling circle on the current ball location.
  Eigen::Vector3d object_xyz = q.tail(3);
  double x_samplec = object_xyz[0];
  double y_samplec = object_xyz[1];
  double z_samplec = object_xyz[2];

  // Prepare to store samples and (theta, elevation_theta) angles.
  std::vector<Eigen::VectorXd> x_lcs_samples;
  std::vector<Eigen::Vector2d> angles;

  // Double for loop over the number of vertical and horizontal samples.
  for (int i = 0; i < num_vertical; i++) {
    double elevation_theta = min_angle_from_vertical +
      i * (max_angle_from_vertical - min_angle_from_vertical)/(num_vertical-1);
    for (int j = 0; j < num_horizontal; j++) {
      double theta = j * 2*M_PI/num_horizontal;

      // Update the hypothetical state's end effector location to the tested
      // sample location.
      Eigen::VectorXd q_ee = Eigen::VectorXd::Zero(3);
      q_ee[0] = x_samplec + sampling_radius * cos(theta) * sin(elevation_theta);
      q_ee[1] = y_samplec + sampling_radius * sin(theta) * sin(elevation_theta);
      q_ee[2] = z_samplec + sampling_radius * cos(elevation_theta);

      Eigen::VectorXd candidate_state = Eigen::VectorXd::Zero(N_Q + N_V);
      candidate_state << q_ee, x_lcs.segment(3, N_Q - 3), v;

      // Store the sample and the angle pair.
      x_lcs_samples.push_back(candidate_state);
      Eigen::Vector2d angle_pair;
      angle_pair << theta, elevation_theta;
      angles.push_back(angle_pair);
    }
  }
  return std::make_pair(x_lcs_samples, angles);
}

void PythonFriendlyVectorOfVectorXdPrint(
  char* name, std::vector<Eigen::VectorXd> some_vector) {
  std::cout << name << " = np.array(";
  if (some_vector.size() > 1) {
    std::cout << "[";
  }
  for (const auto& some_element : some_vector) {
    std::cout << "\n\t[";
    for (int i = 0; i < some_element.size(); i++) {
      std::cout << some_element(i) << ", ";
    }
    std::cout << "], ";
  }
  if (some_vector.size() > 1) {
    std::cout << "]";
  }
  std::cout << ")" << std::endl;
}

void PythonFriendlyVectorOfVectorXdToFile(
  char* name, std::vector<Eigen::VectorXd> some_vector) {
  std::string filename = "/home/sharanya/workspace/dairlib/examples/sampling_c3/push_t/test";
  filename += "/tmp/";
  filename += name;
  filename += ".txt";

  // Remove the file if it already exists.
  std::remove(filename.c_str());
  std::ofstream file(filename);

  if (!file) {
    std::cerr << "Erorr: Could not create " << filename << std::endl;
  }
  std::cout << "Writing to " << filename << std::endl;

  for (const auto& some_element : some_vector) {
    for (int i = 0; i < some_element.size(); i++) {
      file << some_element(i) << " ";
    }
    file << "\n";
  }
  file.close();
}


}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }