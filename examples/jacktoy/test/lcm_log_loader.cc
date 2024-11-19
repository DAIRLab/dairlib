#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include "dairlib/lcmt_c3_state.hpp"
#include "dairlib/lcmt_object_state.hpp"
#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"
#include "dairlib/lcmt_radio_out.hpp"
#include <Eigen/Dense>
#include <drake/multibody/parsing/parser.h>
#include <drake/systems/framework/diagram_builder.h>
#include "systems/controllers/sampling_based_c3_controller.h"
#include "examples/jacktoy/parameters/franka_c3_controller_params.h"
#include "examples/jacktoy/parameters/franka_sim_params.h"
#include "systems/system_utils.h"
#include "systems/framework/timestamped_vector.h"
#include "solvers/c3_output.h"
#include "solvers/lcs_factory_preprocessor.h"
#include "solvers/lcs_factory.h"


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
using solvers::LCSFactoryPreProcessor;

int DoMain(int argc,  char* argv[]) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] <<" <log_folder> <time_into_log>" << std::endl;
    return 1;
  }
  const std::string& log_folder = std::string(argv[1]);
  const double& time_into_log = std::stod(std::string(argv[2]));

  // Turn the folder into a file path
  std::string log_number = log_folder.substr(log_folder.find_last_of("/")+1,6);
  std::string log_filepath = log_folder + "/simlog-" + log_number;
  std::cout<<"Parsing log at: " << log_filepath << std::endl;

  // Set the start time.
  // This time is how many seconds into the log we want to start processing information. Convert to microseconds.
  const int64_t time_into_log_in_microsecs = time_into_log*1e6;

  std::cout << "time into log in seconds: " << time_into_log << std::endl;
  std::cout << " in microseconds: " << time_into_log_in_microsecs << std::endl;


  // Load the recorded parameters.
  std::string franka_c3_controller_params_path = log_filepath;
  std::string to_replace = "simlog-";
  std::string franka_c3_controller_params_path_replacement = "franka_c3_controller_params_";
  franka_c3_controller_params_path.replace(franka_c3_controller_params_path.find(to_replace), 
                                           to_replace.length(), franka_c3_controller_params_path_replacement);
  FrankaC3ControllerParams controller_params = drake::yaml::LoadYamlFile<FrankaC3ControllerParams>(franka_c3_controller_params_path + ".yaml");

  std::string c3_gains_path = log_filepath;
  std::string c3_gains_path_replacement = "c3_gains_";
  c3_gains_path.replace(c3_gains_path.find(to_replace), to_replace.length(), c3_gains_path_replacement);
  C3Options c3_options;
  c3_options = drake::yaml::LoadYamlFile<C3Options>(c3_gains_path + ".yaml");
  // // NOTE:  can temporarily hard code many more ADMM iterations
  // c3_options.admm_iter = 8;

  std::string sim_params_path = log_filepath;
  std::string sim_params_path_replacement = "sim_params_";
  sim_params_path.replace(sim_params_path.find(to_replace), to_replace.length(), sim_params_path_replacement);
  FrankaSimParams sim_params = drake::yaml::LoadYamlFile<FrankaSimParams>(sim_params_path + ".yaml");

  std::string sampling_params_path = log_filepath;
  std::string sampling_params_path_replacement = "sampling_params_";
  sampling_params_path.replace(sampling_params_path.find(to_replace), to_replace.length(), sampling_params_path_replacement);
  SamplingC3SamplingParams sampling_params = \
    drake::yaml::LoadYamlFile<SamplingC3SamplingParams>(sampling_params_path + ".yaml");
  // NOTE:  hard code the number of additional samples to be 0, since this
  // script is just to debug a single C3 solve.
  sampling_params.num_additional_samples_c3 = 0;
  sampling_params.num_additional_samples_repos = 0;


  // Create an instance of the LCM log handler
  lcm::LCM lcm;
  lcm::LogFile log_file(log_filepath, "r");
  // Keep track of the timestamp of the first message in the log.
  int64_t u_init_time = 0;
  const lcm::LogEvent* first_event = log_file.readNextEvent();
  if (first_event != nullptr) {
      u_init_time = first_event->timestamp;  // Store the timestamp of the first event
      std::cout << "Initial event timestamp: " << u_init_time << std::endl;
  } else {
      std::cerr << "Error: No events in the log!" << std::endl;
      return 1;  // Exit if no events are found
  }

  // Now seek to the time we want to start processing the log. That is, time_into_log_in_microseconds offset 
  // from the first message in the log.
  log_file.seekToTimestamp(time_into_log_in_microsecs + u_init_time);

  const lcm::LogEvent* event;



  // This will store the x_solution over the 10 messages. 
  // To test the variability in solutions if everything remains the same.
  std::vector<Eigen::MatrixXf> x_sol_vec;
  std::vector<Eigen::VectorXf> x_lcs_actual_vec;
  std::vector<Eigen::VectorXf> x_lcs_desired_vec;
  std::vector<Eigen::VectorXf> x_lcs_final_desired_vec;
    
// Read the log until the first C3_ACTUAL message after the start time.
int count = 0;
while(count < 10) {
// Prepare to grab the actual LCS state and the desired LCS state.
Eigen::VectorXf x_lcs_actual = Eigen::VectorXf::Zero(19);
Eigen::VectorXf x_lcs_desired = Eigen::VectorXf::Zero(19);
Eigen::VectorXf x_lcs_final_desired = Eigen::VectorXf::Zero(19);
Eigen::MatrixXf dyn_feas_curr_plan_pos = Eigen::MatrixXf::Zero(3, c3_options.N+1);
Eigen::MatrixXf dyn_feas_curr_plan_orientation = Eigen::MatrixXf::Zero(4, c3_options.N+1);
Eigen::MatrixXf u_sol = Eigen::MatrixXf::Zero(3, c3_options.N);
Eigen::MatrixXf x_sol = Eigen::MatrixXf::Zero(19, c3_options.N);
Eigen::MatrixXf lambda_sol = Eigen::MatrixXf::Zero(16, c3_options.N);
Eigen::MatrixXf w_sol = Eigen::MatrixXf::Zero(38, c3_options.N);
Eigen::MatrixXf delta_sol = Eigen::MatrixXf::Zero(38, c3_options.N);
  while ((event = log_file.readNextEvent()) != nullptr) {
    // offset the time stamp by the initial time for better readability.
    int64_t adjusted_timestamp = event->timestamp - u_init_time;

    // if(event->channel == "FRANKA_STATE_SIMULATION"){
    //   // grab event time stamp and message u_time
    //   if(event->timestamp >= time_into_log_in_microsecs + u_init_time){
    //     dairlib::lcmt_robot_output message;
    //      if (message.decode(event->data, 0, event->datalen) > 0){
    //         utime_to_event_map_franka_state[message.utime] = adjusted_timestamp; 
    //      }
    //   }
    // }
    // else if(event->channel == "OBJECT_STATE_SIMULATION"){
    //   // grab event time stamp and message u_time
    //   if(event->timestamp >= time_into_log_in_microsecs + u_init_time){
    //     dairlib::lcmt_object_state message;
    //      if (message.decode(event->data, 0, event->datalen) > 0){
    //         utime_to_event_map_object_state[message.utime] = adjusted_timestamp; 
    //      }
    //   }
    // }
    // else if(event->channel == "TRACKING_TRAJECTORY_ACTOR"){
    //   // grab event time stamp and message u_time
    //   if(event->timestamp >= time_into_log_in_microsecs + u_init_time){
    //     dairlib::lcmt_timestamped_saved_traj message;
    //      if (message.decode(event->data, 0, event->datalen) > 0){
    //         if(utime_to_event_map_franka_state.find(message.utime) != utime_to_event_map_franka_state.end()){
    //           std::cout << "Received FRANKA_STATE_SIMULATION message   at: "<< utime_to_event_map_franka_state[message.utime]*1e-6 << " with u_time = "<<message.utime<<std::endl;
    //           std::cout << "Received TRACKING_TRAJECTORY_ACTOR message at: " << adjusted_timestamp*1e-6 << " with u_time = "<<message.utime<< std::endl;
    //           std::cout << "Timing difference in seconds is "<< (adjusted_timestamp - utime_to_event_map_franka_state[message.utime])*1e-6<<std::endl;
    //         }
    //         else if(utime_to_event_map_object_state.find(message.utime) != utime_to_event_map_object_state.end()){
    //           std::cout << "Received OBJECT_STATE_SIMULATION message   at: "<< utime_to_event_map_object_state[message.utime]*1e-6 << " with u_time = "<<message.utime<<std::endl;
    //           std::cout << "Received TRACKING_TRAJECTORY_ACTOR message at: " << adjusted_timestamp*1e-6 << " with u_time = "<<message.utime<< std::endl;
    //           std::cout << "Timing difference in seconds is "<< (adjusted_timestamp - utime_to_event_map_object_state[message.utime])*1e-6<<std::endl;
    //         }
    //         else{
    //           std::cout << "-----------------------------" << std::endl;
    //           std::cout << "\tReceived TRACKING_TRAJECTORY_ACTOR message at: " << adjusted_timestamp*1e-6 << " with u_time = "<<message.utime<< std::endl;
    //           std::cout << "-----------------------------" << std::endl;
    //         }
    //      }
    //   }
    // }
    // else if(event->channel == "C3_DEBUG_CURR"){
    //   // grab event time stamp and message u_time
    //   if(event->timestamp >= time_into_log_in_microsecs + u_init_time){
    //     dairlib::lcmt_c3_output message;
    //      if (message.decode(event->data, 0, event->datalen) > 0){
    //         if(utime_to_event_map_franka_state.find(message.utime) != utime_to_event_map_franka_state.end()){
    //           // std::cout << "-----------------------------" << std::endl;
    //           // std::cout << "Received FRANKA_STATE_SIMULATION message   at: "<< utime_to_event_map_franka_state[message.utime]*1e-6 << " with u_time = "<<message.utime<<std::endl;
    //           std::cout << "Received C3_DEBUG_CURR message at: " << adjusted_timestamp*1e-6 << " with u_time = "<<message.utime<< std::endl;
    //           // std::cout << "Timing difference in seconds is "<< (adjusted_timestamp - utime_to_event_map_franka_state[message.utime])*1e-6<<std::endl;
    //           std::cout << "-----------------------------" << std::endl;
    //         }
    //         else if(utime_to_event_map_object_state.find(message.utime) != utime_to_event_map_object_state.end()){
    //           // std::cout << "-----------------------------" << std::endl;
    //           // std::cout << "Received OBJECT_STATE_SIMULATION message   at: "<< utime_to_event_map_object_state[message.utime]*1e-6 << " with u_time = "<<message.utime<<std::endl;
    //           std::cout << "Received C3_DEBUG_CURR message at: " << adjusted_timestamp*1e-6 << " with u_time = "<<message.utime<< std::endl;
    //           // std::cout << "Timing difference in seconds is "<< (adjusted_timestamp - utime_to_event_map_object_state[message.utime])*1e-6<<std::endl;
    //           std::cout << "-----------------------------" << std::endl;
    //         }
    //         else{
    //           std::cout << "-----------------------------" << std::endl;
    //           std::cout << "\tReceived C3_DEBUG_CURR message at: " << adjusted_timestamp*1e-6 << " with u_time = "<<message.utime<< std::endl;
    //           std::cout << "-----------------------------" << std::endl;
    //         }
    //      }
    //   }
    // }

    if (event->channel == "C3_ACTUAL") {
      if(event->timestamp > time_into_log_in_microsecs + u_init_time) {
        dairlib::lcmt_c3_state message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received C3_ACTUAL message in seconds utime: " << (message.utime)/1e6 << " and event timestamp "<< adjusted_timestamp/1e6 << std::endl;
          for (int i=0; i<19; i++) {
            x_lcs_actual(i) = message.state[i];
          }
          x_lcs_actual_vec.push_back(x_lcs_actual);
          if ((x_lcs_desired != Eigen::VectorXf::Zero(19)) &&
              (x_lcs_final_desired != Eigen::VectorXf::Zero(19)) &&
              (dyn_feas_curr_plan_obj_pos != Eigen::MatrixXf::Zero(3, c3_options.N+1)) &&
              (dyn_feas_curr_plan_obj_orientation != Eigen::MatrixXf::Zero(4, c3_options.N+1)) &&
              (u_sol != Eigen::MatrixXf::Zero(3, c3_options.N))) {
            break;
          }
        } else {
          std::cerr << "Failed to decode C3_ACTUALs message" << std::endl;
        }
      }
    }
    else if (event->channel == "C3_TARGET") {
      if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
        dairlib::lcmt_c3_state message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received C3_TARGET message at utime: " << (message.utime)/1e6 << " and event timestamp "<< adjusted_timestamp/1e6 << std::endl;
          for (int i=0; i<19; i++) {
            x_lcs_desired(i) = message.state[i];
          }
          x_lcs_desired_vec.push_back(x_lcs_desired);
          if ((x_lcs_actual != Eigen::VectorXf::Zero(19)) &&
              (x_lcs_final_desired != Eigen::VectorXf::Zero(19)) &&
              (dyn_feas_curr_plan_obj_pos != Eigen::MatrixXf::Zero(3, c3_options.N+1)) &&
              (dyn_feas_curr_plan_obj_orientation != Eigen::MatrixXf::Zero(4, c3_options.N+1))) {
            break;
          }
        } else {
          std::cerr << "Failed to decode C3_TARGETs message" << std::endl;
        }
      }
    }
    else if (event->channel == "C3_FINAL_TARGET") {
      if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
        dairlib::lcmt_c3_state message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received C3_FINAL_TARGET message at utime: " << (message.utime)/1e6 << " and event timestamp "<< adjusted_timestamp/1e6 << std::endl;
          for (int i=0; i<19; i++) {
            x_lcs_final_desired(i) = message.state[i];
          }
          x_lcs_final_desired_vec.push_back(x_lcs_final_desired);
          if ((x_lcs_actual != Eigen::VectorXf::Zero(19)) &&
              (x_lcs_desired != Eigen::VectorXf::Zero(19)) &&
              (dyn_feas_curr_plan_obj_pos != Eigen::MatrixXf::Zero(3, c3_options.N+1)) &&
              (dyn_feas_curr_plan_obj_orientation != Eigen::MatrixXf::Zero(4, c3_options.N+1)) &&
              (u_sol != Eigen::MatrixXf::Zero(3, c3_options.N))) {
            break;
          }
        } else {
          std::cerr << "Failed to decode C3_FINAL_TARGETs message" << std::endl;
        }
      }
    }
    else if (event->channel == "DYNAMICALLY_FEASIBLE_CURR_PLAN") {
      if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
        dairlib::lcmt_timestamped_saved_traj message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received DYNAMICALLY_FEASIBLE_CURR_PLAN message at utime: " <<
            (message.utime)/1e6 << " and event timestamp "<< adjusted_timestamp/1e6 << std::endl;
          for (int i=0; i<4; i++) {
            for (int j=0; j<c3_options.N+1; j++) {
              dyn_feas_curr_plan_obj_orientation(i,j) = message.saved_traj.trajectories[0].datapoints[i][j];
            }
          }
          for (int i=0; i<3; i++) {
            for (int j=0; j<c3_options.N+1; j++) {
              dyn_feas_curr_plan_obj_pos(i,j) = message.saved_traj.trajectories[1].datapoints[i][j];
            }
          }
          if ((x_lcs_actual != Eigen::VectorXf::Zero(19)) &&
              (x_lcs_final_desired != Eigen::VectorXf::Zero(19)) &&
              (x_lcs_desired != Eigen::VectorXf::Zero(19)) &&
              (u_sol != Eigen::MatrixXf::Zero(3, c3_options.N))) {
            break;
          }
        } else {
          std::cerr << "Failed to decode DYNAMICALLY_FEASIBLE_CURR_PLANs message" << std::endl;
        }
      }
    }
    else if(event->channel == "DYNAMICALLY_FEASIBLE_CURR_ACTOR_PLAN"){
      dairlib::lcmt_timestamped_saved_traj message;
      if (message.decode(event->data, 0, event->datalen) > 0){
        std::cout << "Received DYNAMICALLY_FEASIBLE_EE_PLAN message at: " <<(message.utime)/1e6 << " and event timestamp "<< adjusted_timestamp/1e6 << std::endl;
        // std::cout<<"name of trajectory in ee: "<<message.saved_traj.trajectories[0].trajectory_name<<std::endl;
        for(int i = 0; i < 3; i++){
          for(int j = 0; j < c3_options.N+1; j++){
            dyn_feas_curr_plan_ee_pos(i,j) = message.saved_traj.trajectories[0].datapoints[i][j];
          }
        }
          if ((x_lcs_actual != Eigen::VectorXf::Zero(19)) &&
              (x_lcs_final_desired != Eigen::VectorXf::Zero(19)) &&
              (x_lcs_desired != Eigen::VectorXf::Zero(19)) &&
              (dyn_feas_curr_plan_obj_pos != Eigen::MatrixXf::Zero(3, c3_options.N+1)) &&
              (dyn_feas_curr_plan_obj_orientation != Eigen::MatrixXf::Zero(4, c3_options.N+1))) {
            break;
          }
      }
    }
    else if (event->channel == "C3_DEBUG_CURR") {
        if(event->timestamp > u_time_into_log + u_init_time) {
          dairlib::lcmt_c3_output message;
          if (message.decode(event->data, 0, event->datalen) > 0) {
            std::cout << "Received C3_DEBUG_CURR message at: " <<
              (message.utime)/1e6 << std::endl;
            for (int i=0; i<3; i++) {
              for (int j=0; j<c3_options.N+1; j++) {
                u_sol(i,j) = message.c3_solution.u_sol[i][j];
              }
            }
            // Read the c3 intermediates and print them
            // std::cout<<"Printing C3 Final solution including the ws and deltas"<<std::endl;
            for(int i = 0; i < 19; i++){
              for(int j = 0; j < c3_options.N; j++){
                x_sol(i,j) = message.c3_solution.x_sol[i][j];
              }
            }
            // Pushing back the x_sol to the vector
            // std::cout<<"Pushing back the x_sol to the vector"<<std::endl;
            // std::cout<<x_sol<<std::endl;
            x_sol_vec.push_back(x_sol);
            for(int i = 0; i < 16; i++){
              for(int j = 0; j < c3_options.N; j++){
                lambda_sol(i,j) = message.c3_solution.lambda_sol[i][j];
              }
            }
            for(int i = 0; i < 38; i++){
              for(int j = 0; j < c3_options.N; j++){
                w_sol(i,j) = message.c3_intermediates.w_sol[i][j];
              }
            }
            for(int i = 0; i < 38; i++){
              for(int j = 0; j < c3_options.N; j++){
                delta_sol(i,j) = message.c3_intermediates.delta_sol[i][j];
              }
            }
            if ((x_lcs_actual != Eigen::VectorXf::Zero(19)) &&
                (x_lcs_final_desired != Eigen::VectorXf::Zero(19)) &&
                (x_lcs_desired != Eigen::VectorXf::Zero(19)) &&
                (dyn_feas_curr_plan_pos != Eigen::MatrixXf::Zero(3, c3_options.N+1)) &&
                (dyn_feas_curr_plan_orientation != Eigen::MatrixXf::Zero(4, c3_options.N+1))) {
              break;
            }
          } else {
            std::cerr << "Failed to decode C3_DEBUG_CURRs message" << std::endl;
          }
        }
    }
  }
    count++;
  } // for loop to read 10 messages

  // iterate over x_sol_vec and print out the variability in the solutions from the average at each time step.
  // Eigen::MatrixXf x_sol_average = std::accumulate(x_sol_vec.begin(), x_sol_vec.end(), Eigen::MatrixXf::Zero(19, c3_options.N).eval(),
  //                                                 [](const Eigen::MatrixXf& a, const Eigen::MatrixXf& b) { return a + b; });    
  // x_sol_average = x_sol_average/x_sol_vec.size();

  // std::vector<Eigen::MatrixXf> x_sol_diff_vec;
  // for(int i = 0; i < x_sol_vec.size(); i++){
  //   Eigen::MatrixXf x_sol_diff = x_sol_vec[i] - x_sol_average;
  //   x_sol_diff_vec.push_back(x_sol_diff);
  // }

  // // Print out the average variability.
  // Eigen::MatrixXf x_sol_diff_average = std::accumulate(x_sol_diff_vec.begin(), x_sol_diff_vec.end(), Eigen::MatrixXf::Zero(19, c3_options.N).eval(),
  //                                                 [](const Eigen::MatrixXf& a, const Eigen::MatrixXf& b) { return a + b; });
  // x_sol_diff_average = x_sol_diff_average/x_sol_diff_vec.size();
  // std::cout<<"Average variability in the x_sol: "<<std::endl;
  // std::cout<<x_sol_diff_average<<std::endl;
  
  // std::cout << "\nFuture timestamps:" << std::endl;
  // int more_msgs_to_print = 5;
  // while(((event = log_file.readNextEvent()) != nullptr) && (more_msgs_to_print > 0)) {
  //   if (event->channel == "C3_ACTUAL") {
  //     std::cout << "Received C3_ACTUAL message in seconds: " << (event->timestamp - u_init_time)/1e6<< std::endl;
  //     more_msgs_to_print--;
  //   }
  //   else if(event->channel == "C3_DEBUG_CURR"){
  //     std::cout << "Received C3_DEBUG_CURR message in seconds: " << (event->timestamp - u_init_time)/1e6<< std::endl;
  //     more_msgs_to_print--;
  //   }
  // }

  // for all solution vectors in x_sol_vec, compute the difference between each pair of matrices and find the max norm.
  float max_norm = -9999999999999;
  for(int i = 0; i < x_sol_vec.size(); i++){
    for(int j = i+1; j < x_sol_vec.size(); j++){
      Eigen::MatrixXf x_sol_diff = x_sol_vec[i] - x_sol_vec[j];
      max_norm = std::max(max_norm, x_sol_diff.norm());
    }
  }
  std::cout<<"Max norm of the difference between the logged solutions: "<<max_norm<<std::endl;


  std::cout << "\nFound these states:" << std::endl;
  // std::cout << "Actual: " << x_lcs_actual.transpose() << std::endl;
  // std::cout << "Desired: " << x_lcs_desired.transpose() << std::endl;
  // std::cout << "Final Desired: " << x_lcs_final_desired.transpose() << std::endl;
  // std::cout << "\nDyn Feas Curr plan from log:\n" << dyn_feas_curr_plan_pos << std::endl;
  // std::cout << "\nDyn Feas Curr plan orientation from log:\n" << dyn_feas_curr_plan_orientation << std::endl;
  // std::cout << "\nFinal U_sol from log:\n" << u_sol << std::endl;
  // std::cout << "\nFinal x_sol from log:\n" << x_sol << std::endl;
  // std::cout << "\nFinal lambda_sol from log:\n" << lambda_sol << std::endl;
  // std::cout << "\nFinal w_sol from log:\n" << w_sol << std::endl;
  // std::cout << "\nFinal delta_sol from log:\n" << delta_sol << std::endl;

  // // Create the plant for the LCS
  DiagramBuilder<double> plant_builder;
  // This function initializes a MultibodyPlant with a specified time step,
  // ensures that the builder is valid, and then passes the newly created Multibodyplant 
  // to an overloaded definition of AddMultibodyPlantSceneGraph that takes in the plant
  // and a scene graph and adds it to the diagram.
  // This overloaded function then returns a tuple of pointers to the plant and the scene graph.
  auto [plant_for_lcs, scene_graph] =
      AddMultibodyPlantSceneGraph(&plant_builder, 0.0);

  Parser parser_for_lcs(&plant_for_lcs);
  parser_for_lcs.SetAutoRenaming(true);
  /// Loading simple model of end effector (just a sphere) for the lcs plant
  parser_for_lcs.AddModels(controller_params.end_effector_simple_model);
  parser_for_lcs.AddModels(controller_params.jack_model);
  parser_for_lcs.AddModels(controller_params.ground_model);	
  // TO DO: The base link may change to the simple end effector model link name
  // or might just be removed entirely.
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
  drake::geometry::GeometryId ground_geoms =
      plant_for_lcs.GetCollisionGeometriesForBody(
          plant_for_lcs.GetBodyByName("ground"))[0];

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

  //   Creating a map of contact geoms
  std::unordered_map<std::string, drake::geometry::GeometryId> contact_geoms;
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
  
  std::vector<std::vector<SortedPair<GeometryId>>>
      contact_pairs;  // will have [[(ee,cap1), (ee,cap2), (ee_cap3)],
                      // [(ground,cap1_sph1)], [(ground,cap1_sph2)],
                      // [(ground,cap2_sph1)], [(ground,cap2_sph2)],
                      // [(ground,cap3_sph1)], [(ground,cap3_sph2)],
  contact_pairs.push_back(ee_contact_pairs);

  if(c3_options.num_contacts_index == 2 || c3_options.num_contacts_index == 3){
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

  // std::cout<<"num_positions: " << plant_for_lcs.num_positions() << std::endl;
  // std::cout<<"num_velocities: " << plant_for_lcs.num_velocities() << std::endl;
  // // plant_for_lcs.SetPositionsAndVelocities(plant_for_lcs_context.get(), x_lcs_actual);
  // std::cout<<"position names: " << plant_for_lcs.GetPositionNames()[0] << std::endl;
  // // cast to VectorXd since the SetPositionsAndVelocities function requires a VectorXd
  // plant_for_lcs.SetPositionsAndVelocities(&plant_for_lcs_context, x_lcs_actual.cast<double>());
  // auto xu_ad = drake::math::InitializeAutoDiff(x_lcs_actual.cast<double>());
  // plant_for_lcs_autodiff->SetPositionsAndVelocities(plant_for_lcs_context_ad.get(), xu_ad);

  plant_diagram->set_name(("franka_c3_plant"));
  DrawAndSaveDiagramGraph(*plant_diagram, "examples/jacktoy/test/franka_c3_plant_in_log_loader");


    // if(sampling_params.use_more_contacts_to_compute_cost){
    //   solvers::ContactModel contact_model;
    //   if (c3_options.contact_model == "stewart_and_trinkle") {
    //     contact_model = solvers::ContactModel::kStewartAndTrinkle;
    //   } else if (c3_options.contact_model == "anitescu") {
    //     contact_model = solvers::ContactModel::kAnitescu;
    //   } else {
    //     std::cerr << ("Unknown or unsupported contact model") << std::endl;
    //     DRAKE_THROW_UNLESS(false);
    //   }
    //   // Create an LCS object with all contact pairs resolved.
    //   // Preprocessing the contact pairs
    //   std::vector<SortedPair<GeometryId>> resolved_contact_pairs_for_cost_simulation;
    //   resolved_contact_pairs_for_cost_simulation = LCSFactoryPreProcessor::PreProcessor(
    //     plant_for_lcs, plant_for_lcs_context, contact_pairs,
    //     {3,6},
    //     c3_options.num_friction_directions,
    //     6, true);
    //   solvers::LCS lcs_object_sample_for_cost_simulation = solvers::LCSFactory::LinearizePlantToLCS(
    //     plant_for_lcs, plant_for_lcs_context, *plant_for_lcs_autodiff, *plant_for_lcs_context_ad, 
    //     resolved_contact_pairs_for_cost_simulation, c3_options.num_friction_directions, 
    //     {0.4615, 0.4615, 0.4615, 0.4615, 0.4615, 0.4615, 0.4615, 0.4615, 0.4615}, 
    //     c3_options.planning_dt, c3_options.N, contact_model);
    //     // simulate the 9 contact lcs
    //     // TODO:
    //     // U's from the dynamically feasible plan.
    //     // change the U's according to cost type.        
    // }
  

  DiagramBuilder<double> builder;
  auto controller = builder.AddSystem<dairlib::systems::SamplingC3Controller>(
      plant_for_lcs, &plant_for_lcs_context, *plant_for_lcs_autodiff,
      plant_for_lcs_context_ad.get(), contact_pairs, c3_options,
      sampling_params, false);
  // get controller context
  auto controller_context = controller->CreateDefaultContext();

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_c3_controller"));

  DrawAndSaveDiagramGraph(*owned_diagram, "examples/jacktoy/test/franka_c3_controller_in_log_loader");

  // // // fix input port values
  std::vector<Eigen::MatrixXf> recomputed_x_sol_vec;
  std::cout<<"count is now  "<<count<<std::endl;
  std::cout<<"size of x_lcs_actual_vec is "<<x_lcs_actual_vec.size()<<std::endl;
  std::cout<<"size of x_lcs_desired_vec is "<<x_lcs_desired_vec.size()<<std::endl;
  std::cout<<"size of x_lcs_final_desired_vec is "<<x_lcs_final_desired_vec.size()<<std::endl;
  std::cout<<"size of x_sol_vec is "<<x_sol_vec.size()<<std::endl;
  for(int i = 0; i<count; i++){
    plant_for_lcs.SetPositionsAndVelocities(&plant_for_lcs_context, x_lcs_actual_vec[i].cast<double>());
    controller->get_input_port_radio().FixValue(controller_context.get(), drake::Value<dairlib::lcmt_radio_out>{});
    controller->get_input_port_lcs_state().FixValue(controller_context.get(),  TimestampedVector<double>(x_lcs_actual_vec[i].cast<double>()));
    controller->get_input_port_target().FixValue(controller_context.get(), x_lcs_desired_vec[i].cast<double>());
    controller->get_input_port_final_target().FixValue(controller_context.get(), x_lcs_final_desired_vec[i].cast<double>());
    auto discrete_state = controller->AllocateDiscreteVariables();
    controller->CalcForcedDiscreteVariableUpdate(*controller_context, discrete_state.get());
    controller->ForcedPublish(*controller_context);
    auto c3_solution = dairlib::C3Output::C3Solution();
    c3_solution.x_sol_ = Eigen::MatrixXf::Zero(19, 5);
    c3_solution.lambda_sol_ = Eigen::MatrixXf::Zero(16, 5);
    c3_solution.u_sol_ = Eigen::MatrixXf::Zero(3, 5);
    c3_solution.time_vector_ = Eigen::VectorXf::Zero(5);
    controller->OutputC3SolutionCurrPlan(*controller_context, &c3_solution);
    recomputed_x_sol_vec.push_back(c3_solution.x_sol_);
  }


  float recomputed_max_norm = -9999999999999;
  for(int i = 0; i < recomputed_x_sol_vec.size(); i++){
    for(int j = i+1; j < recomputed_x_sol_vec.size(); j++){
      Eigen::MatrixXf recomputed_x_sol_diff = recomputed_x_sol_vec[i] - recomputed_x_sol_vec[j];
      recomputed_max_norm = std::max(recomputed_max_norm, recomputed_x_sol_diff.norm());
    }
  }
  std::cout<<"Max norm of the difference between the recomputed solutions: "<<recomputed_max_norm<<std::endl;

  
  float mixed_max_norm = -9999999999999;
  for(int i = 0; i < recomputed_x_sol_vec.size(); i++){
    for(int j = i+1; j < x_sol_vec.size(); j++){
      Eigen::MatrixXf mixed_x_sol_diff = recomputed_x_sol_vec[i] - x_sol_vec[j];
      mixed_max_norm = std::max(mixed_max_norm, mixed_x_sol_diff.norm());
    }
  }
  std::cout<<"Max norm of the difference between the logged and recomputed solutions: "<<mixed_max_norm<<std::endl;

  // // iterate over recomputed_x_sol_vec and print out the variability in the solutions from the average at each time step.
  // Eigen::MatrixXf recomputed_x_sol_average = std::accumulate(recomputed_x_sol_vec.begin(), recomputed_x_sol_vec.end(), Eigen::MatrixXf::Zero(19, c3_options.N).eval(),
  //                                                 [](const Eigen::MatrixXf& a, const Eigen::MatrixXf& b) { return a + b; });
  // recomputed_x_sol_average = recomputed_x_sol_average/recomputed_x_sol_vec.size();
  // std::vector<Eigen::MatrixXf> recomputed_x_sol_diff_vec;
  // for(int i = 0; i < recomputed_x_sol_vec.size(); i++){
  //   Eigen::MatrixXf recomputed_x_sol_diff = recomputed_x_sol_vec[i] - recomputed_x_sol_average;
  //   recomputed_x_sol_diff_vec.push_back(recomputed_x_sol_diff);
  // }
  
  // // Print out the average variability.
  // Eigen::MatrixXf recomputed_x_sol_diff_average = std::accumulate(recomputed_x_sol_diff_vec.begin(), recomputed_x_sol_diff_vec.end(), Eigen::MatrixXf::Zero(19, c3_options.N).eval(),
  //                                                 [](const Eigen::MatrixXf& a, const Eigen::MatrixXf& b) { return a + b; });
  // recomputed_x_sol_diff_average = recomputed_x_sol_diff_average/recomputed_x_sol_diff_vec.size();
  // std::cout<<"Average variability in the recomputed x_sol: "<<std::endl;
  // std::cout<<recomputed_x_sol_diff_average<<std::endl;

  // controller->get_input_port_radio().FixValue(controller_context.get(), drake::Value<dairlib::lcmt_radio_out>{});
  // std::cout<<"input port size: "<<controller->get_input_port_lcs_state().size()<<std::endl;
  // controller->get_input_port_lcs_state().FixValue(controller_context.get(),  TimestampedVector<double>(x_lcs_actual.cast<double>()));
  // controller->get_input_port_target().FixValue(controller_context.get(), x_lcs_desired.cast<double>());
  // controller->get_input_port_final_target().FixValue(controller_context.get(), x_lcs_final_desired.cast<double>());

  // auto discrete_state = controller->AllocateDiscreteVariables();
  // controller->CalcForcedDiscreteVariableUpdate(*controller_context, discrete_state.get());
  // controller->ForcedPublish(*controller_context);

  // auto c3_solution = dairlib::C3Output::C3Solution();
  // c3_solution.x_sol_ = Eigen::MatrixXf::Zero(19, 5);
  // c3_solution.lambda_sol_ = Eigen::MatrixXf::Zero(16, 5);
  // c3_solution.u_sol_ = Eigen::MatrixXf::Zero(3, 5);
  // c3_solution.time_vector_ = Eigen::VectorXf::Zero(5);
  // controller->OutputC3SolutionCurrPlan(*controller_context, &c3_solution);
  // std::cout<<"Recomputed x_sol:"<<std::endl;
  // std::cout<<c3_solution.x_sol_<<std::endl;

  // std::cout << "Finished ForcedPublish" << std::endl;
  return 0;
}
}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }