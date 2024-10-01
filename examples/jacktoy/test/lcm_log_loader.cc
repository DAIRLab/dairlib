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


namespace dairlib {

using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using drake::multibody::AddMultibodyPlantSceneGraph;
using dairlib::systems::TimestampedVector;
using drake::multibody::Parser;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RigidTransform;

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
  // NOTE: Temporarily hard coding the Q values to something higher
  // c3_options.q_vector = {10, 10, 10,  
  //          10, 10, 10, 10,             
  //          20, 20, 20,   
  //          50, 50, 50,            
  //          500, 500, 500,               
  //          50, 50, 50};

  // Eigen::VectorXd q_position_and_orientation = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
  //       q_vector_position_and_orientation.data(), q_vector_position_and_orientation.size());
  // c3_options.Q_position_and_orientation = c3_options.w_Q * q_position_and_orientation.asDiagonal();

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
    
  // Read the log until the first C3_ACTUAL message after the start time.
  std::unordered_map<int64_t, int64_t> utime_to_event_map;
  while ((event = log_file.readNextEvent()) != nullptr) {
    if(event->channel == "FRANKA_STATE_SIMULATION"){
      // grab event time stamp and message u_time
      if(event->timestamp >= time_into_log_in_microsecs + u_init_time){
        // dairlib::lcmt_object_state message;
        dairlib::lcmt_robot_output message;
         if (message.decode(event->data, 0, event->datalen) > 0){
            // Offset the time stamp by the initial time for better readability.
            utime_to_event_map[message.utime] = event->timestamp - u_init_time; 
         }
      }
    }
    else if(event->channel == "TRACKING_TRAJECTORY_ACTOR"){
      // grab event time stamp and message u_time
      if(event->timestamp >= time_into_log_in_microsecs + u_init_time){
        dairlib::lcmt_timestamped_saved_traj message;
         if (message.decode(event->data, 0, event->datalen) > 0){
            if(utime_to_event_map.find(message.utime) != utime_to_event_map.end()){
              // Offset the time stamps by the initial time for better readability.
              std::cout << "-----------------------------" << std::endl;
              std::cout << "Received FRANKA_STATE_SIMULATION message   at: "<< utime_to_event_map[message.utime]*1e-6 << " with u_time = "<<message.utime<<std::endl;
              std::cout << "Received TRACKING_TRAJECTORY_ACTOR message at: " << (event->timestamp - u_init_time)*1e-6 << " with u_time = "<<message.utime<< std::endl;
              std::cout << "Timing difference in seconds is "<< (((event->timestamp- u_init_time)*1e-6) - utime_to_event_map[message.utime]*1e-6)<<std::endl;
              std::cout << "-----------------------------" << std::endl;
            }
            // std::cout << "Received TRACKING_TRAJECTORY_ACTOR message at: " << event->timestamp << " with u_time = "<<message.utime<< std::endl;
            // u_times_to_match.push_back(message.utime);
         }
      }
    }

    // if (event->channel == "C3_ACTUAL") {
    //   if(event->timestamp > time_into_log_in_microsecs + u_init_time) {
    //     dairlib::lcmt_c3_state message;
    //     if (message.decode(event->data, 0, event->datalen) > 0) {
    //       std::cout << "Received C3_ACTUAL message in seconds: " << (message.utime)/1e6<< std::endl;
    //       for (int i=0; i<19; i++) {
    //         x_lcs_actual(i) = message.state[i];
    //       }
    //       if ((x_lcs_desired != Eigen::VectorXf::Zero(19)) &&
    //           (x_lcs_final_desired != Eigen::VectorXf::Zero(19)) &&
    //           (dyn_feas_curr_plan_pos != Eigen::MatrixXf::Zero(3, c3_options.N+1)) &&
    //           (dyn_feas_curr_plan_orientation != Eigen::MatrixXf::Zero(4, c3_options.N+1)) &&
    //           (u_sol != Eigen::MatrixXf::Zero(3, c3_options.N))) {
    //         break;
    //       }
    //     } else {
    //       std::cerr << "Failed to decode C3_ACTUALs message" << std::endl;
    //     }
    //   }
    // }
    // else if (event->channel == "C3_TARGET") {
    //   if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
    //     dairlib::lcmt_c3_state message;
    //     if (message.decode(event->data, 0, event->datalen) > 0) {
    //       std::cout << "Received C3_TARGET message at: " << (message.utime)/1e6 << std::endl;
    //       for (int i=0; i<19; i++) {
    //         x_lcs_desired(i) = message.state[i];
    //       }
    //       if ((x_lcs_actual != Eigen::VectorXf::Zero(19)) &&
    //           (x_lcs_final_desired != Eigen::VectorXf::Zero(19)) &&
    //           (dyn_feas_curr_plan_pos != Eigen::MatrixXf::Zero(3, c3_options.N+1)) &&
    //           (dyn_feas_curr_plan_orientation != Eigen::MatrixXf::Zero(4, c3_options.N+1))) {
    //         break;
    //       }
    //     } else {
    //       std::cerr << "Failed to decode C3_TARGETs message" << std::endl;
    //     }
    //   }
    // }
    // else if (event->channel == "C3_FINAL_TARGET") {
    //   if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
    //     dairlib::lcmt_c3_state message;
    //     if (message.decode(event->data, 0, event->datalen) > 0) {
    //       std::cout << "Received C3_FINAL_TARGET message at: " << (message.utime)/1e6 << std::endl;
    //       for (int i=0; i<19; i++) {
    //         x_lcs_final_desired(i) = message.state[i];
    //       }
    //       if ((x_lcs_actual != Eigen::VectorXf::Zero(19)) &&
    //           (x_lcs_desired != Eigen::VectorXf::Zero(19)) &&
    //           (dyn_feas_curr_plan_pos != Eigen::MatrixXf::Zero(3, c3_options.N+1)) &&
    //           (dyn_feas_curr_plan_orientation != Eigen::MatrixXf::Zero(4, c3_options.N+1)) &&
    //           (u_sol != Eigen::MatrixXf::Zero(3, c3_options.N))) {
    //         break;
    //       }
    //     } else {
    //       std::cerr << "Failed to decode C3_FINAL_TARGETs message" << std::endl;
    //     }
    //   }
    // }
    // else if (event->channel == "DYNAMICALLY_FEASIBLE_CURR_PLAN") {
    //   if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
    //     dairlib::lcmt_timestamped_saved_traj message;
    //     if (message.decode(event->data, 0, event->datalen) > 0) {
    //       std::cout << "Received DYNAMICALLY_FEASIBLE_CURR_PLAN message at: " <<
    //         (message.utime)/1e6 << std::endl;
    //       for (int i=0; i<4; i++) {
    //         for (int j=0; j<c3_options.N+1; j++) {
    //           dyn_feas_curr_plan_orientation(i,j) = message.saved_traj.trajectories[0].datapoints[i][j];
    //         }
    //       }
    //       for (int i=0; i<3; i++) {
    //         for (int j=0; j<c3_options.N+1; j++) {
    //           dyn_feas_curr_plan_pos(i,j) = message.saved_traj.trajectories[1].datapoints[i][j];
    //         }
    //       }
    //       if ((x_lcs_actual != Eigen::VectorXf::Zero(19)) &&
    //           (x_lcs_final_desired != Eigen::VectorXf::Zero(19)) &&
    //           (x_lcs_desired != Eigen::VectorXf::Zero(19)) &&
    //           (u_sol != Eigen::MatrixXf::Zero(3, c3_options.N))) {
    //         break;
    //       }
    //     } else {
    //       std::cerr << "Failed to decode DYNAMICALLY_FEASIBLE_CURR_PLANs message" << std::endl;
    //     }
    //   }
    // }
    // else if (event->channel == "C3_DEBUG_CURR") {
    //   if(event->timestamp >= time_into_log_in_microsecs + u_init_time) {
    //     dairlib::lcmt_c3_output message;
    //     if (message.decode(event->data, 0, event->datalen) > 0) {
    //       std::cout << "Received C3_DEBUG_CURR message at: " <<
    //         (message.utime)/1e6 << std::endl;
    //       for (int i=0; i<3; i++) {
    //         for (int j=0; j<c3_options.N+1; j++) {
    //           u_sol(i,j) = message.c3_solution.u_sol[i][j];
    //         }
    //       }
    //       // Read the c3 intermediates and print them
    //       // std::cout<<"Printing C3 Final solution including the ws and deltas"<<std::endl;
    //       for(int i = 0; i < 19; i++){
    //         for(int j = 0; j < c3_options.N; j++){
    //           x_sol(i,j) = message.c3_solution.x_sol[i][j];
    //         }
    //       }
    //       for(int i = 0; i < 16; i++){
    //         for(int j = 0; j < c3_options.N; j++){
    //           lambda_sol(i,j) = message.c3_solution.lambda_sol[i][j];
    //         }
    //       }
    //       for(int i = 0; i < 38; i++){
    //         for(int j = 0; j < c3_options.N; j++){
    //           w_sol(i,j) = message.c3_intermediates.w_sol[i][j];
    //         }
    //       }
    //       for(int i = 0; i < 38; i++){
    //         for(int j = 0; j < c3_options.N; j++){
    //           delta_sol(i,j) = message.c3_intermediates.delta_sol[i][j];
    //         }
    //       }
    //       if ((x_lcs_actual != Eigen::VectorXf::Zero(19)) &&
    //           (x_lcs_final_desired != Eigen::VectorXf::Zero(19)) &&
    //           (x_lcs_desired != Eigen::VectorXf::Zero(19)) &&
    //           (dyn_feas_curr_plan_pos != Eigen::MatrixXf::Zero(3, c3_options.N+1)) &&
    //           (dyn_feas_curr_plan_orientation != Eigen::MatrixXf::Zero(4, c3_options.N+1))) {
    //         break;
    //       }
    //     } else {
    //       std::cerr << "Failed to decode C3_DEBUG_CURRs message" << std::endl;
    //     }
    //   }
    // }
  }
  
  std::cout << "\nFuture timestamps:" << std::endl;
  int more_msgs_to_print = 5;
  while(((event = log_file.readNextEvent()) != nullptr) && (more_msgs_to_print > 0)) {
    if (event->channel == "OBJECT_STATE_SIMULATION") {
      std::cout << "Received OBJECT_STATE_SIMULATION message in seconds: " << (event->timestamp - u_init_time)/1e6<< std::endl;
      more_msgs_to_print--;
    }
    if (event->channel == "TRACKING_TRAJECTORY_ACTOR") {
      std::cout << "Received TRACKING_TRAJECTORY_ACTOR message in seconds: " << (event->timestamp - u_init_time)/1e6<< std::endl;
      more_msgs_to_print--;
    }
    // if (event->channel == "C3_ACTUAL") {
    //   std::cout << "Received C3_ACTUAL message in seconds: " << (event->timestamp - u_init_time)/1e6<< std::endl;
    //   more_msgs_to_print--;
    // }
  }

  // std::cout << "\nFound these states:" << std::endl;
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
  // DiagramBuilder<double> plant_builder;
  // // This function initializes a MultibodyPlant with a specified time step,
  // // ensures that the builder is valid, and then passes the newly created Multibodyplant 
  // // to an overloaded definition of AddMultibodyPlantSceneGraph that takes in the plant
  // // and a scene graph and adds it to the diagram.
  // // This overloaded function then returns a tuple of pointers to the plant and the scene graph.
  // auto [plant_for_lcs, scene_graph] =
  //     AddMultibodyPlantSceneGraph(&plant_builder, 0.0);

  // Parser parser_for_lcs(&plant_for_lcs);
  // parser_for_lcs.SetAutoRenaming(true);
  // /// Loading simple model of end effector (just a sphere) for the lcs plant
  // parser_for_lcs.AddModels(controller_params.end_effector_simple_model);
  // parser_for_lcs.AddModels(controller_params.jack_model);
  // parser_for_lcs.AddModels(controller_params.ground_model);	
  // // TO DO: The base link may change to the simple end effector model link name
  // // or might just be removed entirely.
	// RigidTransform<double> X_WI = RigidTransform<double>::Identity();
  // Eigen::Vector3d p_world_to_ground = sim_params.p_world_to_franka + 
  //                                     sim_params.p_franka_to_ground;
  // RigidTransform<double> X_W_G =
  //     RigidTransform<double>(drake::math::RotationMatrix<double>(),
  //                            p_world_to_ground);
  // plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
  //                          plant_for_lcs.GetFrameByName("base_link"), X_WI);
  // plant_for_lcs.WeldFrames(plant_for_lcs.world_frame(),
  //                          plant_for_lcs.GetFrameByName("ground"),
  //                          X_W_G);
  // plant_for_lcs.Finalize();
  // std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_for_lcs_autodiff =
  //     drake::systems::System<double>::ToAutoDiffXd(plant_for_lcs);

  // auto plant_diagram = plant_builder.Build();
  // std::unique_ptr<drake::systems::Context<double>> diagram_context =
  //     plant_diagram->CreateDefaultContext();
  // auto& plant_for_lcs_context = plant_diagram->GetMutableSubsystemContext(
  //     plant_for_lcs, diagram_context.get());
  // auto plant_for_lcs_context_ad = plant_for_lcs_autodiff->CreateDefaultContext();

  // drake::geometry::GeometryId ee_contact_points =
  //     plant_for_lcs.GetCollisionGeometriesForBody(
  //         plant_for_lcs.GetBodyByName("end_effector_simple"))[0];
  // drake::geometry::GeometryId capsule1_geoms =
  //     plant_for_lcs.GetCollisionGeometriesForBody(
  //         plant_for_lcs.GetBodyByName("capsule_1"))[0];
  // drake::geometry::GeometryId capsule2_geoms =
  //     plant_for_lcs.GetCollisionGeometriesForBody(
  //         plant_for_lcs.GetBodyByName("capsule_2"))[0];
  // drake::geometry::GeometryId capsule3_geoms =
  //     plant_for_lcs.GetCollisionGeometriesForBody(
  //         plant_for_lcs.GetBodyByName("capsule_3"))[0];
  // drake::geometry::GeometryId ground_geoms =
  //     plant_for_lcs.GetCollisionGeometriesForBody(
  //         plant_for_lcs.GetBodyByName("ground"))[0];

  // drake::geometry::GeometryId capsule1_sphere1_geoms =
  //     plant_for_lcs.GetCollisionGeometriesForBody(
  //         plant_for_lcs.GetBodyByName("capsule_1"))[1];
  // drake::geometry::GeometryId capsule1_sphere2_geoms =
  //     plant_for_lcs.GetCollisionGeometriesForBody(
  //         plant_for_lcs.GetBodyByName("capsule_1"))[2];
  // drake::geometry::GeometryId capsule2_sphere1_geoms =
  //     plant_for_lcs.GetCollisionGeometriesForBody(
  //         plant_for_lcs.GetBodyByName("capsule_2"))[1];
  // drake::geometry::GeometryId capsule2_sphere2_geoms =
  //     plant_for_lcs.GetCollisionGeometriesForBody(
  //         plant_for_lcs.GetBodyByName("capsule_2"))[2];
  // drake::geometry::GeometryId capsule3_sphere1_geoms =
  //     plant_for_lcs.GetCollisionGeometriesForBody(
  //         plant_for_lcs.GetBodyByName("capsule_3"))[1];
  // drake::geometry::GeometryId capsule3_sphere2_geoms =
  //     plant_for_lcs.GetCollisionGeometriesForBody(
  //         plant_for_lcs.GetBodyByName("capsule_3"))[2];

  // //   Creating a map of contact geoms
  // std::unordered_map<std::string, drake::geometry::GeometryId> contact_geoms;
  // contact_geoms["EE"] = ee_contact_points;
  // contact_geoms["CAPSULE_1"] = capsule1_geoms;
  // contact_geoms["CAPSULE_2"] = capsule2_geoms;
  // contact_geoms["CAPSULE_3"] = capsule3_geoms;
  // contact_geoms["CAPSULE_1_SPHERE_1"] = capsule1_sphere1_geoms;
  // contact_geoms["CAPSULE_1_SPHERE_2"] = capsule1_sphere2_geoms;
  // contact_geoms["CAPSULE_2_SPHERE_1"] = capsule2_sphere1_geoms;
  // contact_geoms["CAPSULE_2_SPHERE_2"] = capsule2_sphere2_geoms;
  // contact_geoms["CAPSULE_3_SPHERE_1"] = capsule3_sphere1_geoms;
  // contact_geoms["CAPSULE_3_SPHERE_2"] = capsule3_sphere2_geoms;
  // contact_geoms["GROUND"] = ground_geoms;

  // std::vector<SortedPair<GeometryId>> ee_contact_pairs;

  // //   Creating a list of contact pairs for the end effector and the jack to
  // //   hand over to lcs factory in the controller to resolve
  // ee_contact_pairs.push_back(
  //     SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_1"]));
  // ee_contact_pairs.push_back(
  //     SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_2"]));
  // ee_contact_pairs.push_back(
  //     SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_3"]));
  // //   Creating a list of contact pairs for the jack and the ground
  //   SortedPair<GeometryId> ground_contact_1_1{
  //     SortedPair(contact_geoms["CAPSULE_1_SPHERE_1"], contact_geoms["GROUND"])};
  //   SortedPair<GeometryId> ground_contact_1_2{
  //     SortedPair(contact_geoms["CAPSULE_1_SPHERE_2"], contact_geoms["GROUND"])};
  //   SortedPair<GeometryId> ground_contact_2_1{
  //     SortedPair(contact_geoms["CAPSULE_2_SPHERE_1"], contact_geoms["GROUND"])};
  //   SortedPair<GeometryId> ground_contact_2_2{
  //     SortedPair(contact_geoms["CAPSULE_2_SPHERE_2"], contact_geoms["GROUND"])};
  //   SortedPair<GeometryId> ground_contact_3_1{
  //     SortedPair(contact_geoms["CAPSULE_3_SPHERE_1"], contact_geoms["GROUND"])};
  //   SortedPair<GeometryId> ground_contact_3_2{
  //     SortedPair(contact_geoms["CAPSULE_3_SPHERE_2"], contact_geoms["GROUND"])};
  
  // std::vector<std::vector<SortedPair<GeometryId>>>
  //     contact_pairs;  // will have [[(ee,cap1), (ee,cap2), (ee_cap3)],
  //                     // [(ground,cap1)], [(ground,cap2)], [(ground,cap3)]]
  // contact_pairs.push_back(ee_contact_pairs);

  // if(c3_options.num_contacts_index == 2 || c3_options.num_contacts_index == 3){
  //   // If num_contacts_index is 2 or 3, we add an additional contact pair 
  //   // between the end effector and the ground.
  //   std::vector<SortedPair<GeometryId>> ee_ground_contact{
  //     SortedPair(contact_geoms["EE"], contact_geoms["GROUND"])};
  //   contact_pairs.push_back(ee_ground_contact);
  // }

  // std::vector<SortedPair<GeometryId>> ground_object_contact_pairs;
  // ground_object_contact_pairs.push_back(ground_contact_1_1);
  // ground_object_contact_pairs.push_back(ground_contact_1_2);
  // ground_object_contact_pairs.push_back(ground_contact_2_1);
  // ground_object_contact_pairs.push_back(ground_contact_2_2);
  // ground_object_contact_pairs.push_back(ground_contact_3_1);
  // ground_object_contact_pairs.push_back(ground_contact_3_2);
  // contact_pairs.push_back(ground_object_contact_pairs);

  // std::cout<<"num_positions: " << plant_for_lcs.num_positions() << std::endl;
  // std::cout<<"num_velocities: " << plant_for_lcs.num_velocities() << std::endl;
  // // plant_for_lcs.SetPositionsAndVelocities(plant_for_lcs_context.get(), x_lcs_actual);
  // std::cout<<"position names: " << plant_for_lcs.GetPositionNames()[0] << std::endl;
  // // cast to VectorXd since the SetPositionsAndVelocities function requires a VectorXd
  // plant_for_lcs.SetPositionsAndVelocities(&plant_for_lcs_context, x_lcs_actual.cast<double>());
  // // auto xu_ad = drake::math::InitializeAutoDiff(x_lcs_actual.cast<double>());
  // // plant_for_lcs_autodiff->SetPositionsAndVelocities(plant_for_lcs_context_ad.get(), xu_ad);

  // plant_diagram->set_name(("franka_c3_plant"));
  // DrawAndSaveDiagramGraph(*plant_diagram, "examples/jacktoy/test/franka_c3_plant_in_log_loader");

  // DiagramBuilder<double> builder;
  // auto controller = builder.AddSystem<dairlib::systems::SamplingC3Controller>(
  //     plant_for_lcs, &plant_for_lcs_context, *plant_for_lcs_autodiff,
  //     plant_for_lcs_context_ad.get(), contact_pairs, c3_options,
  //     sampling_params, true);
  // // get controller context
  // auto controller_context = controller->CreateDefaultContext();

  // auto owned_diagram = builder.Build();
  // owned_diagram->set_name(("franka_c3_controller"));

  // DrawAndSaveDiagramGraph(*owned_diagram, "examples/jacktoy/test/franka_c3_controller_in_log_loader");

  // // fix input port values
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
  

  std::cout << "Finished ForcedPublish" << std::endl;
  return 0;
}
}  // namespace dairlib

int main(int argc, char* argv[]) { return dairlib::DoMain(argc, argv); }