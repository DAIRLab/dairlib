#include <lcm/lcm-cpp.hpp>
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
#include "examples/jacktoy/parameters/franka_c3_controller_params.h"
#include "examples/jacktoy/parameters/franka_sim_params.h"
#include "systems/system_utils.h"
#include "systems/framework/timestamped_vector.h"
#include "solvers/c3_output.h"
#include "solvers/lcs_factory_preprocessor.h"
#include "solvers/lcs_factory.h"

// Uncomment this line to output cost information for evenly spaced samples.
#define DO_SAMPLE_VISUALIZATIONS

// Sample grid locations.
#define N_VERTICAL 30
#define N_HORIZONTAL 150

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
using solvers::LCSFactoryPreProcessor;


// Declare function that will generate samples around jack location.
std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::Vector2d>> 
  GenerateEvenlySpacedSamples(
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
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] <<" <log_folder> <time_into_log>" <<
      std::endl;
    return 1;
  }
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

  // (2/4) C3 parameters.
  std::string c3_gains_path = log_filepath;
  std::string c3_gains_path_replacement = "c3_gains_";
  c3_gains_path.replace(c3_gains_path.find(to_replace), to_replace.length(),
                        c3_gains_path_replacement);
  C3Options c3_options;
  c3_options = drake::yaml::LoadYamlFile<C3Options>(c3_gains_path + ".yaml");
  // NOTE:  can temporarily hard code many more ADMM iterations or other
  // changes here, e.g.:
  // c3_options.admm_iter = 8;

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
    "C3_DEBUG_CURR"};

  Eigen::VectorXd x_lcs_actual = Eigen::VectorXd::Zero(19);
  Eigen::VectorXd x_lcs_desired = Eigen::VectorXd::Zero(19);
  Eigen::VectorXd x_lcs_final_desired = Eigen::VectorXd::Zero(19);
  Eigen::MatrixXd dyn_feas_curr_plan_obj_pos = Eigen::MatrixXd::Zero(3, c3_options.N+1);
  Eigen::MatrixXd dyn_feas_curr_plan_ee_pos = Eigen::MatrixXd::Zero(3, c3_options.N+1);
  Eigen::MatrixXd dyn_feas_curr_plan_obj_orientation = Eigen::MatrixXd::Zero(4, c3_options.N+1);

  Eigen::MatrixXd u_sol = Eigen::MatrixXd::Zero(3, c3_options.N);
  Eigen::MatrixXd x_sol = Eigen::MatrixXd::Zero(19, c3_options.N);
  Eigen::MatrixXd lambda_sol = Eigen::MatrixXd::Zero(16, c3_options.N);
  Eigen::MatrixXd w_sol = Eigen::MatrixXd::Zero(38, c3_options.N);
  Eigen::MatrixXd delta_sol = Eigen::MatrixXd::Zero(38, c3_options.N);

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
  
    // Break out of loop if we have one message for every desired channel.
    if (channels_to_check.find(event->channel) != channels_to_check.end()) {
      if ((x_lcs_actual != Eigen::VectorXd::Zero(19)) &&
          (x_lcs_desired != Eigen::VectorXd::Zero(19)) &&
          (x_lcs_final_desired != Eigen::VectorXd::Zero(19)) &&
          (dyn_feas_curr_plan_ee_pos != Eigen::MatrixXd::Zero(3, c3_options.N+1)) &&
          (dyn_feas_curr_plan_obj_pos != Eigen::MatrixXd::Zero(3, c3_options.N+1)) &&
          (dyn_feas_curr_plan_obj_orientation != Eigen::MatrixXd::Zero(4, c3_options.N+1)) &&
          (u_sol != Eigen::MatrixXd::Zero(3, c3_options.N))) {
        break;
      }
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
  // TODO: The base link may change to the simple end effector model link name
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

  // Create a map of contact geoms.
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

  // Create a list of contact pairs for the end effector and the jack to hand
  // over to lcs factory in the controller to resolve.
  ee_contact_pairs.push_back(
      SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_1"]));
  ee_contact_pairs.push_back(
      SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_2"]));
  ee_contact_pairs.push_back(
      SortedPair(contact_geoms["EE"], contact_geoms["CAPSULE_3"]));
  // Create a list of contact pairs for the jack and the ground.
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

  plant_diagram->set_name(("franka_c3_plant"));
  DrawAndSaveDiagramGraph(
    *plant_diagram, "examples/jacktoy/test/franka_c3_plant_in_log_loader");

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
    sampling_params, verbose);
  auto controller_context = controller->CreateDefaultContext();

  auto owned_diagram = builder.Build();
  owned_diagram->set_name(("franka_c3_controller"));

  DrawAndSaveDiagramGraph(
    *owned_diagram, "examples/jacktoy/test/franka_c3_controller_in_log_loader");

  // Fix static input ports.
  controller->get_input_port_radio().FixValue(
    controller_context.get(), drake::Value<dairlib::lcmt_radio_out>{});
  controller->get_input_port_target().FixValue(
    controller_context.get(), x_lcs_desired);
  controller->get_input_port_final_target().FixValue(
    controller_context.get(), x_lcs_final_desired);

  std::vector<Eigen::VectorXd> x_lcs_actuals;
  x_lcs_actuals.push_back(x_lcs_actual);

  #ifdef DO_SAMPLE_VISUALIZATIONS
    std::cout<<"DO_SAMPLE_VISUALIZATIONS"<<std::endl;

    // Generate evenly spaced samples around the jack location.
    auto [x_lcs_samples, angles] = GenerateEvenlySpacedSamples(
      x_lcs_actual, sampling_params, N_VERTICAL, N_HORIZONTAL);

    x_lcs_actuals.clear();
    x_lcs_actuals = x_lcs_samples;

    // Prepare to print out the costs.
    Eigen::VectorXd costs = Eigen::VectorXd::Zero(angles.size());
  #endif

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
std::pair<std::vector<Eigen::VectorXd>, std::vector<Eigen::Vector2d>> 
  GenerateEvenlySpacedSamples(
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
  std::string filename = "/home/sharanya/workspace/dairlib/examples/jacktoy/test";
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