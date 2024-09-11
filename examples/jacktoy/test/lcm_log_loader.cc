#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include "dairlib/lcmt_c3_state.hpp"
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

using drake::multibody::MultibodyPlant;
using drake::systems::DiagramBuilder;
using drake::multibody::AddMultibodyPlantSceneGraph;
using dairlib::systems::TimestampedVector;
using drake::multibody::Parser;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::RigidTransform;

int main(int argc,  char* argv[]) {
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

  // Set the start time
  const int64_t u_time_into_log = time_into_log*1e6;

  std::cout << "time into log: " << time_into_log << std::endl;
  std::cout << " in useconds: " << u_time_into_log << std::endl;


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

  // Keep track of the initial timestamp in the log.
  int64_t u_init_time = 0;

  const lcm::LogEvent* event;

  log_file.seekToTimestamp(u_time_into_log + u_init_time);
  int64_t t = log_file.readNextEvent()->timestamp;
  log_file.seekToTimestamp(u_time_into_log + u_init_time);
  event = log_file.readNextEvent();

  // Prepare to grab the actual LCS state and the desired LCS state.
  Eigen::VectorXf x_lcs_actual = Eigen::VectorXf::Zero(19);
  Eigen::VectorXf x_lcs_desired = Eigen::VectorXf::Zero(19);
  Eigen::VectorXf x_lcs_final_desired = Eigen::VectorXf::Zero(19);
  Eigen::MatrixXf dyn_feas_curr_plan_pos = Eigen::MatrixXf::Zero(3, c3_options.N+1);
  Eigen::MatrixXf dyn_feas_curr_plan_orientation = Eigen::MatrixXf::Zero(4, c3_options.N+1);
  Eigen::MatrixXf u_sol = Eigen::MatrixXf::Zero(3, c3_options.N);
    
  // Read the log until the first C3_ACTUAL message after the start time.
  while ((event = log_file.readNextEvent()) != nullptr) {
    if (u_init_time == 0) {
      u_init_time = event->timestamp;
    }

    if (event->channel == "C3_ACTUAL") {
      if(event->timestamp > u_time_into_log + u_init_time) {
        dairlib::lcmt_c3_state message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received C3_ACTUAL message in seconds: " << (event->timestamp - u_init_time)/1e6<< std::endl;
          for (int i=0; i<19; i++) {
            x_lcs_actual(i) = message.state[i];
          }
          if ((x_lcs_desired != Eigen::VectorXf::Zero(19)) &&
              (x_lcs_final_desired != Eigen::VectorXf::Zero(19)) &&
              (dyn_feas_curr_plan_pos != Eigen::MatrixXf::Zero(3, c3_options.N+1)) &&
              (dyn_feas_curr_plan_orientation != Eigen::MatrixXf::Zero(4, c3_options.N+1)) &&
              (u_sol != Eigen::MatrixXf::Zero(3, c3_options.N))) {
            break;
          }
        } else {
          std::cerr << "Failed to decode C3_ACTUALs message" << std::endl;
        }
      }
    }
    else if (event->channel == "C3_TARGET") {
      if(event->timestamp > u_time_into_log + u_init_time) {
        dairlib::lcmt_c3_state message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received C3_TARGET message at: " << (event->timestamp - u_init_time)/1e6 << std::endl;
          for (int i=0; i<19; i++) {
            x_lcs_desired(i) = message.state[i];
          }
          if ((x_lcs_actual != Eigen::VectorXf::Zero(19)) &&
              (x_lcs_final_desired != Eigen::VectorXf::Zero(19)) &&
              (dyn_feas_curr_plan_pos != Eigen::MatrixXf::Zero(3, c3_options.N+1)) &&
              (dyn_feas_curr_plan_orientation != Eigen::MatrixXf::Zero(4, c3_options.N+1))) {
            break;
          }
        } else {
          std::cerr << "Failed to decode C3_TARGETs message" << std::endl;
        }
      }
    }
    else if (event->channel == "C3_FINAL_TARGET") {
      if(event->timestamp > u_time_into_log + u_init_time) {
        dairlib::lcmt_c3_state message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received C3_FINAL_TARGET message at: " << (event->timestamp- u_init_time)/1e6 << std::endl;
          for (int i=0; i<19; i++) {
            x_lcs_final_desired(i) = message.state[i];
          }
          if ((x_lcs_actual != Eigen::VectorXf::Zero(19)) &&
              (x_lcs_desired != Eigen::VectorXf::Zero(19)) &&
              (dyn_feas_curr_plan_pos != Eigen::MatrixXf::Zero(3, c3_options.N+1)) &&
              (dyn_feas_curr_plan_orientation != Eigen::MatrixXf::Zero(4, c3_options.N+1)) &&
              (u_sol != Eigen::MatrixXf::Zero(3, c3_options.N))) {
            break;
          }
        } else {
          std::cerr << "Failed to decode C3_FINAL_TARGETs message" << std::endl;
        }
      }
    }
    else if (event->channel == "DYNAMICALLY_FEASIBLE_CURR_PLAN") {
      if(event->timestamp > u_time_into_log + u_init_time) {
        dairlib::lcmt_timestamped_saved_traj message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received DYNAMICALLY_FEASIBLE_CURR_PLAN message at: " <<
            (event->timestamp- u_init_time)/1e6 << std::endl;
          for (int i=0; i<4; i++) {
            for (int j=0; j<c3_options.N+1; j++) {
              dyn_feas_curr_plan_orientation(i,j) = message.saved_traj.trajectories[0].datapoints[i][j];
            }
          }
          for (int i=0; i<3; i++) {
            for (int j=0; j<c3_options.N+1; j++) {
              dyn_feas_curr_plan_pos(i,j) = message.saved_traj.trajectories[1].datapoints[i][j];
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
    else if (event->channel == "C3_DEBUG_CURR") {
      if(event->timestamp > u_time_into_log + u_init_time) {
        dairlib::lcmt_c3_output message;
        if (message.decode(event->data, 0, event->datalen) > 0) {
          std::cout << "Received C3_DEBUG_CURR message at: " <<
            (event->timestamp- u_init_time)/1e6 << std::endl;
          for (int i=0; i<3; i++) {
            for (int j=0; j<c3_options.N+1; j++) {
              u_sol(i,j) = message.c3_solution.u_sol[i][j];
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
  
  std::cout << "\nFuture timestamps:" << std::endl;
  int more_msgs_to_print = 5;
  while(((event = log_file.readNextEvent()) != nullptr) && (more_msgs_to_print > 0)) {
    if (event->channel == "C3_ACTUAL") {
      std::cout << "Received C3_ACTUAL message in seconds: " << (event->timestamp - u_init_time)/1e6<< std::endl;
      more_msgs_to_print--;
    }
  }

  std::cout << "\nFound these states:" << std::endl;
  std::cout << "Actual: " << x_lcs_actual.transpose() << std::endl;
  std::cout << "Desired: " << x_lcs_desired.transpose() << std::endl;
  std::cout << "Final Desired: " << x_lcs_final_desired.transpose() << std::endl;
  std::cout << "\nDyn Feas Curr plan from log:\n" << dyn_feas_curr_plan_pos << std::endl;
  std::cout << "\nDyn Feas Curr plan orientation from log:\n" << dyn_feas_curr_plan_orientation << std::endl;
  std::cout << "\nU_sol from log:\n" << u_sol << std::endl;

  // Create the plant for the LCS
  DiagramBuilder<double> plant_builder;
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
                      // [(ground,cap1)], [(ground,cap2)], [(ground,cap3)]]
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

  std::cout<<"num_positions: " << plant_for_lcs.num_positions() << std::endl;
  std::cout<<"num_velocities: " << plant_for_lcs.num_velocities() << std::endl;
  // plant_for_lcs.SetPositionsAndVelocities(plant_for_lcs_context.get(), x_lcs_actual);
  std::cout<<"position names: " << plant_for_lcs.GetPositionNames()[0] << std::endl;
  // cast to VectorXd since the SetPositionsAndVelocities function requires a VectorXd
  plant_for_lcs.SetPositionsAndVelocities(&plant_for_lcs_context, x_lcs_actual.cast<double>());


  DiagramBuilder<double> builder;
  auto controller = builder.AddSystem<dairlib::systems::SamplingC3Controller>(
      plant_for_lcs, &plant_for_lcs_context, *plant_for_lcs_autodiff,
      plant_for_lcs_context_ad.get(), contact_pairs, c3_options,
      sampling_params, true);
  // get controller context
  auto controller_context = controller->CreateDefaultContext();
  // fix input port values
  controller->get_input_port_radio().FixValue(controller_context.get(), drake::Value<dairlib::lcmt_radio_out>{});
  std::cout<<"input port size: "<<controller->get_input_port_lcs_state().size()<<std::endl;
  controller->get_input_port_lcs_state().FixValue(controller_context.get(),  TimestampedVector<double>(x_lcs_actual.cast<double>()));
  controller->get_input_port_target().FixValue(controller_context.get(), x_lcs_desired.cast<double>());
  controller->get_input_port_final_target().FixValue(controller_context.get(), x_lcs_final_desired.cast<double>());

  auto discrete_state = controller->AllocateDiscreteVariables();
  controller->CalcForcedDiscreteVariableUpdate(*controller_context, discrete_state.get());

  std::cout << "Finished ForcedPublish" << std::endl;
  return 0;
}