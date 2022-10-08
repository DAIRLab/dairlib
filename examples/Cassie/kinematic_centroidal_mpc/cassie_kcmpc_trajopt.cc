
#include <drake/systems/framework/diagram_builder.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <iostream>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/systems/rendering/multibody_position_to_geometry_pose.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/geometry/drake_visualizer.h>
#include "common/find_resource.h"
#include "systems/controllers/kinematic_centroidal_mpc/kinematic_centroidal_mpc.h"
#include "examples/Cassie/cassie_utils.h"
#include "systems/primitives/subvector_pass_through.h"

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::geometry::DrakeVisualizer;

Eigen::VectorXd GenerateNominalStand(const drake::multibody::MultibodyPlant<double>& plant){
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_x = n_q + n_v;
  std::map<std::string, int> positions_map = dairlib::multibody::MakeNameToPositionsMap(plant);

  Eigen::VectorXd q_ik_guess = Eigen::VectorXd::Zero(n_x);

  std::map<std::string, double> pos_value_map;
  Eigen::Vector4d quat(2000.06, -0.339462, -0.609533, -0.760854);
  quat.normalize();
  pos_value_map["base_qw"] = quat(0);
  pos_value_map["base_qx"] = quat(1);
  pos_value_map["base_qy"] = quat(2);
  pos_value_map["base_qz"] = quat(3);
  pos_value_map["base_x"] = 0.000889849;
  pos_value_map["base_y"] = 0.000626865;
  pos_value_map["base_z"] = 1.0009;
  pos_value_map["hip_roll_left"] = 0.00927845;
  pos_value_map["hip_roll_right"] = 0.00927845;
  pos_value_map["hip_yaw_left"] = -0.000895805;
  pos_value_map["hip_yaw_right"] = 0.000895805;
  pos_value_map["hip_pitch_left"] = 0.610808;
  pos_value_map["hip_pitch_right"] = 0.610808;
  pos_value_map["knee_left"] = -1.35926;
  pos_value_map["knee_right"] = -1.35926;
  pos_value_map["ankle_joint_left"] = 1.00716;
  pos_value_map["ankle_joint_right"] = 1.00716;
  pos_value_map["toe_left"] = -M_PI / 2;
  pos_value_map["toe_right"] = -M_PI / 2;

  for (auto pair : pos_value_map) {
    q_ik_guess(positions_map.at(pair.first)) = pair.second;
  }
  return q_ik_guess;
}


void DoMain(int n_knot_points, double duration, double com_height, double tol){
  // Create fix-spring Cassie MBP
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);
  MultibodyPlant<double> plant_vis(0.0);

  Parser parser(&plant);
  Parser parser_vis(&plant_vis, &scene_graph);

  std::string full_name =
      dairlib::FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);
  plant.Finalize();
  plant_vis.Finalize();
  std::map<std::string, int> positions_map = dairlib::multibody::MakeNameToPositionsMap(plant);

  auto left_toe_pair = dairlib::LeftToeFront(plant);
  auto left_heel_pair = dairlib::LeftToeRear(plant);
  auto right_toe_pair = dairlib::RightToeFront(plant);
  auto right_heel_pair = dairlib::RightToeRear(plant);

  std::vector<int> toe_active_inds{0, 1, 2};
  std::vector<int> heel_active_inds{0, 1, 2};

  auto left_toe_eval = std::make_shared<dairlib::multibody::WorldPointEvaluator<double>>(
      plant, left_toe_pair.first, left_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), toe_active_inds);

  auto left_heel_eval = std::make_shared<dairlib::multibody::WorldPointEvaluator<double>>(
      plant, left_heel_pair.first, left_heel_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), heel_active_inds);

  auto right_toe_eval = std::make_shared<dairlib::multibody::WorldPointEvaluator<double>>(
      plant, right_toe_pair.first, right_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), toe_active_inds);

  auto right_heel_eval = std::make_shared<dairlib::multibody::WorldPointEvaluator<double>>(
      plant, right_heel_pair.first, right_heel_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), heel_active_inds);

  std::cout<<"Creating MPC"<<std::endl;
  KinematicCentroidalMPC mpc (plant, n_knot_points, duration/(n_knot_points-1),
                              {left_toe_eval, left_heel_eval, right_toe_eval, right_heel_eval});

  // create joint/motor names
  std::vector<std::pair<std::string, std::string>> l_r_pairs{
      std::pair<std::string, std::string>("_left", "_right"),
      std::pair<std::string, std::string>("_right", "_left"),
  };
  std::vector<std::string> asy_joint_names{
      "hip_roll",
      "hip_yaw",
  };
  std::vector<std::string> sym_joint_names{"hip_pitch", "knee", "ankle_joint", "toe"};
  std::vector<std::string> joint_names{};
  std::vector<std::string> motor_names{};
  for (auto &l_r_pair : l_r_pairs) {
    for (unsigned int i = 0; i < asy_joint_names.size(); i++) {
      joint_names.push_back(asy_joint_names[i] + l_r_pair.first);
      motor_names.push_back(asy_joint_names[i] + l_r_pair.first + "_motor");
    }
    for (unsigned int i = 0; i < sym_joint_names.size(); i++) {
      joint_names.push_back(sym_joint_names[i] + l_r_pair.first);
      if (sym_joint_names[i].compare("ankle_joint") != 0) {
        motor_names.push_back(sym_joint_names[i] + l_r_pair.first + "_motor");
      }
    }
  }
  mpc.AddPlantJointLimits(joint_names);

  std::cout<<"Setting initial guess"<<std::endl;
  mpc.SetZeroInitialGuess();

  double cost_force = 0.01;

  double cost_joint_pos = 0.0005;
  double cost_joint_vel = 0.0001;

  double cost_contact_pos = 1;
  double cost_contact_vel = 2;

  double cost_com_pos = 10;
  double cost_com_vel = 0.001;
  double cost_com_orientation = 8;
  double cost_angular_vel = 0.01;

  double stance_width = 0.25;
  double stance_wiggle = 0.01;

  Eigen::Vector3d left_lb(std::numeric_limits<double>::lowest(), -stance_width/2-stance_wiggle, std::numeric_limits<double>::lowest());
  Eigen::Vector3d left_ub(std::numeric_limits<double>::max(), -stance_width/2+stance_wiggle, std::numeric_limits<double>::max());

  Eigen::Vector3d right_lb(std::numeric_limits<double>::lowest(), stance_width/2-stance_wiggle, std::numeric_limits<double>::lowest());
  Eigen::Vector3d right_ub(std::numeric_limits<double>::max(), stance_width/2+stance_wiggle, std::numeric_limits<double>::max());
  mpc.AddContactPointPositionConstraint(0, left_lb, left_ub);
  mpc.AddContactPointPositionConstraint(1, left_lb, left_ub);
  mpc.AddContactPointPositionConstraint(2, right_lb, right_ub);
  mpc.AddContactPointPositionConstraint(3, right_lb, right_ub);

  mpc.AddConstantForceTrackingReference(Eigen::VectorXd::Zero(12), cost_force * Eigen::MatrixXd::Identity(12,12));

  Eigen::VectorXd reference_state = GenerateNominalStand(plant);

  Eigen::VectorXd Q_state = Eigen::VectorXd::Zero(plant.num_positions() + plant.num_velocities());
  Q_state.segment(7, plant.num_positions()-7) = cost_joint_pos * Eigen::VectorXd::Ones(plant.num_positions()-7);
  Q_state.tail(plant.num_velocities() - 6) = cost_joint_vel * Eigen::VectorXd::Ones(plant.num_velocities() - 6);
  mpc.AddConstantStateReference(reference_state,Q_state.asDiagonal());

  Eigen::VectorXd reference_cent_state = Eigen::VectorXd::Zero(13);
  reference_cent_state[0] = 1;
  reference_cent_state[6] = com_height;

  Eigen::VectorXd reference_cent_state_bottom = Eigen::VectorXd::Zero(13);
  reference_cent_state_bottom[0] = 1;
  reference_cent_state_bottom[6] = com_height-0.5;
  std::vector<double> time_points = {0, duration};
  auto centroidal_reference = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(time_points,
                                                                                               {reference_cent_state,
                                                                                                reference_cent_state_bottom});

  Eigen::VectorXd Q_cent(13);
  Q_cent.head(4) = cost_com_orientation * Eigen::VectorXd::Ones(4);
  Q_cent.segment(4,3) = cost_com_pos * Eigen::VectorXd::Ones(3);
  Q_cent.segment(7,3) = cost_angular_vel * Eigen::VectorXd::Ones(3);
  Q_cent.segment(10,3) = cost_com_vel * Eigen::VectorXd::Ones(3);
  mpc.AddCentroidalReference(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(centroidal_reference),Q_cent.asDiagonal());

  Eigen::VectorXd Q_contact = cost_contact_pos * Eigen::VectorXd::Ones(4 * 6);
  Q_contact.tail(4 * 3) = cost_contact_vel * Eigen::VectorXd::Ones(4 * 3);
  mpc.AddContactTrackingReference(std::make_unique<drake::trajectories::PiecewisePolynomial<double>>(Eigen::VectorXd::Zero(
      4 * 6)), Q_contact.asDiagonal());

  std::cout<<"Adding solver options"<<std::endl;
  {
    drake::solvers::SolverOptions options;
    auto id = drake::solvers::IpoptSolver::id();
    options.SetOption(id, "tol", tol);
    options.SetOption(id, "dual_inf_tol", tol);
    options.SetOption(id, "constr_viol_tol", tol);
    options.SetOption(id, "compl_inf_tol", tol);
    options.SetOption(id, "max_iter", 500);
    options.SetOption(id, "nlp_lower_bound_inf", -1e6);
    options.SetOption(id, "nlp_upper_bound_inf", 1e6);
    options.SetOption(id, "print_timing_statistics", "yes");
    options.SetOption(id, "print_level", 5);

    // Set to ignore overall tolerance/dual infeasibility, but terminate when
    // primal feasible and objective fails to increase over 5 iterations.
    options.SetOption(id, "acceptable_compl_inf_tol", tol);
    options.SetOption(id, "acceptable_constr_viol_tol", tol);
    options.SetOption(id, "acceptable_obj_change_tol", 1e-3);
    options.SetOption(id, "acceptable_tol", 1e2);
    options.SetOption(id, "acceptable_iter", 5);
    mpc.Build(options);
  }

  std::cout<<"Adding visualization callback"<<std::endl;
  double alpha = .2;
  mpc.CreateVisualizationCallback(
      "examples/Cassie/urdf/cassie_fixed_springs.urdf", alpha);

  std::cout << "Solving DIRCON\n\n";
  const auto pp_xtraj = mpc.Solve();

  auto traj_source =
      builder.AddSystem<drake::systems::TrajectorySource>(pp_xtraj);
  auto passthrough = builder.AddSystem<dairlib::systems::SubvectorPassThrough>(
      plant.num_positions() + plant.num_velocities(), 0, plant.num_positions());
  builder.Connect(traj_source->get_output_port(),
                  passthrough->get_input_port());
  auto to_pose =
      builder.AddSystem<drake::systems::rendering::MultibodyPositionToGeometryPose<double>>(plant_vis);
  builder.Connect(passthrough->get_output_port(), to_pose->get_input_port());

  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant_vis.get_source_id().value()));

  DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);
  auto diagram = builder.Build();

  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(.4);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
  }

}

int main(int argc, char* argv[]) {
  DoMain(12, 0.5, 1.9,1e-4);
}
