

#include "reference_generator.h"
#include <drake/systems/framework/diagram_builder.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <iostream>
#include <drake/systems/analysis/simulator.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/solvers/solve.h>
#include "examples/Cassie/cassie_utils.h"
#include "multibody/visualization_utils.h"

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

Eigen::VectorXd GenerateNominalStand(const drake::multibody::MultibodyPlant<double> &plant,
                                     double pelvis_height,
                                     double stance_width,
                                     bool visualize) {
  using Eigen::VectorXd;
  using Eigen::Vector3d;
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_x = n_q + n_v;
  std::map<std::string, int> positions_map = dairlib::multibody::MakeNameToPositionsMap(plant);

  Eigen::VectorXd q_ik_guess = Eigen::VectorXd::Zero(n_q);

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

  Eigen::Vector3d heel_rt_toe = {.122, 0, 0};

  Eigen::Vector3d pelvis_pos = {0,0, pelvis_height};
  Eigen::Vector3d l_toe_pos = {0.06, stance_width/2, 0};
  Eigen::Vector3d r_toe_pos = {0.06, -stance_width/2, 0};

  Eigen::Vector3d l_heel_pos = l_toe_pos - heel_rt_toe;
  Eigen::Vector3d r_heel_pos = r_toe_pos-heel_rt_toe;


  const auto& world_frame = plant.world_frame();
  const auto& pelvis_frame = plant.GetFrameByName("pelvis");
  const auto& toe_left_frame = plant.GetFrameByName("toe_left");
  const auto& toe_right_frame = plant.GetFrameByName("toe_right");

  drake::multibody::InverseKinematics ik(plant);
  double eps = 1e-3;
  Vector3d eps_vec = eps * VectorXd::Ones(3);
  ik.AddPositionConstraint(pelvis_frame, Vector3d(0, 0, 0), world_frame,
                           pelvis_pos - eps * VectorXd::Ones(3),
                           pelvis_pos + eps * VectorXd::Ones(3));
  ik.AddOrientationConstraint(pelvis_frame, drake::math::RotationMatrix<double>(),
                              world_frame, drake::math::RotationMatrix<double>(), eps);
  ik.AddPositionConstraint(toe_left_frame, dairlib::LeftToeFront(plant).first, world_frame,
                           l_toe_pos - eps_vec,
                           l_toe_pos + eps_vec);
  ik.AddPositionConstraint(toe_left_frame, dairlib::LeftToeRear(plant).first, world_frame,
                           l_heel_pos - eps_vec,
                           l_heel_pos + eps_vec);

  ik.AddPositionConstraint(toe_right_frame, dairlib::RightToeFront(plant).first, world_frame,
                           r_toe_pos - eps_vec, r_toe_pos + eps_vec);
  ik.AddPositionConstraint(toe_right_frame, dairlib::RightToeRear(plant).first, world_frame,
                           r_heel_pos - eps_vec, r_heel_pos + eps_vec);

  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("hip_yaw_left")) == 0);
  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("hip_yaw_right")) == 0);
  // Four bar linkage constraint (without spring)
  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("knee_left")) +
          (ik.q())(positions_map.at("ankle_joint_left")) ==
          M_PI * 13 / 180.0);
  ik.get_mutable_prog()->AddLinearConstraint(
      (ik.q())(positions_map.at("knee_right")) +
          (ik.q())(positions_map.at("ankle_joint_right")) ==
          M_PI * 13 / 180.0);

  ik.get_mutable_prog()->SetInitialGuess(ik.q(), q_ik_guess);
  const auto result = drake::solvers::Solve(ik.prog());
  const auto q_sol = result.GetSolution(ik.q());
  VectorXd q_sol_normd(n_q);
  q_sol_normd << q_sol.head(4).normalized(), q_sol.tail(n_q - 4);
  q_ik_guess = q_sol_normd;

  if(visualize){
    // Build temporary diagram for visualization
    drake::systems::DiagramBuilder<double> builder_ik;
    drake::geometry::SceneGraph<double>& scene_graph_ik = *builder_ik.AddSystem<drake::geometry::SceneGraph>();
    scene_graph_ik.set_name("scene_graph_ik");
    MultibodyPlant<double> plant_ik(0.0);
    Parser parser(&plant_ik, &scene_graph_ik);
    std::string full_name =
        dairlib::FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
    parser.AddModelFromFile(full_name);
    plant_ik.Finalize();

    // Visualize
    VectorXd x_const = VectorXd::Zero(n_x);
    x_const.head(n_q) = q_sol;
    drake::trajectories::PiecewisePolynomial<double> pp_xtraj(x_const);

    dairlib::multibody::ConnectTrajectoryVisualizer(&plant_ik, &builder_ik,
                                                    &scene_graph_ik, pp_xtraj);
    auto diagram = builder_ik.Build();
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(.1);
    simulator.Initialize();
    simulator.AdvanceTo(1.0);
  }

  Eigen::VectorXd rv = Eigen::VectorXd::Zero(n_x);
  rv.head(n_q) = q_ik_guess;
  return rv;
}

drake::trajectories::PiecewisePolynomial<double> GenerateComTrajectory(const Eigen::Vector3d& current_com,
                                                              const std::vector<Eigen::Vector3d>& vel_ewrt_w,
                                                              const std::vector<double>& time_points){
  DRAKE_DEMAND(vel_ewrt_w.size() == time_points.size());
  auto n_points = vel_ewrt_w.size();

  std::vector<drake::MatrixX<double>> samples(n_points);
  samples[0] = current_com;
  for(int i = 1; i<n_points; i++){
    samples[i] = samples[i-1] + (time_points[i] - time_points[i-1]) * vel_ewrt_w[i-1];
  }
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(time_points, samples);
}

drake::trajectories::PiecewisePolynomial<double> GenerateGeneralizedStateTrajectory(const Eigen::VectorXd& nominal_stand,
                                                                                    const Eigen::Vector3d& base_rt_com_ewrt_w,
                                                                                    const drake::trajectories::PiecewisePolynomial<double>& com_traj,
                                                                                    const std::vector<double>& time_points,
                                                                                    int base_pos_start,
                                                                                    int base_vel_start){
  auto n_points = time_points.size();
  std::vector<drake::MatrixX<double>> samples(n_points);
  for(int i = 0; i < n_points; i++){
    samples[i] = nominal_stand;
    samples[i].block<3,1>(base_pos_start,0, 3, 1) = com_traj.value(time_points[i]);
    samples[i].block<3,1>(base_vel_start,0, 3, 1) = com_traj.derivative().value(time_points[i]);
  }
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(time_points, samples);
}

int FindCurrentMode(const Gait& active_gait, double time_now){
  double phase_now = fmod(time_now/active_gait.period, 1);
  for(int i = 0; i < active_gait.gait_pattern.size(); i++){
    const auto& mode = active_gait.gait_pattern[i];
    if(mode.start_phase <= phase_now){
      return i;
    }
  }
  DRAKE_ASSERT(false);
  return 0;
}

drake::trajectories::PiecewisePolynomial<double> GenerateModeSequence(const std::vector<Gait>& gait_sequence,
                                                                      const std::vector<double>& time_points){
  DRAKE_DEMAND(gait_sequence.size() == time_points.size());

  auto traj = gait_sequence[0].ToTrajectory(time_points[0], time_points[1]);
  for(int i = 1; i<gait_sequence.size() - 1 ; i++){
    traj.ConcatenateInTime(gait_sequence[i].ToTrajectory(time_points[i], time_points[i + 1]));
  }
  return traj;
}

drake::trajectories::PiecewisePolynomial<double> Gait::ToTrajectory(double current_time, double end_time) const {
  std::vector<double> break_points;
  std::vector<drake::MatrixX<double>> samples;

  // Calculate initial mode index, and phase
  int current_mode = FindCurrentMode(*this, current_time);
  double current_phase = fmod(current_time/period, 1);

  // Loop until time is greater than end time
  while(current_time < end_time){
    // Add the break point for the current time
    break_points.push_back(current_time);
    samples.emplace_back(gait_pattern[current_mode].contact_status.cast<double>());

    // Update time based on how much longer the current mode will last
    current_time += (gait_pattern[current_mode].end_phase - current_phase) * period;
    // Update the mode and mod if necessary
    current_mode = (current_mode + 1) % gait_pattern.size();
    // The new phase is the start phase of the updated mode
    current_phase = gait_pattern[current_mode].start_phase;
  }
  return drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(break_points, samples);
}

double TimeNextValue(const drake::trajectories::PiecewisePolynomial<double>& trajectory,
                     double current_time,
                     int index,
                     double value){
  if (trajectory.value(current_time).coeff(index) == value)
    return current_time;

  const auto& segment_times = trajectory.get_segment_times();

  for(int i = trajectory.get_segment_index(current_time) + 1; i<segment_times.size(); i++){
    if(trajectory.value(segment_times[i]).coeff(i) == value)
      return segment_times[i];
  }
  return trajectory.end_time();
}

drake::trajectories::PiecewisePolynomial<double> GenerateGrfReference(const drake::trajectories::PiecewisePolynomial<double>& mode_trajectory,
                                                                      double m){
  std::vector<drake::MatrixX<double>> samples;
  const int n_contact_points = mode_trajectory.rows();

  for(const auto& time: mode_trajectory.get_segment_times()){
    const auto& mode = mode_trajectory.value(time);
    double num_in_contact = mode.sum();
    auto& grf = samples.emplace_back(Eigen::VectorXd::Zero(3 * n_contact_points));
    for(int i = 0; i<n_contact_points; i++){
      grf.coeffRef(2 + 3 * i) = m * 9.81 /num_in_contact;
    }
  }
  return drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(mode_trajectory.get_segment_times(), samples);
}

drake::trajectories::PiecewisePolynomial<double> GenerateContactPointReference(const drake::multibody::MultibodyPlant<double> &plant,
                                                                               const std::vector<dairlib::multibody::WorldPointEvaluator<
                                                                                   double>> &contacts,
                                                                               const drake::trajectories::PiecewisePolynomial<
                                                                                   double> &state_trajectory){
  auto context =  plant.CreateDefaultContext();
  std::vector<double> break_points = state_trajectory.get_segment_times();
  break_points.emplace_back(state_trajectory.end_time());
  std::vector<drake::MatrixX<double>> samples;
  int n_contact = contacts.size();
  for(const auto& time : break_points){
    dairlib::multibody::SetPositionsAndVelocitiesIfNew<double>(plant, state_trajectory.value(time), context.get());
    drake::VectorX<double> knot_point_value = samples.emplace_back(Eigen::VectorXd::Zero(6 * n_contact));
    for(int i = 0; i <n_contact; i++){
      knot_point_value.segment(i * 3, i * 3 + 3) = contacts[i].EvalFull(*context);
      knot_point_value.segment(3 * n_contact + i * 3, i * 3 + 3) = contacts[i].EvalFullJacobianDotTimesV(*context);
    }
  }
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(break_points, samples);
}