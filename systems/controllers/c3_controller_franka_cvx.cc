#include "c3_controller_franka_cvx.h"

#include <utility>
#include <chrono>


#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/common/sorted_pair.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/multibody/plant/multibody_plant.h"
#include "multibody/multibody_utils.h"
#include "solvers/c3.h"
#include "solvers/c3_miqp.h"
#include "solvers/c3_approx.h"
#include "solvers/lcs_factory.h"

#include "drake/solvers/moby_lcp_solver.h"
#include "multibody/geom_geom_collider.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "solvers/lcs_factory_cvx.h"
#include "drake/math/autodiff_gradient.h"

using std::vector;

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::multibody::JacobianWrtVariable;
using drake::math::RotationMatrix;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Quaterniond;

// task space helper function that generates the target trajectory
// modified version of code from impedance_controller.cc
std::vector<Eigen::Vector3d> move_to_initial_position(
    const Eigen::Vector3d& start, const Eigen::Vector3d& finish,
    double curr_time, double stabilize_time1,
    double move_time, double stabilize_time2);

namespace dairlib {
namespace systems {
namespace controllers {

C3Controller_franka_Convex::C3Controller_franka_Convex(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::multibody::MultibodyPlant<double>& plant_f,
    const drake::multibody::MultibodyPlant<double>& plant_franka,
    drake::systems::Context<double>& context,
    drake::systems::Context<double>& context_f,
    drake::systems::Context<double>& context_franka,
    const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad_f,
    drake::systems::Context<drake::AutoDiffXd>& context_ad,
    drake::systems::Context<drake::AutoDiffXd>& context_ad_f,
    const drake::geometry::SceneGraph<double>& scene_graph,
    const drake::systems::Diagram<double>& diagram,
    std::vector<drake::geometry::GeometryId> contact_geoms,
    int num_friction_directions, double mu, const vector<MatrixXd>& Q,
    const vector<MatrixXd>& R, const vector<MatrixXd>& G,
    const vector<MatrixXd>& U, const vector<VectorXd>& xdesired, const drake::trajectories::PiecewisePolynomial<double>& pp, const int num_balls)
    : plant_(plant),
      plant_f_(plant_f),
      plant_franka_(plant_franka),
      context_(context),
      context_f_(context_f),
      context_franka_(context_franka),
      plant_ad_(plant_ad),
      plant_ad_f_(plant_ad_f),
      context_ad_(context_ad),
      context_ad_f_(context_ad_f),
      scene_graph_(scene_graph),
      diagram_(diagram),
      contact_geoms_(contact_geoms),
      num_friction_directions_(num_friction_directions),
      mu_(mu),
      Q_(Q),
      R_(R),
      G_(G),
      U_(U),
      xdesired_(xdesired),
      pp_(pp),
      time_horizon_(5),
      num_balls_(num_balls){



  /// number of balls in simulation
  int num_ee_xyz_pos = 3;
  int num_ee_xyz_vel = 3;
  int num_ee_orientation = 4;
  int num_visualizer_states = 9;
  //output [ (finger) end effector next state (x,y,z), (finger) end effector orientation (generated via heuristic), ball positions , finger + ball velocities, force_desired, num_visualizer_states  ]
  num_output_ = num_ee_xyz_pos + num_ee_orientation + num_ee_xyz_vel + 4*num_balls_ + 1;


  /// declare the input port ( franka + balls state, franka + balls velocities, franka + balls inputs = franka inputs )
  state_input_port_ =
      this->DeclareVectorInputPort(
              "x, u, t",
              OutputVector<double>(plant_franka_.num_positions(), plant_franka_.num_velocities(), plant_franka_.num_actuators()))
          .get_index();


  /// output [ (finger) end effector next state (x,y,z), (finger) end effector orientation (generated via heuristic), ball positions , finger + ball velocities, force_desired, num_visualizer_states  ]
  state_output_port_ = this->DeclareVectorOutputPort(
          "xee, xball, xee_dot, xball_dot, lambda, visualization",
          TimestampedVector<double>(num_output_), &C3Controller_franka_Convex::CalcControl)
      .get_index();

  q_map_franka_ = multibody::makeNameToPositionsMap(plant_franka_);
  v_map_franka_ = multibody::makeNameToVelocitiesMap(plant_franka_);
  q_map_ = multibody::makeNameToPositionsMap(plant_);
  v_map_ = multibody::makeNameToVelocitiesMap(plant_);

  // get c3_parameters
  // TODO: file should be an argument
  param_ = drake::yaml::LoadYamlFile<C3Parameters>(
      "examples/franka_trajectory_following/parameters.yaml");
  max_desired_velocity_ = param_.velocity_limit;

  // filter info
  prev_timestamp_ = 0;
  dt_filter_length_ = param_.dt_filter_length;
}

void C3Controller_franka_Convex::CalcControl(const Context<double>& context,
                                      TimestampedVector<double>* state_contact_desired) const {

  // for timing the code
  auto t_start = std::chrono::high_resolution_clock::now();

  // get values ( franka + balls state, franka + balls velocities, franka + balls inputs = franka inputs )
  auto robot_output = (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
  double timestamp = robot_output->get_timestamp();


  if (!received_first_message_){
    received_first_message_ = true;
    first_message_time_ = timestamp;
  }

  // parse some useful values
  double roll_phase = param_.roll_phase;
  double return_phase = param_.return_phase;
  double period = roll_phase + return_phase;
  double settling_time = param_.stabilize_time1 + param_.move_time + param_.stabilize_time2 + first_message_time_;
  double x_c = param_.x_c;
  double y_c = param_.y_c;
  double traj_radius = param_.traj_radius;
  double ball_radius = param_.ball_radius;
  double table_offset = param_.table_offset;

  if (timestamp <= settling_time){
    // move to initial position
    Eigen::Vector3d start = param_.initial_start;
    Eigen::Vector3d finish = param_.initial_finish;
    finish(0) = x_c + traj_radius * sin(param_.phase * PI/ 180) + finish(0);
    finish(1) = y_c + traj_radius * cos(param_.phase * PI / 180) + finish(1);
    std::vector<Eigen::Vector3d> target = move_to_initial_position(start, finish, timestamp,
                                                                   param_.stabilize_time1 + first_message_time_,
                                                                   param_.move_time, param_.stabilize_time2);

    //Eigen::Quaterniond orientation_d(0, 1, 0, 0);

    Eigen::Quaterniond default_quat(0, 1, 0, 0);
    RotationMatrix<double> Rd(default_quat);

    double duration = settling_time - first_message_time_;
    double t = timestamp - first_message_time_;

    RotationMatrix<double> rot_y = RotationMatrix<double>::MakeYRotation((t / duration) * param_.orientation_degrees * 3.14 / 180);
    Eigen::Quaterniond orientation_d = (Rd * rot_y).ToQuaternion();

    // fill st_desired for debug purposes: (state_next.head(3), orientation_d(4), ball_position(7), finger_velocity(3), ball_velocity(6), force_des.head(6), ball_xyz_d, ball_xyz, true_ball_xyz)
    VectorXd traj = pp_.value(timestamp);
    VectorXd st_desired = VectorXd::Zero(num_output_);
    // desired finger (ee) x,y,z
    st_desired.head(3) << target[0];
    // desired finger (ee) orientation
    st_desired.segment(3, 4) << orientation_d.w(), orientation_d.x(), orientation_d.y(), orientation_d.z();
    // ball position (x,y,z) NOT USED (COMMENTED OUT)
    //st_desired.segment(11, 3) << finish(0), finish(1), ball_radius + table_offset;
    // finger (ee) velocity
    st_desired.segment(7, 3) << target[1];
    // visualization
//    st_desired.segment(32, 3) << finish(0), finish(1), ball_radius + table_offset;
//    // visualization
//    st_desired.tail(3) << finish(0), finish(1), ball_radius + table_offset;

    state_contact_desired->SetDataVector(st_desired);
    state_contact_desired->set_timestamp(timestamp);
    prev_timestamp_ = (timestamp);

    return;
  }

  /// FK

  // update context once for FK
  plant_franka_.SetPositions(&context_franka_, robot_output->GetPositions());
  plant_franka_.SetVelocities(&context_franka_, robot_output->GetVelocities());
  Vector3d EE_offset_ = param_.EE_offset;
  const drake::math::RigidTransform<double> H_mat =
      plant_franka_.EvalBodyPoseInWorld(context_franka_, plant_franka_.GetBodyByName("panda_link10"));
  const RotationMatrix<double> R_current = H_mat.rotation();
  Vector3d end_effector = H_mat.translation() + R_current*EE_offset_;

  // jacobian and end_effector_dot
  auto EE_frame_ = &plant_franka_.GetBodyByName("panda_link10").body_frame();
  auto world_frame_ = &plant_franka_.world_frame();
  MatrixXd J_fb (6, plant_franka_.num_velocities());
  plant_franka_.CalcJacobianSpatialVelocity(
      context_franka_, JacobianWrtVariable::kV,
      *EE_frame_, EE_offset_,
      *world_frame_, *world_frame_, &J_fb);
  MatrixXd J_franka = J_fb.block(0, 0, 6, 7);
  VectorXd end_effector_dot = ( J_franka * (robot_output->GetVelocities()).head(7) ).tail(3);

  /// ensure that ALL state variables derive from q_plant and v_plant to ensure that noise is added EVERYWHERE!
  VectorXd q_plant = robot_output->GetPositions();
  VectorXd v_plant = robot_output->GetVelocities();
  Vector3d true_ball_xyz = q_plant.tail(3);    // extract true state for visualization purposes only
  q_plant.tail(3) << ProjectStateEstimate(end_effector, q_plant.tail(3));
  // uncomment this line if using OLD simulation (without state estimator)
  // StateEstimation (q_plant, v_plant, end_effector, timestamp);


  /////recent changes for multi-ball
  /// update franka position again to include noise
  plant_franka_.SetPositions(&context_franka_, q_plant);
  plant_franka_.SetVelocities(&context_franka_, v_plant);

  // parse franka state info
  ///for three balls, change for n-balls!!!!! (CHECK THIS PART)
  VectorXd ball1 = q_plant.segment(7,7); //first 7 are Franka
  VectorXd ball2 = q_plant.segment(14,7);
  VectorXd ball3 = q_plant.segment(21,7);

  VectorXd ball1_dot = v_plant.segment(7,6);
  VectorXd ball2_dot = v_plant.segment(13,6);
  VectorXd ball3_dot = v_plant.segment(19,6);

  //VectorXd ball = q_plant.tail(7);
  Vector3d ball_xyz = ball1.tail(3);  // xyz for ball1
  //VectorXd ball_dot = v_plant.tail(6);   // velocity for ball 1
  Vector3d v_ball = ball1_dot.tail(3);

  //for 2nd ball
  Vector3d ball_xyz2 = ball2.tail(3);

  //for 3rd ball
  Vector3d ball_xyz3 = ball3.tail(3);

  VectorXd q(3 + 7 * num_balls_);
  q << end_effector, ball1, ball2, ball3;
  VectorXd v(3 + 6 * num_balls_);
  v << end_effector_dot, ball1_dot, ball2_dot, ball3_dot;

  VectorXd u = VectorXd::Zero(3);

  VectorXd state(plant_.num_positions() + plant_.num_velocities());
  state << end_effector, q_plant.tail(7 * num_balls_), end_effector_dot, v_plant.tail(6 * num_balls_);
  /// CHANGES IN ALL THIS PART

  ///change this for adaptive path
  VectorXd traj_desired_vector = pp_.value(timestamp);
  // compute adaptive path if enable_adaptive_path is 1
  if (param_.enable_adaptive_path == 1){
    // traj_desired_vector 7-9 is xyz of sphere
    double x = ball_xyz(0) - x_c;
    double y = ball_xyz(1) - y_c;

    double x2 = ball_xyz2(0) - x_c;
    double y2 = ball_xyz2(1) - y_c;

    double x3 = ball_xyz3(0) - x_c;
    double y3 = ball_xyz3(1) - y_c;

    // note that the x and y arguments are intentionally flipped
    // since we want to get the angle from the y-axis, not the x-axis
    double angle = atan2(x,y);
    double theta = angle + param_.lead_angle * PI / 180;

    traj_desired_vector(q_map_.at("sphere_x")) = x_c; // + traj_radius * sin(theta);
    traj_desired_vector(q_map_.at("sphere_y")) = y - 0.01; // + traj_radius * cos(theta);
    traj_desired_vector(q_map_.at("sphere_z")) = ball_radius + table_offset;

    traj_desired_vector(q_map_.at("sphere2_x")) = x_c; // + traj_radius * sin(theta);
    traj_desired_vector(q_map_.at("sphere2_y")) = y2 - 0.01; //  y - 0.01 - 2*ball_radius; // + traj_radius * cos(theta);
    traj_desired_vector(q_map_.at("sphere2_z")) = ball_radius + table_offset;

    traj_desired_vector(q_map_.at("sphere3_x")) = x_c; // + traj_radius * sin(theta);
    traj_desired_vector(q_map_.at("sphere3_y")) = y3 - 0.01; //  y - 0.01 - 2*ball_radius; // + traj_radius * cos(theta);
    traj_desired_vector(q_map_.at("sphere3_z")) = ball_radius + table_offset;

  }

  // compute sphere positional error
  Vector3d ball_xyz_d(traj_desired_vector(q_map_.at("sphere_x")),
                      traj_desired_vector(q_map_.at("sphere_y")),
                      traj_desired_vector(q_map_.at("sphere_z")));

  Vector3d ball_xyz_d2(traj_desired_vector(q_map_.at("sphere2_x")),
                      traj_desired_vector(q_map_.at("sphere2_y")),
                      traj_desired_vector(q_map_.at("sphere2_z")));

  Vector3d ball_xyz_d3(traj_desired_vector(q_map_.at("sphere3_x")),
                       traj_desired_vector(q_map_.at("sphere3_y")),
                       traj_desired_vector(q_map_.at("sphere3_z")));


  Vector3d error_xy = ball_xyz_d - ball_xyz;
  error_xy(2) = 0;
  Vector3d error_hat = error_xy / error_xy.norm();
  // compute phase
  double shifted_time = timestamp - settling_time -  return_phase;
  if (shifted_time < 0) shifted_time += period;
  double ts = shifted_time - period * floor((shifted_time / period));
  double back_dist = param_.gait_parameters(0);

  if (state[8] > state[15] && state[8] > state[22]) {

    //std::cout << "here" << std::endl;
    /// rolling phase
    if (ts < roll_phase) {
      traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[7]; //- 0.2* back_dist*error_hat(0);  //14
      traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[8]; //- 0.2* back_dist*error_hat(1);  //15
      traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] =
          traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] + 0.004;
    }
      /// upwards phase
    else if (ts < roll_phase + return_phase / 3) {
      traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[0]; //0.55;
      traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[1]; //0.1;
      traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = param_.gait_parameters(1) + table_offset;

    }
      /// side ways phase
    else if (ts < roll_phase + 2 * return_phase / 3) {
      traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[7] - back_dist * error_hat(0);
      traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[8] - back_dist * error_hat(1);
      traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = param_.gait_parameters(2) + table_offset;
    }
      /// position finger phase
    else {
      traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[7] - 1 * back_dist * error_hat(0);
      traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[8] - 1 * back_dist * error_hat(1);
      traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = param_.gait_parameters(3) + table_offset;
    }

  }

  else if (state[15] > state[8] && state[15] > state[22]) {

    error_xy = ball_xyz_d2 - ball_xyz2;
    error_xy(2) = 0;
    error_hat = error_xy / error_xy.norm();

    /// rolling phase
    if (ts < roll_phase) {
      traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[14]; //- 0.2* back_dist*error_hat(0);  //14
      traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[15]; //- 0.2* back_dist*error_hat(1);  //15
      traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] =
          traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] + 0.004;
    }
      /// upwards phase
    else if (ts < roll_phase + return_phase / 3) {
      traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[0]; //0.55;
      traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[1]; //0.1;
      traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = param_.gait_parameters(1) + table_offset;

    }
      /// side ways phase
    else if (ts < roll_phase + 2 * return_phase / 3) {
      traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[14] - back_dist * error_hat(0);
      traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[15] - back_dist * error_hat(1);
      traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = param_.gait_parameters(2) + table_offset;
    }
      /// position finger phase
    else {
      traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[14] - 1 * back_dist * error_hat(0);
      traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[15] - 1 * back_dist * error_hat(1);
      traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = param_.gait_parameters(3) + table_offset;
    }

  }

  else{

    error_xy = ball_xyz_d3 - ball_xyz3;
    error_xy(2) = 0;
    error_hat = error_xy / error_xy.norm();

    /// rolling phase
    //std::cout << "here" << std::endl;
    if (ts < roll_phase) {
      traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[21]; //- 0.2* back_dist*error_hat(0);  //14
      traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[22]; //- 0.2* back_dist*error_hat(1);  //15
      traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] =
          traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] + 0.004;
    }
      /// upwards phase
    else if (ts < roll_phase + return_phase / 3) {
      traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[0]; //0.55;
      traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[1]; //0.1;
      traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = param_.gait_parameters(1) + table_offset;

    }
      /// side ways phase
    else if (ts < roll_phase + 2 * return_phase / 3) {
      traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[21] - back_dist * error_hat(0);
      traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[22] - back_dist * error_hat(1);
      traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = param_.gait_parameters(2) + table_offset;
    }
      /// position finger phase
    else {
      traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[21] - 1 * back_dist * error_hat(0);
      traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[22] - 1 * back_dist * error_hat(1);
      traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = param_.gait_parameters(3) + table_offset;
    }


  }


  std::vector<VectorXd> traj_desired(Q_.size() , traj_desired_vector);

  /// compute desired orientation
  Vector3d axis = VectorXd::Zero(3);
  if (param_.axis_option == 1){
    // OPTION 1: tilt EE away from the desired direction of the ball
    axis << error_hat(1), -error_hat(0), 0;
  }
  else if (param_.axis_option == 2){
    // OPTION 2: tilt EE toward the center of the circle trajectory
    axis << ball_xyz(1)-y_c, -(ball_xyz(0)-x_c), 0;
    axis = axis / axis.norm();
  }

//  Eigen::AngleAxis<double> angle_axis(PI * param_.orientation_degrees / 180.0, axis);
//  RotationMatrix<double> rot(angle_axis);
  RotationMatrix<double> rot = RotationMatrix<double>::MakeYRotation(-param_.orientation_degrees * 3.14 / 180);
  RotationMatrix<double> default_orientation(Quaterniond(0, 1, 0, 0));
  RotationMatrix<double> R_desired = rot * default_orientation;  /// compute interpolated orientation
  Eigen::AngleAxis<double> R_cd = (R_current.inverse() * R_desired).ToAngleAxis();
  double max_delta_angle = 0.5 * 3.14 / 180.0;
  R_cd.angle() = (R_cd.angle() > max_delta_angle) ? max_delta_angle : R_cd.angle();
  R_cd.angle() = (R_cd.angle() < -max_delta_angle) ? -max_delta_angle : R_cd.angle();
  VectorXd orientation_d = (R_current * RotationMatrix<double>(R_cd)).ToQuaternionAsVector4();

  // VectorXd orientation_d = (R_desired).ToQuaternionAsVector4();

  /// update autodiff
  VectorXd xu(plant_f_.num_positions() + plant_f_.num_velocities() +
      plant_f_.num_actuators());
  xu << q, v, u;
  auto xu_ad = drake::math::InitializeAutoDiff(xu);

  plant_ad_f_.SetPositionsAndVelocities(
      &context_ad_f_,
      xu_ad.head(plant_f_.num_positions() + plant_f_.num_velocities()));
  multibody::SetInputsIfNew<AutoDiffXd>(
      plant_ad_f_, xu_ad.tail(plant_f_.num_actuators()), &context_ad_f_);


  /// upddate context


  plant_f_.SetPositions(&context_f_, q);
  plant_f_.SetVelocities(&context_f_, v);
  multibody::SetInputsIfNew<double>(plant_f_, u, &context_f_);

  //std::cout << "here" << std::endl;


  /// figure out a nice way to do this as SortedPairs with pybind is not working
  /// (potentially pass a matrix 2xnum_pairs?)

  ///change for more than 2 spheres!!! (0: finger, 1: sphere1, 2: ground, 3: sphere2, 4: sphere3)

  std::vector<SortedPair<GeometryId>> contact_pairs;
  contact_pairs.push_back(SortedPair(contact_geoms_[0], contact_geoms_[1]));    // ee & ball1
  contact_pairs.push_back(SortedPair(contact_geoms_[1], contact_geoms_[2]));    // ball1 & ground
  contact_pairs.push_back(SortedPair(contact_geoms_[0], contact_geoms_[3]));    // ee & ball2
  contact_pairs.push_back(SortedPair(contact_geoms_[2], contact_geoms_[3]));    // ball2 & ground
  contact_pairs.push_back(SortedPair(contact_geoms_[1], contact_geoms_[3]));    // ball1 & ball2
  ///3rd ball
  contact_pairs.push_back(SortedPair(contact_geoms_[0], contact_geoms_[4]));    // ee & ball3
  contact_pairs.push_back(SortedPair(contact_geoms_[2], contact_geoms_[4]));    // ball3 & ground
  contact_pairs.push_back(SortedPair(contact_geoms_[1], contact_geoms_[4]));    // ball1 & ball3
  contact_pairs.push_back(SortedPair(contact_geoms_[3], contact_geoms_[4]));    // ball2 & ball3


  auto system_scaling_pair = solvers::LCSFactoryConvex::LinearizePlantToLCS(
      plant_f_, context_f_, plant_ad_f_, context_ad_f_, contact_pairs,
      num_friction_directions_, mu_, 0.1, time_horizon_);

  solvers::LCS lcs_system_full = system_scaling_pair.first;
  solvers::LCS* lcs = &lcs_system_full;
  // double scaling = system_scaling_pair.second;

  int n_full = ((lcs->A_)[0].cols());
  int m_full = ((lcs->D_)[0].cols());
  int k_full = ((lcs->B_)[0].cols());

//  std::cout << "n_full" << n_full << std::endl;
//  std::cout << "m_full" << m_full << std::endl;
//  std::cout << "k_full" << k_full << std::endl;

  std::set<int> active_lambda_inds;
  std::set<int> inactive_lambda_inds;
//  active_lambda_inds.insert(3);   ///\lambda_n for sphere1 - ground contact
//  active_lambda_inds.insert(8);   ///\lambda_t for sphere1 - ground contact
//  active_lambda_inds.insert(9);
//  active_lambda_inds.insert(10);
//  active_lambda_inds.insert(11);
//  inactive_lambda_inds.insert(1);  ///gamma for sphere1 - ground contact

  active_lambda_inds.insert(4);   ///\lambda for sphere1 - ground contact
  active_lambda_inds.insert(5);
  active_lambda_inds.insert(6);
  active_lambda_inds.insert(7);

  active_lambda_inds.insert(12);   ///\lambda for sphere2 - ground contact
  active_lambda_inds.insert(13);
  active_lambda_inds.insert(14);
  active_lambda_inds.insert(15);

  active_lambda_inds.insert(24);   ///\lambda for sphere3 - ground contact
  active_lambda_inds.insert(25);
  active_lambda_inds.insert(26);
  active_lambda_inds.insert(27);


  auto lcs_system_fixed = solvers::LCSFactoryConvex::FixSomeModes(lcs_system_full,
      active_lambda_inds, inactive_lambda_inds);

  if (param_.fix_sticking_ground) {
    lcs = &lcs_system_fixed;
  }

  // auto positions = multibody::makeNameToPositionsMap(plant_f_);

  // for ( const auto &[key, value]: positions ) {
  //       std::cout << key << " " << value << std::endl;
  // }

  C3Options options;
  int N = (lcs->A_).size();
  int n = ((lcs->A_)[0].cols());
  int m = ((lcs->D_)[0].cols());
  int k = ((lcs->B_)[0].cols());



  ///****** START CHANGE OF VARIABLES *********
  // Change of variables x->y
  // y = S*x (selection matrix S)
  // x = (S^T + W)*y + + x0 for constant x0
  // W estimates the angular velocity using the linear position
  // x0 contains the nominal height and quaternion
  // Hardcoding, but could do automatically
  // TODO: this will need to adapt to multiple spheres
  // q = [finger_pos; sphere_quat; sphere_pos]
  // v = [finger_vel; sphere_ang_vel; sphere_vel]
  // y = [finger_pos; sphere_pos_2d; finger_vel; sphere_vel_2d]
  int num_spheres = num_balls_;
  int ny = n - num_spheres * 9; // remove quat/angvel/z/zdot=3+4+1+1=9
  int ny_q = plant_f_.num_positions() - num_spheres * 5;
  MatrixXd S = MatrixXd::Zero(ny, n);

  // finger_pos
  S.block(0, 0, 3, 3) = MatrixXd::Identity(3, 3);
  // sphere_pos (1)
  S.block(3, 7, 2, 2) = MatrixXd::Identity(2, 2);
  // sphere_pos (2)
  S.block(5, 14, 2, 2) = MatrixXd::Identity(2, 2);

  // sphere_pos (3)
  S.block(7, 21, 2, 2) = MatrixXd::Identity(2, 2);

  // finger_vel
  S.block(ny_q, plant_f_.num_positions(), 3, 3) = MatrixXd::Identity(3, 3);
  // sphere_vel(1)
  S.block(ny_q + 3, plant_f_.num_positions() + 6, 2, 2) = MatrixXd::Identity(2, 2);
  // sphere_vel(2)
  S.block(ny_q + 5, plant_f_.num_positions() + 12, 2, 2) = MatrixXd::Identity(2, 2);

  // sphere_vel(3)
  S.block(ny_q + 7, plant_f_.num_positions() + 18, 2, 2) = MatrixXd::Identity(2, 2);

  // Offset
  VectorXd x0 = VectorXd::Zero(n);
  //quaternion for sphere(1)
  x0(3) = 1;
  //height for sphere(1)
  x0(9) = param_.ball_radius + param_.table_offset;
  //quaternion for sphere(2)
  x0(10) = 1;
  //height for sphere(2)
  x0(16) = param_.ball_radius + param_.table_offset;

  //quaternion for sphere(3)
  x0(17) = 1;
  //height for sphere(2)
  x0(23) = param_.ball_radius + param_.table_offset;

  //Angular velocity
  MatrixXd W = MatrixXd::Zero(n, ny);

  //sphere(1)
  W(plant_f_.num_positions() + 3, ny_q + 4) = -param_.ball_radius;
  W(plant_f_.num_positions() + 4, ny_q + 3) = param_.ball_radius;

  //sphere(2)
  W(plant_f_.num_positions() + 9, ny_q + 6) = -param_.ball_radius;
  W(plant_f_.num_positions() + 10, ny_q + 5) = param_.ball_radius;

  //sphere(3)
  W(plant_f_.num_positions() + 15, ny_q + 8) = -param_.ball_radius;
  W(plant_f_.num_positions() + 16, ny_q + 7) = param_.ball_radius;

  // y' = S*x' = S*A*x + S*B*u + S*D*lambda + S*d
  //           = S*A*(S^T + W)*y + S*B*u + S*D*lambda + S*d + S*A*x0
  // note that S*A*x0 = 0
  //
  // E*x + F*lambda + H*u + c
  //  = E*(S^T + W)*y + F*lambda + H*u + c + E*x0
  // note that E*x0 = 0
  // auto lcs_reduced = LCS(A, B, D, d, E, F, H, c, N);

  // assumes original lcs_system is invariant LCS

  auto A_red = S*lcs->A_[0]*(S.transpose()+W);
  auto B_red = S*lcs->B_[0];
  auto D_red = S*lcs->D_[0];
  auto d_red = S*lcs->d_[0];
  auto E_red = lcs->E_[0]*(S.transpose()+W);
  auto F_red = lcs->F_[0];
  auto H_red = lcs->H_[0];
  auto c_red = lcs->c_[0];
  auto lcs_reduced = dairlib::solvers::LCS(A_red, B_red, D_red, d_red, E_red,
      F_red, H_red, c_red, N);

  if (param_.rolling_state_reduction) {
    lcs = &lcs_reduced;
    // change dimension definition

    //n = ny;

    n = A_red.cols();
    m = D_red.cols();
    k = B_red.cols();

//    std::cout << "n:" << n << std::endl;
//    std::cout << "m:" << m << std::endl;
//    std::cout << "k:" << k << std::endl;

  }


  // std::cout << F_red << std::endl << std::endl;
  // std::cout << lcs->E_[0]*x0 << std::endl << std::endl;

  ///****** END CHANGE OF VARIABLES *********

//
//
  /// initialize ADMM variables (delta, w)
  std::vector<VectorXd> delta(N, VectorXd::Zero(n + m + k));
  std::vector<VectorXd> w(N, VectorXd::Zero(n + m + k));

  /// initialize ADMM reset variables (delta, w are reseted to these values)
  std::vector<VectorXd> delta_reset(N, VectorXd::Zero(n + m + k));
  std::vector<VectorXd> w_reset(N, VectorXd::Zero(n + m + k));

  if (options.delta_option == 1) {
    /// reset delta and w (option 1)
    delta = delta_reset;
    w = w_reset;
    for (int j = 0; j < N; j++) {
      //delta[j].head(n) = xdesired_[0]; //state
      // delta[j].head(n) << S * state; //state
      if (param_.rolling_state_reduction) {
        delta[j].head(n) << S * state; //state
      } else {
        delta[j].head(n) << state; //state
      }
    }
  } else {
    /// reset delta and w (default option)
    delta = delta_reset;
    w = w_reset;
  }


  MatrixXd Qnew = Q_[0];

  if (ts > roll_phase){
    double Qnew_finger = param_.Qnew_finger;
    Qnew(0,0) = Qnew_finger;
    Qnew(1,1) = Qnew_finger;
    Qnew(2,2) = Qnew_finger;
    Qnew(7,7) = param_.Qnew_ball_x;
    Qnew(8,8) = param_.Qnew_ball_y;
    Qnew(14,14) = param_.Qnew_ball_x;
    Qnew(15,15) = param_.Qnew_ball_y;


    //sphere(3)
    Qnew(21,21) = param_.Qnew_ball_x;
    Qnew(22,22) = param_.Qnew_ball_y;


  }

  if (param_.rolling_state_reduction) {
    Qnew = (S.transpose() + W).transpose() * Qnew * (S.transpose() + W);
    traj_desired = std::vector<VectorXd>(Q_.size() , S * traj_desired_vector);
  }

//  std::cout << "here" << std::endl;
//  std::cout << Qnew << std::endl;

  std::vector<MatrixXd> Qha(Q_.size(), Qnew);

  /// initialize warm start for the MIQP
//  int nx = plant_.num_positions() + plant_.num_velocities();
//  int nlambda = 12;
//  int nu = plant_.num_actuators();


  int nx = n;
  int nlambda = m;
  int nu = k;

  for (int i = 0; i < time_horizon_; i++){
    warm_start_delta_.push_back(VectorXd::Zero(nx+nlambda+nu));
  }
  for (int i = 0; i < time_horizon_; i++){
    warm_start_binary_.push_back(VectorXd::Zero(nlambda));
  }
  for (int i = 0; i < time_horizon_+1; i++){
    warm_start_x_.push_back(VectorXd::Zero(nx));
  }
  for (int i = 0; i < time_horizon_; i++){
    warm_start_lambda_.push_back(VectorXd::Zero(nlambda));
  }
  for (int i = 0; i < time_horizon_; i++){
    warm_start_u_.push_back(VectorXd::Zero(nu));
  }

   //TODO: fix other args to opt() --- fixed, double-check
   //G, U, traj_desired, warm starts;
   //auto Q_red = (S + W.transpose()) * Qha * (S.transpose() + W);

   /// assumes G and U have the form: [ constant_1*I 0 0; 0 constant_2*I 0; 0 0 constant_3*I ]
  std::vector<MatrixXd> G, U;
   if (param_.rolling_state_reduction) {
     for (int i = 0; i < G_.size(); i++) {
       MatrixXd G_holder = MatrixXd::Zero(n + m + k, n + m + k);
       //G_holder.block(0,0,n,n) = (S + W.transpose()) *  G_[i].block(0, 0, n_full, n_full)  *  (S.transpose() + W);
       G_holder.block(0,0,n,n) = G_[i].block(0, 0, n, n);
       G_holder.block(n,n,m,m) = G_[i].block(n_full, n_full, m, m);
       G_holder.block(n+m,n+m,k,k) = G_[i].block(n_full + m_full, n_full + m_full, k, k) ;
       G.push_back( G_holder );

       MatrixXd U_holder = MatrixXd::Zero(n + m + k, n + m + k);
       //U_holder.block(0,0,n,n) = (S + W.transpose()) *  U_[i].block(0, 0, n_full, n_full)  *  (S.transpose() + W);
       U_holder.block(0,0,n,n) = U_[i].block(0, 0, n, n);
       U_holder.block(n,n,m,m) = U_[i].block(n_full, n_full, m, m);
       U_holder.block(n+m,n+m,k,k) = U_[i].block(n_full + m_full, n_full + m_full, k, k) ;
       U.push_back( U_holder );

       warm_start_delta_[i] = warm_start_delta_[i].head(n + m + k);
       warm_start_binary_[i] = warm_start_binary_[i].head(m);
       warm_start_lambda_[i] = warm_start_lambda_[i].head(m);
       warm_start_x_[i] = warm_start_x_[i].head(n);
       warm_start_u_[i] = warm_start_u_[i].head(k);
     }
   }
   else {
     for (int i = 0; i < G_.size(); i++) {
       G.push_back(G_[i].block(0, 0, n + m + k, n + m + k));
       U.push_back(U_[i].block(0, 0, n + m + k, n + m + k));
       warm_start_delta_[i] = warm_start_delta_[i].head(n + m + k);
       warm_start_binary_[i] = warm_start_binary_[i].head(m);
       warm_start_lambda_[i] = warm_start_lambda_[i].head(m);
       warm_start_x_[i] = warm_start_x_[i].head(n);
       warm_start_u_[i] = warm_start_u_[i].head(k);
     }
   }

  solvers::C3APPROX opt(*lcs, Qha, R_, G, U, traj_desired, options,
    warm_start_delta_, warm_start_binary_, warm_start_x_,
    warm_start_lambda_, warm_start_u_, true);

  /// calculate the input given x[i]

  auto t_setup = std::chrono::high_resolution_clock::now();

  VectorXd input;
  if (param_.rolling_state_reduction) {
    VectorXd state_init = S * state;
    input = opt.Solve(state_init, delta, w);
  } else {
    input = opt.Solve(state, delta, w);
  }
  auto t_solve = std::chrono::high_resolution_clock::now();
//
  warm_start_x_ = opt.GetWarmStartX();
  warm_start_lambda_ = opt.GetWarmStartLambda();
  warm_start_u_ = opt.GetWarmStartU();
  warm_start_delta_ = opt.GetWarmStartDelta();
  warm_start_binary_ = opt.GetWarmStartBinary();

  // compute dt based on moving averaege filter
  double dt = 0;
  if (moving_average_.empty()){
    dt = param_.dt;
  }
  else{
    for (int i = 0; i < (int) moving_average_.size(); i++){
      dt += moving_average_[i];
    }
    dt /= moving_average_.size();
  }

  ///calculate state and force
  auto system_scaling_pair2 = solvers::LCSFactoryConvex::LinearizePlantToLCS(
      plant_f_, context_f_, plant_ad_f_, context_ad_f_, contact_pairs,
      num_friction_directions_, mu_, dt, time_horizon_);

  solvers::LCS system2_ = system_scaling_pair2.first;
  double scaling2 = system_scaling_pair2.second;

  drake::solvers::MobyLCPSolver<double> LCPSolver;
  VectorXd force;

  auto flag = LCPSolver.SolveLcpLemkeRegularized(system2_.F_[0], system2_.E_[0] * scaling2 * state + system2_.c_[0] * scaling2 + system2_.H_[0] * scaling2 * input,
                                                 &force);
  //(void)flag; // suppress compiler unused variable warning
  std::cout << "force" << std::endl;
  std::cout << force << std::endl;


  VectorXd state_next = system2_.A_[0] * state + system2_.B_[0] * input + system2_.D_[0] * force / scaling2 + system2_.d_[0];

  // check if the desired end effector position is unreasonably far from the current location
  Vector3d vd = state_next.segment(3+7*num_balls_, 3); ///needs to change for n balls
  if (vd.norm() > max_desired_velocity_){
    /// update new desired position accordingly
    Vector3d dx = state_next.head(3) - state.head(3);
    state_next.head(3) << max_desired_velocity_ * dt * dx / dx.norm() + state.head(3);

    /// update new desired velocity accordingly
    Vector3d clamped_velocity = max_desired_velocity_ * vd / vd.norm();
    state_next(3+7*num_balls_) = clamped_velocity(0);
    state_next(3+7*num_balls_+1) = clamped_velocity(1);
    state_next(3+7*num_balls_+2) = clamped_velocity(2);

    std::cout << "velocity limit(c3)" << std::endl;

    /// update the user
    // std::cout << "The desired EE velocity was " << vd.norm() << "m/s. ";
    // std::cout << "Clamping the desired EE velocity to " << max_desired_velocity_ << "m/s." << std::endl;
  }
//

  ///COMMENT UP

  VectorXd force_des = VectorXd::Zero(4*num_balls_);
  //force_des << force(2), force(4), force(5), force(6), force(7);
//
//  //VectorXd st_desired(10 + 5*num_balls_ + 1);
  VectorXd st_desired = VectorXd::Zero(num_output_);
//
//   //st_desired for debug purposes: (state_next.head(3), orientation_d(4), ball_position(7), finger_velocity(3), ball_velocity(6), force_des.head(6), ball_xyz_d, ball_xyz, true_ball_xyz)
//  //st_desired << state_next.head(3), orientation_d, state_next.tail(16), force_des, VectorXd::Zero(9);
//
  // change format to des_ee_xyz_pos, des_ee_orientation, des_ee_vel, des_force
  VectorXd des_ee_xyz_pos = state_next.head(3);
  VectorXd des_ee_orientation = orientation_d;
  VectorXd des_ee_vel = state_next.segment(3+7*num_balls_,3);
  VectorXd des_ee_force = force_des;
  VectorXd misc = VectorXd::Zero(1);

  st_desired << des_ee_xyz_pos, des_ee_orientation, des_ee_vel, des_ee_force, misc;

  state_contact_desired->SetDataVector(st_desired);
  state_contact_desired->set_timestamp(timestamp);

  /// update moving average filter and prev variables
  if (moving_average_.size() < dt_filter_length_){
    moving_average_.push_back(timestamp - prev_timestamp_);
  }
  else {
    moving_average_.pop_front();
    moving_average_.push_back(timestamp - prev_timestamp_);
  }
  prev_timestamp_ = timestamp;
  // prev_position_ = ball_xyz;
  // prev_velocity_ = v_ball;

  /// update moving average filter and prev variables
  // if (past_velocities_.size() < 10){
  //   past_velocities_.push_back(v_ball);
  // }
  // else {
  //   past_velocities_.pop_front();
  //   past_velocities_.push_back(v_ball);
  // }

  /// timing the code
//  auto t_end = std::chrono::high_resolution_clock::now();
//  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);
//  auto duration_solve = std::chrono::duration_cast<std::chrono::milliseconds>(t_solve - t_setup);
//  auto duration_setup = std::chrono::duration_cast<std::chrono::milliseconds>(t_setup - t_start);
//  std::cout << duration.count() << " " << duration_setup.count() << " " << duration_solve.count() << std::endl;
  //std::cout << state_contact_desired->size() << std::endl;
}

void C3Controller_franka_Convex::StateEstimation(Eigen::VectorXd& q_plant, Eigen::VectorXd& v_plant,
                                          const Eigen::Vector3d end_effector, double timestamp) const {
  /// estimate q_plant
  if (abs(param_.ball_stddev) > 1e-9) {
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{0, param_.ball_stddev};

    double dist_x = d(gen);
    double dist_y = d(gen);
    double noise_threshold = 1000000;
    if (dist_x > noise_threshold) {
      dist_x = noise_threshold;
    } else if (dist_x < -noise_threshold) {
      dist_x = -noise_threshold;
    }
    if (dist_y > noise_threshold) {
      dist_y = noise_threshold;
    } else if (dist_y < -noise_threshold) {
      dist_y = -noise_threshold;
    }
    double x_obs = q_plant(q_map_franka_.at("sphere_x")) + dist_x;
    double y_obs = q_plant(q_map_franka_.at("sphere_y")) + dist_y;

    double alpha_p = param_.alpha_p;
    q_plant(q_map_franka_.at("sphere_x")) = alpha_p*x_obs + (1-alpha_p)*prev_position_(0);
    q_plant(q_map_franka_.at("sphere_y")) = alpha_p*y_obs + (1-alpha_p)*prev_position_(1);

    ///project estimate
    q_plant.tail(7) << 1, 0, 0, 0, ProjectStateEstimate(end_effector, q_plant.tail(3));;
  }
  else{
    q_plant.tail(7) << 1, 0, 0, 0, q_plant.tail(3);
  }

  /// estimate v_plant
//  std::cout << "before\n" << v_plant.tail(6) << std::endl;
  double alpha_v = param_.alpha_v;
  double ball_radius = param_.ball_radius;

  ///1
  Vector3d ball_xyz_dot = v_plant.tail(3);

  ///2
  // Vector3d curr_velocity = (q_plant.tail(3) - prev_position_) / (timestamp - prev_timestamp_);
  // Vector3d ball_xyz_dot = alpha_v * curr_velocity + (1-alpha_v)*prev_velocity_;
  // ball_xyz_dot(2) = 0; // expect no velocity in z direction

  ///3
//  Vector3d average = Vector3d::Zero(3);
//  for(int i = 0; i < 10; i++){
//    average = average + past_velocities_[i]/10;
//  }
//  Vector3d ball_xyz_dot = average;

  Vector3d r_ball(0, 0, ball_radius);
  Vector3d computed_ang_vel = r_ball.cross(ball_xyz_dot) / (ball_radius * ball_radius);
  v_plant.tail(6) << computed_ang_vel, ball_xyz_dot;
}

Eigen::Vector3d C3Controller_franka_Convex::ProjectStateEstimate(
    const Eigen::Vector3d& endeffector,
    const Eigen::Vector3d& estimate) const {

  Eigen::Vector3d dist_vec = estimate - endeffector;
  double R = param_.ball_radius;
  double r = param_.finger_radius;
  
  if (dist_vec.norm() < (R+r)*(1)){
    Eigen::Vector3d u(dist_vec(0), dist_vec(1), 0);
    double u_norm = u.norm();
    double du = sqrt((R+r)*(R+r) - dist_vec(2)*dist_vec(2)) - u_norm;

    return estimate + du * u / u_norm;
  }
  else {
    return estimate;
  }
}

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib

std::vector<Eigen::Vector3d> move_to_initial_position(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& finish,
    double curr_time, double stabilize_time1,
    double move_time, double stabilize_time2){

  Eigen::Vector3d zero(0,0,0);

  if (curr_time < stabilize_time1){
    return {start, zero};
  }
  else if (curr_time < stabilize_time1 + move_time){
    double a = (curr_time-stabilize_time1) / (move_time);
    Eigen::Vector3d p = (1-a)*start + a*finish;
    Eigen::Vector3d v = (finish - start) / move_time;
    return {p, v};
  }
  else{
    return {finish, zero};
  }
}