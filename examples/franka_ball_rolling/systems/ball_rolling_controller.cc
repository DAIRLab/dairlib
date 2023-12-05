#include "ball_rolling_controller.h"

#include <utility>
#include <chrono>


#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/common/sorted_pair.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/multibody/plant/multibody_plant.h"
#include "multibody/multibody_utils.h"
#include "solvers/c3.h"
#include "solvers/c3_miqp.h"
#include "solvers/lcs_factory.h"

//#include "solvers/miqp.h"

#include "drake/solvers/moby_lcp_solver.h"
#include "multibody/geom_geom_collider.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "solvers/lcs_factory.h"
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

C3Controller_franka::C3Controller_franka(
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
    const vector<MatrixXd>& U, const vector<VectorXd>& xdesired, const drake::trajectories::PiecewisePolynomial<double>& pp)
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
      pp_(pp){


  // initialize warm start
  int time_horizon = 5;
  int nx = 19;
  int nlambda = 12;
  int nu = 3;

  for (int i = 0; i < time_horizon; i++){
    warm_start_delta_.push_back(VectorXd::Zero(nx+nlambda+nu));
  }
  for (int i = 0; i < time_horizon; i++){
    warm_start_binary_.push_back(VectorXd::Zero(nlambda));
  }
  for (int i = 0; i < time_horizon+1; i++){
    warm_start_x_.push_back(VectorXd::Zero(nx));
  }
  for (int i = 0; i < time_horizon; i++){
    warm_start_lambda_.push_back(VectorXd::Zero(nlambda));
  }
  for (int i = 0; i < time_horizon; i++){
    warm_start_u_.push_back(VectorXd::Zero(nu));
  }
  

  state_input_port_ =
      this->DeclareVectorInputPort(
              "x, u, t",
              OutputVector<double>(14, 13, 7))
          .get_index();


  state_output_port_ = this->DeclareVectorOutputPort(
          "xee, xball, xee_dot, xball_dot, lambda, visualization",
          TimestampedVector<double>(38), &C3Controller_franka::CalcControl)
      .get_index();

  q_map_franka_ = multibody::MakeNameToPositionsMap(plant_franka_);
  v_map_franka_ = multibody::MakeNameToVelocitiesMap(plant_franka_);
  q_map_ = multibody::MakeNameToPositionsMap(plant_);
  v_map_ = multibody::MakeNameToVelocitiesMap(plant_);

  // get c3_parameters
  param_ = drake::yaml::LoadYamlFile<C3Parameters>(
      "examples/franka_trajectory_following/parameters.yaml");
  max_desired_velocity_ = param_.velocity_limit;

  // filter info
  prev_timestamp_ = 0;
  dt_filter_length_ = param_.dt_filter_length;
}

void C3Controller_franka::CalcControl(const Context<double>& context,
                                      TimestampedVector<double>* state_contact_desired) const {

  // get values
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

    // fill st_desired
    VectorXd traj = pp_.value(timestamp);
    VectorXd st_desired = VectorXd::Zero(38);
    st_desired.head(3) << target[0];
    st_desired.segment(3, 4) << orientation_d.w(), orientation_d.x(), orientation_d.y(), orientation_d.z();
    st_desired.segment(11, 3) << finish(0), finish(1), ball_radius + table_offset;
    st_desired.segment(14, 3) << target[1];
    st_desired.segment(32, 3) << finish(0), finish(1), ball_radius + table_offset;
    st_desired.tail(3) << finish(0), finish(1), ball_radius + table_offset;

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


  /// update franka position again to include noise
  plant_franka_.SetPositions(&context_franka_, q_plant);
  plant_franka_.SetVelocities(&context_franka_, v_plant);

  // parse franka state info
  VectorXd ball = q_plant.tail(7);
  Vector3d ball_xyz = ball.tail(3);
  VectorXd ball_dot = v_plant.tail(6);
  Vector3d v_ball = ball_dot.tail(3);

  VectorXd q(10);
  q << end_effector, ball;
  VectorXd v(9);
  v << end_effector_dot, ball_dot;
  VectorXd u = 0*VectorXd::Ones(3);

  VectorXd state(plant_.num_positions() + plant_.num_velocities());
  state << end_effector, q_plant.tail(7), end_effector_dot, v_plant.tail(6);


  ///change this for adaptive path
  VectorXd traj_desired_vector = pp_.value(timestamp);
  // compute adaptive path if enable_adaptive_path is 1
  if (param_.enable_adaptive_path == 1){
    // traj_desired_vector 7-9 is xyz of sphere
    double x = ball_xyz(0) - x_c;
    double y = ball_xyz(1) - y_c;

    // note that the x and y arguments are intentionally flipped
    // since we want to get the angle from the y-axis, not the x-axis
    double angle = atan2(x,y);
    double theta = angle + param_.lead_angle * PI / 180;

    traj_desired_vector(q_map_.at("base_x")) = x_c + traj_radius * sin(theta);
    traj_desired_vector(q_map_.at("base_y")) = y_c + traj_radius * cos(theta);
    traj_desired_vector(q_map_.at("base_z")) = ball_radius + table_offset;
  }

  // compute sphere positional error
  Vector3d ball_xyz_d(traj_desired_vector(q_map_.at("base_x")),
                      traj_desired_vector(q_map_.at("base_y")),
                      table_offset + ball_radius);
  Vector3d error_xy = ball_xyz_d - ball_xyz;
  error_xy(2) = 0;
  Vector3d error_hat = error_xy / error_xy.norm();

  // compute phase
  double shifted_time = timestamp - settling_time -  return_phase;
  if (shifted_time < 0) shifted_time += period;
  double ts = shifted_time - period * floor((shifted_time / period));
  double back_dist = param_.gait_parameters(0);
  
  /// rolling phase
  if ( ts < roll_phase ) {
    traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[7];
    traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[8];
    traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] + 0.004;
  }
  /// upwards phase
  else if (ts < roll_phase + return_phase / 3){
//    traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[7];
//    traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[8];
//    traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] + 0.004;
    traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[0]; //0.55;
    traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[1]; //0.1;
    traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = param_.gait_parameters(1) + table_offset;

  }
  /// side ways phase
  else if( ts < roll_phase + 2 * return_phase / 3 ) {
//    traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[7];
//    traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[8];
//    traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] + 0.004;
    traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[7] - back_dist*error_hat(0);
    traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[8] - back_dist*error_hat(1);
    traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = param_.gait_parameters(2) + table_offset;
  }
  /// position finger phase
  else{
//    traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[7];
//    traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[8];
//    traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] + 0.004;
    traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[7] - back_dist*error_hat(0);
    traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[8] - back_dist*error_hat(1);
    traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = param_.gait_parameters(3) + table_offset;
  }
  std::vector<VectorXd> traj_desired(Q_.size() , traj_desired_vector);

//  std::cout << "LOOP" << std::endl;
//  std::cout << "CURRENT_X" << std::endl;
//  std::cout << ball_xyz(0) << std::endl;
//  std::cout << "DESIRED X" << std::endl;
//  std::cout << traj_desired_vector(q_map_.at("base_x")) << std::endl;
//  std::cout << "CURRENT_Y" << std::endl;
//  std::cout << ball_xyz(1) << std::endl;
//  std::cout << "DESIRED Y" << std::endl;
//  std::cout << traj_desired_vector(q_map_.at("base_y")) << std::endl;


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


//eski olan
//  RotationMatrix<double> rot = RotationMatrix<double>::MakeYRotation(-param_.orientation_degrees * 3.14 / 180);
//  RotationMatrix<double> default_orientation(Quaterniond(0, 1, 0, 0));
//  RotationMatrix<double> R_desired = rot * default_orientation;  /// compute interpolated orientation
//  Eigen::AngleAxis<double> R_cd = (R_current.inverse() * R_desired).ToAngleAxis();
//  double max_delta_angle = 0.5 * 3.14 / 180.0;
//  R_cd.angle() = (R_cd.angle() > max_delta_angle) ? max_delta_angle : R_cd.angle();
//  R_cd.angle() = (R_cd.angle() < -max_delta_angle) ? -max_delta_angle : R_cd.angle();
//  VectorXd orientation_d = (R_current * RotationMatrix<double>(R_cd)).ToQuaternionAsVector4();


Eigen::AngleAxis<double> angle_axis(PI * param_.orientation_degrees / 180.0, axis);
RotationMatrix<double> rot(angle_axis);
Quaterniond temp(0, 1, 0, 0);
RotationMatrix<double> default_orientation(temp);
VectorXd orientation_d = (rot * default_orientation).ToQuaternionAsVector4();



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

  /// figure out a nice way to do this as SortedPairs with pybind is not working
  /// (potentially pass a matrix 2xnum_pairs?)

  std::vector<SortedPair<GeometryId>> contact_pairs;
  contact_pairs.push_back(SortedPair(contact_geoms_[0], contact_geoms_[1]));
  contact_pairs.push_back(SortedPair(contact_geoms_[1], contact_geoms_[2]));

  auto system_scaling_pair = solvers::LCSFactoryFranka::LinearizePlantToLCS(
      plant_f_, context_f_, plant_ad_f_, context_ad_f_, contact_pairs,
      num_friction_directions_, mu_, 0.1);

  solvers::LCS system_ = system_scaling_pair.first;
  double scaling = system_scaling_pair.second;

  C3Options options;
  int N = (system_.A_).size();
  int n = ((system_.A_)[0].cols());
  int m = ((system_.D_)[0].cols());
  int k = ((system_.B_)[0].cols());


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
      delta[j].head(n) << state; //state
    }
  } else {
    /// reset delta and w (default option)
    delta = delta_reset;
    w = w_reset;
  }

  MatrixXd Qnew;
  MatrixXd Gnew;
  Qnew = Q_[0];
  Gnew = G_[0];

  if (ts > roll_phase){
    double Qnew_finger = param_.Qnew_finger;
    Qnew(0,0) = Qnew_finger;
    Qnew(1,1) = Qnew_finger;
    Qnew(2,2) = Qnew_finger;
    Qnew(7,7) = param_.Qnew_ball_x;
    Qnew(8,8) = param_.Qnew_ball_x;
    Qnew(9,9) = param_.Qnew_ball_y;
    Gnew = 0.001 * MatrixXd::Identity(n+k+m, n+k+m);
    Gnew(19+3,19+3) = 0.0000001;
  }

  std::vector<MatrixXd> Qha(Q_.size(), Qnew);
  std::vector<MatrixXd> Gha(G_.size(), Gnew);

  solvers::C3MIQP opt(system_, Qha, R_, Gha, U_, traj_desired, options,
   warm_start_delta_, warm_start_binary_, warm_start_x_,
   warm_start_lambda_, warm_start_u_, true);


//  solvers::MIQP opt(system_, Qha, R_, G_, U_, traj_desired, options, scaling,
//                      warm_start_delta_, warm_start_binary_, warm_start_x_,
//                      warm_start_lambda_, warm_start_u_, true);


  /// calculate the input given x[i]
  auto zfin = opt.Solve(state, delta, w);
  VectorXd input = zfin[0].segment(n + m, k);

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
  auto system_scaling_pair2 = solvers::LCSFactoryFranka::LinearizePlantToLCS(
      plant_f_, context_f_, plant_ad_f_, context_ad_f_, contact_pairs,
      num_friction_directions_, mu_, dt);

  solvers::LCS system2_ = system_scaling_pair2.first;
  double scaling2 = system_scaling_pair2.second;

  drake::solvers::MobyLCPSolver<double> LCPSolver;
  VectorXd force;

  auto flag = LCPSolver.SolveLcpLemkeRegularized(system2_.F_[0], system2_.E_[0] * scaling2 * state + system2_.c_[0] * scaling2 + system2_.H_[0] * scaling2 * input,
                                                 &force);
  (void)flag; // suppress compiler unused variable warning

  //std::cout << "flag" << flag << std::endl;

  VectorXd state_next = system2_.A_[0] * state + system2_.B_[0] * input + system2_.D_[0] * force / scaling2 + system2_.d_[0];

  // check if the desired end effector position is unreasonably far from the current location
  Vector3d vd = state_next.segment(10, 3);
  if (vd.norm() > max_desired_velocity_){
    /// update new desired position accordingly
    Vector3d dx = state_next.head(3) - state.head(3);
    state_next.head(3) << max_desired_velocity_ * dt * dx / dx.norm() + state.head(3);
    
    /// update new desired velocity accordingly
    Vector3d clamped_velocity = max_desired_velocity_ * vd / vd.norm();
    state_next(10) = clamped_velocity(0);
    state_next(11) = clamped_velocity(1);
    state_next(12) = clamped_velocity(2);

    std::cout << "velocity limit(c3)" << std::endl;

    /// update the user
    // std::cout << "The desired EE velocity was " << vd.norm() << "m/s. ";
    // std::cout << "Clamping the desired EE velocity to " << max_desired_velocity_ << "m/s." << std::endl;
  }

  VectorXd force_des = VectorXd::Zero(6);
  force_des << force(0), force(2), force(4), force(5), force(6), force(7);

  VectorXd st_desired(force_des.size() + state_next.size() + orientation_d.size() + ball_xyz_d.size() + ball_xyz.size() + true_ball_xyz.size());

  st_desired << state_next.head(3), orientation_d, state_next.tail(16), force_des.head(6), ball_xyz_d, ball_xyz, true_ball_xyz;

//  std::cout << "ADMM_X" << std::endl;
//  std::cout << state_next.tail(9) << std::endl;

//  std::cout << "ADMM_X" << std::endl;
//  std::cout << state_next(7) << std::endl;

//std::cout << "FORCE" << std::endl;
//std::cout << force << std::endl;

//if(force(2)>0) {
//  std::cout << "force_effect" << std::endl;
//  VectorXd test_vector = system2_.D_[0] * force / scaling2;
//  std::cout << test_vector.tail(3) << std::endl;
//}
//  std::cout << "ADMM_Y" << std::endl;
//  std::cout << state_next(8) << std::endl;

//  std::cout << "FORCE_CONTRIBUTION" << std::endl;
//  std::cout << system2_.D_[0] * force / scaling2 << std::endl;
  
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
}

void C3Controller_franka::StateEstimation(Eigen::VectorXd& q_plant, Eigen::VectorXd& v_plant,
                                          const Eigen::Vector3d end_effector, double timestamp) const {
  /// estimate q_plant
  std::cout << "here" << std::endl;
  if (abs(param_.ball_stddev) > 1e-9) {
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{0, param_.ball_stddev};


    double dist_x = d(gen);
    double dist_y = d(gen);


    double noise_threshold = 0.01;
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
    double x_obs = q_plant(q_map_franka_.at("base_x")) + dist_x;
    double y_obs = q_plant(q_map_franka_.at("base_y")) + dist_y;

    double alpha_p = param_.alpha_p;
    q_plant(q_map_franka_.at("base_x")) = alpha_p*x_obs + (1-alpha_p)*prev_position_(0);
    q_plant(q_map_franka_.at("base_y")) = alpha_p*y_obs + (1-alpha_p)*prev_position_(1);

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

Eigen::Vector3d C3Controller_franka::ProjectStateEstimate(
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