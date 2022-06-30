#include "c3_controller_franka.h"

#include <utility>
#include <chrono>


#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/common/sorted_pair.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/multibody/plant/multibody_plant.h"
#include "multibody/multibody_utils.h"
#include "solvers/c3.h"
#include "solvers/c3_miqp.h"
#include "solvers/lcs_factory.h"

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

  state_input_port_ =
      this->DeclareVectorInputPort(
              "x, u, t",
              OutputVector<double>(14, 13, 7))
          .get_index();


  state_output_port_ = this->DeclareVectorOutputPort(
          "xee, xball, xee_dot, xball_dot, lambda, visualization",
          TimestampedVector<double>(34), &C3Controller_franka::CalcControl)
      .get_index();


  // get c3_parameters
  param_ = drake::yaml::LoadYamlFile<C3Parameters>(
      "examples/franka_trajectory_following/parameters.yaml");

  // filter info
  prev_timestamp_ = 0;
  dt_filter_length_ = param_.dt_filter_length;
  ball_xyz_prev_ = VectorXd::Zero(3);

  // kalman filter
  // xhat_prev = VectorXd::Zero(6);
  // P_prev = MatrixXd::Zero(6,6);

}

void C3Controller_franka::CalcControl(const Context<double>& context,
                                      TimestampedVector<double>* state_contact_desired) const {

  // get values
  auto robot_output = (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
  double timestamp = robot_output->get_timestamp();

  // parse some useful values
  double period = param_.period;
  double duty_cycle = param_.duty_cycle;
  double return_cycle = 1-duty_cycle;
  double settling_time = param_.stabilize_time1 + param_.move_time + param_.stabilize_time2;
  double x_c = param_.x_c;
  double y_c = param_.y_c;
  double traj_radius = param_.traj_radius;
  double ball_radius = param_.ball_radius;

  if (timestamp <= settling_time){
    // move to initial position
    Eigen::Vector3d start = param_.initial_start;
    Eigen::Vector3d finish = param_.initial_finish;
    finish(0) = x_c + traj_radius * sin(param_.phase * PI/ 180) + finish(0);
    finish(1) = y_c + traj_radius * cos(param_.phase * PI / 180) + finish(1);
    std::vector<Eigen::Vector3d> target = move_to_initial_position(start, finish, timestamp,
                                                                   param_.stabilize_time1, param_.move_time, param_.stabilize_time2);
    
    // fill st_desired
    VectorXd traj = pp_.value(timestamp);
    VectorXd st_desired = VectorXd::Zero(34);
    st_desired.head(3) << target[0];
    st_desired(7) = finish(0);
    st_desired(8) = finish(1);
    st_desired(9) = ball_radius;
    st_desired(28) = traj(7);
    st_desired(29) = traj(8);
    st_desired(30) = traj(9);
    st_desired.tail(3) << traj(7), traj(8), traj(9);

    state_contact_desired->SetDataVector(st_desired);
    state_contact_desired->set_timestamp(timestamp);
    prev_timestamp_ = (timestamp);
    ball_xyz_prev_ = st_desired.segment(7,3);

    // xhat_prev << finish(0), finish(1), ball_radius, 0, 0, 0;
    // P_prev.topLeftCorner(3,3) << param_.ball_stddev * MatrixXd::Identity(3,3);
    // P_prev.bottomRightCorner(3,3) << 0 * MatrixXd::Identity(3,3);
    return;
  }

  // franka
  VectorXd state_franka(27);
  state_franka << robot_output->GetPositions(), robot_output->GetVelocities();
  plant_franka_.SetPositions(&context_franka_, robot_output->GetPositions());
  plant_franka_.SetVelocities(&context_franka_, robot_output->GetVelocities());

  // forward kinematics
  Vector3d EE_offset_ = param_.EE_offset;
  const drake::math::RigidTransform<double> H_mat =
      plant_franka_.EvalBodyPoseInWorld(context_franka_, plant_franka_.GetBodyByName("panda_link8"));
  const RotationMatrix<double> Rotation = H_mat.rotation();
  Vector3d end_effector = H_mat.translation() + Rotation*EE_offset_;

  // jacobian and end_effector_dot
  auto EE_frame_ = &plant_franka_.GetBodyByName("panda_link8").body_frame();
  auto world_frame_ = &plant_franka_.world_frame();
  MatrixXd J_fb (6, plant_franka_.num_velocities());
  plant_franka_.CalcJacobianSpatialVelocity(
      context_franka_, JacobianWrtVariable::kV,
      *EE_frame_, EE_offset_,
      *world_frame_, *world_frame_, &J_fb);
  MatrixXd J_franka = J_fb.block(0, 0, 6, 7);
  VectorXd end_effector_dot = ( J_franka * (robot_output->GetVelocities()).head(7) ).tail(3);

  // parse franka state info
  VectorXd ball = robot_output->GetPositions().tail(7);
  VectorXd ball_dot = robot_output->GetVelocities().tail(6);
  VectorXd q(10);
  q << end_effector, ball;
  VectorXd v(9);
  v << end_effector_dot, ball_dot;
  VectorXd u = VectorXd::Zero(3);

  // add noise to ball position
  Vector3d ball_xyz = ball.tail(3);
  if (abs(param_.ball_stddev) > 1e-9){
    std::random_device rd{};
    std::mt19937 gen{rd()};
    std::normal_distribution<> d{0, param_.ball_stddev};
    ball_xyz = ball.tail(3) + Vector3d(d(gen), d(gen), 0);
    ProjectStateEstimate(end_effector, ball_xyz);
    // ball_xyz = ball.tail(3) + Vector3d(d(gen), d(gen), d(gen));
  }
  // estimate ball velocity
  Vector3d v_ball = (ball_xyz - ball_xyz_prev_) /  (timestamp - prev_timestamp_);
  // Vector3d v_ball = ball_dot.tail(3);

  // compute the angular velocity
  Vector3d r_ball(0, 0, ball_radius);
  Vector3d computed_ang_vel = r_ball.cross(v_ball) / (ball_radius * ball_radius);
  VectorXd state(plant_.num_positions() + plant_.num_velocities());
  state << end_effector, 1, 0, 0, 0, ball_xyz, end_effector_dot, computed_ang_vel, v_ball;
  // state << end_effector, ball, end_effector_dot, ball_dot;

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

    traj_desired_vector(7) = x_c + traj_radius * sin(theta);
    traj_desired_vector(8) = y_c + traj_radius * cos(theta);
    traj_desired_vector(9) = ball_radius;
  }

  // compute sphere positional error
  Vector3d ball_xyz_d(traj_desired_vector(7),
                      traj_desired_vector(8),
                      traj_desired_vector(9));
  Vector3d error_xy = ball_xyz_d - ball_xyz;
  error_xy(2) = 0;
  Vector3d error_hat = error_xy / error_xy.norm();

  // compute phase
  double shifted_time = timestamp - settling_time -  return_cycle * period;
  if (shifted_time < 0) shifted_time += period;
  double ts = shifted_time - period * floor((shifted_time / period));
  double back_dist = param_.test_parameters(0);
  
  /// rolling phase
  if ( ts < period*duty_cycle ) {
    traj_desired_vector[0] = state[7];
    traj_desired_vector[1] = state[8];
  }
  /// upwards phase
  else if (ts < period * (duty_cycle+param_.duty_cycle_upwards_ratio * return_cycle)){
    traj_desired_vector[0] = state[0];
    traj_desired_vector[1] = state[1];
    traj_desired_vector[2] = param_.test_parameters(1);
  }
  /// side ways phase
  else if( ts < period * (duty_cycle+0.66 * return_cycle) ) {
    traj_desired_vector[0] = state[7] - back_dist*error_hat(0);
    traj_desired_vector[1] = state[8] - back_dist*error_hat(1);
    traj_desired_vector[2] = param_.test_parameters(2);
  }
  /// position finger phase
  else{
    traj_desired_vector[0] = state[7] - back_dist*error_hat(0);
    traj_desired_vector[1] = state[8] - back_dist*error_hat(1);
    traj_desired_vector[2] = param_.test_parameters(3);
  }
  std::vector<VectorXd> traj_desired(Q_.size() , traj_desired_vector);

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
  contact_pairs.push_back(SortedPair(contact_geoms_[0], contact_geoms_[1]));  //was 0, 3
  contact_pairs.push_back(SortedPair(contact_geoms_[1], contact_geoms_[2]));

  auto system_scaling_pair = solvers::LCSFactoryFranka::LinearizePlantToLCS(
      plant_f_, context_f_, plant_ad_f_, context_ad_f_, contact_pairs,
      num_friction_directions_, mu_, 0.1);

  solvers::LCS system_ = system_scaling_pair.first;
  // double scaling = system_scaling_pair.second;

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
  Qnew = Q_[0];

  if (ts > period * duty_cycle){
    double Qnew_finger = param_.Qnew_finger;
    Qnew(0,0) = Qnew_finger;
    Qnew(1,1) = Qnew_finger;
    Qnew(2,2) = Qnew_finger;
    Qnew(7,7) = param_.Qnew_ball_x;
    Qnew(8,8) = param_.Qnew_ball_y;
  }

  std::vector<MatrixXd> Qha(Q_.size(), Qnew);

  solvers::C3MIQP opt(system_, Qha, R_, G_, U_, traj_desired, options);


  /// calculate the input given x[i]
  VectorXd input = opt.Solve(state, delta, w);

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
  //std::cout << dt << std::endl;

  //std::cout << "second call" << std::endl;

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

  VectorXd state_next = system2_.A_[0] * state + system2_.B_[0] * input + system2_.D_[0] * force / scaling2 + system2_.d_[0];

  // std::cout << "A: \n" << system2_.A_[0] << std::endl << std::endl;
  // std::cout << "B: \n" << system2_.B_[0] << std::endl << std::endl;
  // std::cout << "D: \n" << system2_.D_[0] << std::endl << std::endl;

  VectorXd force_des = VectorXd::Zero(6);
  force_des << force(0), force(2), force(4), force(5), force(6), force(7);

  VectorXd st_desired(force_des.size() + state_next.size() + ball_xyz_d.size() + ball_xyz.size() + 3);
  st_desired << state_next, force_des.head(6), ball_xyz_d, ball_xyz, ball.tail(3);
  state_contact_desired->SetDataVector(st_desired);
  state_contact_desired->set_timestamp(timestamp);

  /// update moving average filter
  if (moving_average_.size() < dt_filter_length_){
    moving_average_.push_back(timestamp - prev_timestamp_);
  }
  else{
    moving_average_.pop_front();
    moving_average_.push_back(timestamp - prev_timestamp_);
  }

  /// update prev variables
  prev_timestamp_ = timestamp;
  ball_xyz_prev_ = ball_xyz;

//  std::cout << "estimated v\n" << v_ball << std::endl;
//  std::cout << "actual v\n" << ball_dot.tail(3) << std::endl;
//  std::cout << "error norm\n" << (v_ball - ball_dot.tail(3)).norm() << std::endl;
//  std::cout << "alignment\n" << v_ball.dot(ball_dot.tail(3)) / (v_ball.norm() * ball_dot.tail(3).norm()) << std::endl;
//  std::cout << std::endl;

  // update kalman filter according to https://en.wikipedia.org/wiki/Kalman_filter
  // prediction step
  // MatrixXd F = MatrixXd::Zero(6,6);
  // MatrixXd B = MatrixXd::Zero(6,12);
  // MatrixXd H = MatrixXd::Zero(3, 6);
  // F.topLeftCorner(3,6) << system2_.A_[0].block(7, 7, 3, 3),
  //                         system2_.A_[0].block(7, 16, 3, 3);
  // F.bottomRightCorner(3,6) << system2_.A_[0].block(16, 7, 3, 3),
  //                             system2_.A_[0].block(16, 16, 3, 3);
  // B.topLeftCorner(3,12) << system2_.D_[0].block(7, 0, 3, 12);
  // B.bottomRightCorner(3,12) << system2_.D_[0].block(16, 0, 3, 12);
  // H.topLeftCorner(3,3) << MatrixXd::Identity(3,3);

  // VectorXd xhat_predict = VectorXd::Zero(6);
  // xhat_predict << state_next.segment(7, 3), state_next.tail(3);

  // std::cout << "F:\n" << F << std::endl;
  // std::cout << "B:\n" << B << std::endl;
  // std::cout << "H:\n" << H << std::endl;
  // std::cout << "D\n" << system2_.D_[0] << std::endl;
  // std::cout << "bot of D\n" << system2_.D_[0].block(16, 0, 3, 12) << std::endl;
  // std::cout << "xhat_predict\n" << xhat_predict << std::endl;
  // std::cout << "v_ball\n" << v_ball << std::endl;
  // std::cout << "d\n" << system2_.d_[0] << std::endl;

}

void C3Controller_franka::ProjectStateEstimate(Eigen::Vector3d endeffector, Eigen::Vector3d& estimate) const {
  Eigen::Vector3d dist_vec = estimate - endeffector;
  double R = param_.ball_radius;
  double r = param_.finger_radius;
  
  if (dist_vec.norm() < R+r){
    Eigen::Vector3d u(dist_vec(0), dist_vec(1), 0);
    double u_norm = u.norm();
    double du = sqrt((R+r)*(R+r) - dist_vec(2)*dist_vec(2)) - u_norm;

    estimate = estimate + du * u / u_norm;
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