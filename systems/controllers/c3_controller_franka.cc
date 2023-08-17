#include "c3_controller_franka.h"

#include <utility>
#include <chrono>
#include <iterator>
#include <vector>
#include <cmath>


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

Eigen::Vector3d generate_next_position(std::vector<Eigen::Vector3d> points, double t);
double generate_next_z(double current, double end, double t);

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
          TimestampedVector<double>(44), &C3Controller_franka::CalcControl)
      .get_index();

  q_map_franka_ = multibody::makeNameToPositionsMap(plant_franka_);
  v_map_franka_ = multibody::makeNameToVelocitiesMap(plant_franka_);
  q_map_ = multibody::makeNameToPositionsMap(plant_);
  v_map_ = multibody::makeNameToVelocitiesMap(plant_);

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

    //  std::cout<<"ENTERING START POSITION"<<std::endl;

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
  
  //print ee pose snippet
  // std::cout<< "end_effector "<< end_effector <<std::endl;
  // end_effector = -end_effector;
  // std::cout<<"x_coordinate "<<end_effector[0]<<std::endl;
  // end_effector[0] = -end_effector[0];
//  std::cout<<"- x_coordinate "<<end_effector[0];

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
  VectorXd u = 1000*VectorXd::Ones(3);

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
                      traj_desired_vector(q_map_.at("base_z")));
  Vector3d error_xy = ball_xyz_d - ball_xyz;
  error_xy(2) = 0;
  Vector3d error_hat = error_xy / error_xy.norm();

  // compute phase
  double shifted_time = timestamp - settling_time -  return_phase;
  if (shifted_time < 0) shifted_time += period;
  double ts = shifted_time - period * floor((shifted_time / period));
  double back_dist = param_.gait_parameters(0);
  
  /// rolling phase
  // if ( ts < roll_phase ) {
    //Maintaining roll phase without time based heuristic. Instead constantly rolling except when making decision to reposition instead.
    traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[7];
    traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[8];
    traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] + 0.004;
  // }
  /// upwards phase
  // else if (ts < roll_phase + return_phase / 3){
  //   traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = state[0]; //0.55;
  //   traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = state[1]; //0.1;
  //   traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = param_.gait_parameters(1) + table_offset;

  // }
  // /// side ways phase
  // else if( ts < roll_phase + 2 * return_phase / 3 ) {
  //   // optimal_cost_ = min;
  //   // optimal_sample_ = candidate_states[index];

  //   traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = optimal_sample_[0]; // state[7] - back_dist*error_hat(0);
  //   traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = optimal_sample_[1]; // state[8] - back_dist*error_hat(1);
  //   traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = 0.07; //param_.gait_parameters(2) + table_offset; //optimal_sample_[2]; //
  // }
  /// position finger phase
  // else{
    //isolating repositioning to the best state selected in the previous sampling run
    // traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = optimal_sample_[0]; //state[7] - back_dist*error_hat(0);
    // traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = optimal_sample_[1]; //state[8] - back_dist*error_hat(1);
    // traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = 0.07; //param_.gait_parameters(3) + table_offset;  //optimal_sample_[2]; //
  // }
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


  /// update context
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

  // if (ts > roll_phase){
  //   double Qnew_finger = param_.Qnew_finger;
  //   Qnew(0,0) = Qnew_finger;
  //   Qnew(1,1) = Qnew_finger;
  //   Qnew(2,2) = Qnew_finger;
  //   Qnew(7,7) = param_.Qnew_ball_x;
  //   Qnew(8,8) = param_.Qnew_ball_y;
  // }

  std::vector<MatrixXd> Qha(Q_.size(), Qnew);

  solvers::C3MIQP opt(system_, Qha, R_, G_, U_, traj_desired, options,
    warm_start_delta_, warm_start_binary_, warm_start_x_,
    warm_start_lambda_, warm_start_u_, true);


  
  
  //Position sampling
  // Eigen::VectorXd ball_orientation_vector = q_plant.head(4);
  // // std::cout<<"quaternion orientation of ball "<<ball_orientation_vector<<std::endl;
  // Eigen::Quaternion<double> ball_orientation(ball_orientation_vector[0],ball_orientation_vector[1],ball_orientation_vector[2],ball_orientation_vector[3]);
  // // ball_orientation.w
  // Matrix3d ball_rot_matrix = ball_orientation.toRotationMatrix();
  // std::cout<<"rot matrix orientation of all "<<ball_rot_matrix<<std::endl;
  // // Eigen::RotationMatrix ball_rot_matrix(&ball_orientation);
  // // std::cout<<"rot matrix orientation of all "<<ball_rot_matrix<<std::endl;


 ////////////////////////////////////////////////////////////////////////////////////
 ////////////////////////////////////////////////////////////////////////////////////
 ////////////////////////////////////////////////////////////////////////////////////

  // if (ts < roll_phase) {



    //Multi-sample code piece
    double x_samplec; //center of sampling circle
    double y_samplec; //center of sampling circle
    double radius = 0.052; //radius of sampling circle (0.05) //0.06 //0.08
    int num_samples = 4;
    double theta = 360 / num_samples * PI / 180;
    double angular_offset = 0 * PI/180;

    
    std::vector<VectorXd> candidate_states(num_samples, VectorXd::Zero(plant_.num_positions() + plant_.num_velocities()));
    VectorXd st_desired(6 + optimal_sample_.size() + orientation_d.size() + ball_xyz_d.size() + ball_xyz.size() + true_ball_xyz.size() + 3 + 3);

    VectorXd test_state(plant_.num_positions() + plant_.num_velocities());
    // Vector3d curr_ee = end_effector; //end effector current position
    Vector3d ee = end_effector; //end effector test position
    // VectorXd cost_vector = VectorXd::Zero(num_samples);

    // double theta = 360/num_samples;
    // std::cout<<"Theta"<<theta<<std::endl;
    // std::cout<<"state vector"<<state<<std::endl;
    // std::cout<<"Ball x position"<<state[7]<<std::endl;
    // Vector3d ball_xyz = ball.tail(3);
    // std::cout<< "Ball xyz here :::::: "<< ball_xyz <<std::endl;
    x_samplec = ball_xyz[0]; //state[7];
    y_samplec = ball_xyz[1]; //state[8];
    // std::cout<<"current ball position: "<< x_samplec <<" , "<< y_samplec <<std::endl;

    double phase = atan2(ee[1]-y_samplec, ee[0]-x_samplec);    //What would happen if the ee is right above the ball? Unlikely to happen, at least numerically ee will lean to one direction
    // double phase = 0;
   
    // std::cout<<"phase angle = "<< phase * 180/PI << std::endl;

    std::vector<double> cost_vector(num_samples);


    for (int i = 0; i < num_samples; i++) {
      
      double pos_x = 0;
      double pos_y = 0;

      // std::cout<<"sample "<< i << " angle = "<< (i*theta + phase + angular_offset) * 180/PI <<std::endl;

      pos_x  = x_samplec + radius * cos(i*theta + phase + angular_offset); //state[7]
      pos_y = y_samplec + radius * sin(i*theta + phase + angular_offset);

      // std::cout<<"sample position "<< i << " : "<< pos_x <<" , "<< pos_y <<std::endl;

      // test_state << ee, q_plant.head(4), ball_xyz, end_effector_dot, v_plant.tail(6);  //current state with modified ee position

      VectorXd test_q(10);

      VectorXd test_v(9);

      test_q << q;
      test_v << v;

      test_q[0] = pos_x;
      test_q[1] = pos_y;
      test_q[2] = 0.02;

//      std::cout << "test_q" << std::endl;

      /// update autodiff
      VectorXd xu_test(plant_f_.num_positions() + plant_f_.num_velocities() +
          plant_f_.num_actuators());
      xu_test << test_q, test_v, u;
      auto xu_ad_test = drake::math::InitializeAutoDiff(xu_test);

      plant_ad_f_.SetPositionsAndVelocities(
          &context_ad_f_,
          xu_ad_test.head(plant_f_.num_positions() + plant_f_.num_velocities()));
      multibody::SetInputsIfNew<AutoDiffXd>(
          plant_ad_f_, xu_ad_test.tail(plant_f_.num_actuators()), &context_ad_f_);

      plant_f_.SetPositions(&context_f_, test_q);
      plant_f_.SetVelocities(&context_f_, test_v);
      multibody::SetInputsIfNew<double>(plant_f_, u, &context_f_);

      auto test_system_scaling_pair = solvers::LCSFactoryFranka::LinearizePlantToLCS(
          plant_f_, context_f_, plant_ad_f_, context_ad_f_, contact_pairs,
          num_friction_directions_, mu_, 0.1);

      solvers::LCS test_system = test_system_scaling_pair.first;



//      ///trying outside .simulate
//      double scaling2 = test_system_scaling_pair.second;
//      drake::solvers::MobyLCPSolver<double> LCPSolver_test;
//      VectorXd force_test; //This contains the contact forces.
//      //tangential forces and normal forces for two contacts --> gamma slack variable, ball+ee and ball+ground from stewart trinkle formulation
//
//      //double scaling2 = 1;
//      VectorXd input_test = VectorXd::Zero(3);
//      auto flag_test = LCPSolver_test.SolveLcpLemkeRegularized(test_system.F_[0], test_system.E_[0] * scaling2 * state + test_system.c_[0] * scaling2 + test_system.H_[0] * scaling2 * input_test,
//                                                     &force_test);
//
//      std::cout << "flag_test" << std::endl;
//      std::cout << flag_test << std::endl;

      test_state << test_q.head(3), state.tail(16);

      candidate_states[i] = test_state;

      solvers::C3MIQP opt_test(test_system, Qha, R_, G_, U_, traj_desired, options,
                               warm_start_delta_, warm_start_binary_, warm_start_x_,
                               warm_start_lambda_, warm_start_u_, true);


    // solvers::C3MIQP opt(system_, Qha, R_, G_, U_, traj_desired, options,
    // warm_start_delta_, warm_start_binary_, warm_start_x_,
    // warm_start_lambda_, warm_start_u_, true);
    
      ///ADDING DELTA
        if (options.delta_option == 1) {
        /// reset delta and w (option 1)
        delta = delta_reset;
        w = w_reset;
        for (int j = 0; j < N; j++) {
          //delta[j].head(n) = xdesired_[0]; //state
          delta[j].head(n) << test_state; //state
        }
      } else {
        /// reset delta and w (default option)
        delta = delta_reset;
        w = w_reset;
  }

      vector<VectorXd> fullsol = opt_test.SolveFullSolution(test_state, delta, w);  //outputs full z

      //std::cout << "test state" << test_state.head(10) - test_q << std::endl;

      vector<VectorXd> optimalinputseq = opt_test.OptimalInputSeq(fullsol);  //outputs u over horizon
      // double cost = opt_test.CalcCost(test_state, optimalinputseq); //purely positional cost
      // std::cout<< "purely translational cost of sample "<< i << " = " << std::sqrt(std::pow((test_q[0]-ee[0]),2)+std::pow((test_q[1]-ee[1]),2)) << std::endl;
      double cost = opt_test.CalcCost(test_state, optimalinputseq) + 1 * std::sqrt(std::pow((test_q[0]-ee[0]),2) + std::pow((test_q[1]-ee[1]),2)); //+ std::pow((test_q[2]-ee[2]),2)); 
      cost_vector[i] = cost;

      // std::cout << "This is the cost of sample " << i << " : " << cost << std::endl;


    }

    double min = *std::min_element(cost_vector.begin(), cost_vector.end());
    // double* min_index = std::min_element(std::begin(cost_vector), std::end(cost_vector));
    // std::cout << "index of smallest element: " << std::distance(std::begin(cost_vector), min_index);
    // std::cout << " minimum value is " << min << std::endl;

    

    std::vector<double>::iterator it = std::min_element(std::begin(cost_vector), std::end(cost_vector));
    int index = std::distance(std::begin(cost_vector), it);
    // std::cout << "index of smallest element: " << index <<std::endl;
    // std::cout << " chosen sample " << index << " and state ee position : " << candidate_states[index] << std::endl;
    
    vector<VectorXd> fullsol = opt.SolveFullSolution(state, delta, w);  //outputs full z
  vector<VectorXd> optimalinputseq = opt.OptimalInputSeq(fullsol);  //outputs u over horizon
  double curr_ee_cost = opt.CalcCost(state, optimalinputseq); //computes cost for given x0
  std::cout<<"This is the current cost "<<curr_ee_cost<<std::endl;

    double hyp = 5;
    if(C3_flag_ == 0){
        hyp = 3;
    }
    else{
        hyp = 17;
    }
    // if (reposition_flag_ == 1){
    //     hyp = 0;
    // }
    
    //update to best state
    if(curr_ee_cost - min >= hyp){//75){ //heuristic threshold for if the difference between where I am and where I want to be is more than the threshold, then move towards that point
      //if(min < optimal_cost_) 
         //if the min cost you have is lesser than the previous optimal cost, then reposition. 
        //  C3_flag_ = 0;
         std::cout << "Decided to reposition"<<std::endl;
         optimal_cost_ = min; 
         optimal_sample_ = candidate_states[index];

         std::cout<<"Min : "<<min<<std::endl;
        //  Eigen::Vector3d v2(1.0, 0.0, 0.0);
        //  double normV2 = v2.norm();
        //  std::cout << "The norm of v2 is: " << normV2 << std::endl;

        //  std::cout<<"original st desired " << optimal_sample_.head(3) <<std::endl;
        //  std::cout<<"current state " << state.head(3) <<std::endl;

         
        //  std::cout<<"ballxyz " << ball_xyz <<std::endl;
        // //  way_point[0] = way_point[0] + 0.08;
        // //  way_point[1] = way_point[1] + 0.08;
        //  way_point[2] = way_point[2] + 0.08;

         
         std::vector<Vector3d> points(4, VectorXd::Zero(3));
         
         points[0] = end_effector;
        //  points[1] = way_point;
         points[3] = optimal_sample_.head(3);
        //  std::cout<<"optimal sample"<<points[3]<<std::endl;

         Eigen::Vector3d way_point1  = points[0] + 0.25*(points[3] - points[0]) - ball_xyz  ;
         points[1] = ball_xyz + (radius + 0.01) * way_point1/way_point1.norm();
        //  std::cout<<"way_point1"<<way_point1<<std::endl;

        //KIND OF WORKING
        //  Eigen::Vector3d way_point1  = points[0] + 0.5*(points[3] - points[0]);
        //  way_point1[2] = 0.06;
        // //  std::cout<<"ee"<<points[0]<<std::endl;
        // //  std::cout<<"way_point1"<<way_point1<<std::endl;
        // //  std::cout<<"ee"<<points[2]<<std::endl;
        //  points[1] = way_point1;


         Eigen::Vector3d way_point2  = points[0] + 0.75*(points[3] - points[0]) - ball_xyz;
         points[2] = ball_xyz + (radius + 0.01) * way_point2/way_point2.norm();
        // std::cout << "The norm of v2 is: " << points[1].norm() << std::endl;
         
         double t = 0.02;

         Eigen::Vector3d next_point = points[0] + t*(-3*points[0] + 3*points[1]) + std::pow(t,2) * (3*points[0] -6*points[1] + 3*points[2]) + std::pow(t,3) * (-1*points[0] +3*points[1] -3*points[2] + points[3]);
         
         

        //  std::cout<<"end effector z " << end_effector[2] <<std::endl;

        //  std::cout<<"points 0 " << points[0] <<std::endl;
        //  std::cout<<" " <<std::endl;
        //  std::cout<<"points 1 " << points[1] <<std::endl;
        //  std::cout<<" " <<std::endl;
        //  std::cout<<"points 2 " << points[2] <<std::endl;
        //  std::cout<<" " <<std::endl;

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
         
        //  for(int i = 0; i < 660; i++){
            //  Eigen::Vector3d next_point = generate_next_position(points, 0.1);
            //  std::cout<<"generated next point " << next_point <<std::endl;
        //      double next_z = generate_next_z(end_effector[2], 0.2, 2*i*0.015); 
             
        //      std::cout<<"Moving down"<<std::endl;
        //      st_desired << next_point.head(3), orientation_d, optimal_sample_.tail(16), VectorXd::Zero(6), ball_xyz_d, ball_xyz, true_ball_xyz;
        //     //  st_desired << next_point.head(2), next_z , orientation_d, optimal_sample_.tail(16), VectorXd::Zero(6), ball_xyz_d, ball_xyz, true_ball_xyz;

        //      state_contact_desired->SetDataVector(st_desired);
        //      state_contact_desired->set_timestamp(timestamp);
        //  }
         

        //  points[0] = end_effector;
        // //  points[1] = way_point;
        //  points[2] = ball_xyz.head(2), 0.08;

        //  for(int i = 0; i < 66; i++){
        //      Eigen::Vector3d next_point = generate_next_position(points, i*0.015);
        //      std::cout<<"generated next point " << next_point <<std::endl;
        //      double next_z = generate_next_z(end_effector[2], 0.08, i*0.015); 
             
        //      std::cout<<"Moving up"<<std::endl;
          st_desired << next_point.head(3), orientation_d, optimal_sample_.tail(16), VectorXd::Zero(6), candidate_states[0].head(3), candidate_states[1].head(3), candidate_states[2].head(3), candidate_states[3].head(3), optimal_sample_.head(3);

        //      state_contact_desired->SetDataVector(st_desired);
        //      state_contact_desired->set_timestamp(timestamp);
         std::cout<<"hyp in reposition "<< hyp <<std::endl;
        //  if (curr_ee_cost - min <= hyp + 0.02){
        //       C3_flag_ = 0;
        //       reposition_flag_ = 0;
        //       std::cout<< "got close to optimal solution and c3 flag is " << C3_flag_ << std::endl;
        //  }
         
         
         
         
        //  std::cout <<"Repositioned!! "<<std::endl;

        //  /// update moving average filter and prev variables
        //  if (moving_average_.size() < dt_filter_length_){
        //      moving_average_.push_back(timestamp - prev_timestamp_);
        //  }
        //  else {
        //     moving_average_.pop_front();
        //     moving_average_.push_back(timestamp - prev_timestamp_);
        //  }
        //     prev_timestamp_ = timestamp;
    //      traj_desired_vector[q_map_.at("tip_link_1_to_base_x")] = optimal_sample_[0]; //state[7] - back_dist*error_hat(0);
    //      traj_desired_vector[q_map_.at("tip_link_1_to_base_y")] = optimal_sample_[1]; //state[8] - back_dist*error_hat(1);
    //      traj_desired_vector[q_map_.at("tip_link_1_to_base_z")] = 0.08;
    }
    else{
      // VectorXd input = opt.Solve(candidate_states[index], delta, w);
        C3_flag_ = 1; //when repositioning is good enough, switch flag to 0

  /// calculate the input given x[i]
  //std::cout<<"original sol"<< std::endl;
  
    ///ADDING DELTA
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


  VectorXd input = opt.Solve(state, delta, w);
  std::cout<<"Using C3 "<<std::endl;
  std::cout<<"Min : "<<min<<std::endl;
  // std::cout<<"This is where the end effector is rn : "<< state.head(3) <<std::endl;
  // C3_flag_++;
  

  warm_start_x_ = opt.GetWarmStartX();
  warm_start_lambda_ = opt.GetWarmStartLambda();
  warm_start_u_ = opt.GetWarmStartU();
  warm_start_delta_ = opt.GetWarmStartDelta();
  warm_start_binary_ = opt.GetWarmStartBinary();

  // compute dt based on moving average filter
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
  VectorXd force; //This contains the contact forces. 
  //tangential forces and normal forces for two contacts --> gamma slack variable, ball+ee and ball+ground from stewart trinkle formulation

  auto flag = LCPSolver.SolveLcpLemkeRegularized(system2_.F_[0], system2_.E_[0] * scaling2 * state + system2_.c_[0] * scaling2 + system2_.H_[0] * scaling2 * input,
                                                 &force);
                                                 //The final force solution comes with 12 components (stewart trinkle formulation). 
  //In order, these components are : 
  //gamma_ee, gamma_bg (slack ee and slack ball-ground)
  //normal force between ee and ball lambda_eeb^n
  //normal force between ball and ground lambda_bg^n
  //4 tangential forces between ee and ball lambda_eeb^{t1-t4}
  //4 tangential forces between ball and ground lambda_bg^{t1-t4}

  (void)flag; // suppress compiler unused variable warning

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
  force_des << force(0), force(2), force(4), force(5), force(6), force(7);  //We only care about the ee and ball forces when we send deired forces and here we extract those from the solution and send it in force_des.


  //Cost computation piece
  // vector<VectorXd> fullsol = opt.SolveFullSolution(state, delta, w);  //outputs full z
  // vector<VectorXd> optimalinputseq = opt.OptimalInputSeq(fullsol);  //outputs u over horizon
  // double cost = opt.CalcCost(state, optimalinputseq); //computes cost for given x0
  // std::cout<<"This is the cost "<<cost<<std::endl;

  // VectorXd st_desired(force_des.size() + state_next.size() + orientation_d.size() + ball_xyz_d.size() + ball_xyz.size() + true_ball_xyz.size());

  
  //The next two lines are only used to verify sample direction
  ///testing
//  VectorXd state_next_test = VectorXd::Zero(19);
//  state_next_test[0] = ball_xyz[0] + 0.05 * cos(PI/2); //+ (10*PI/180)
//  state_next_test[1] = ball_xyz[1] + 0.05 * sin(PI/2);
//  state_next_test[2] = 0.07;
//  state_next = state_next_test;

// state_next = candidate_states[index];
  std::cout<<"hyp in C3 "<< hyp <<std::endl;
  if (curr_ee_cost - min >= 10){
    std::cout<< "Can't make any progress from here and flag is : " << C3_flag_ << std::endl;
    C3_flag_ = 0;
    // reposition_flag_ = 1;
  }

  st_desired << state_next.head(3), orientation_d, state_next.tail(16), force_des.head(6), candidate_states[0].head(3), candidate_states[1].head(3), candidate_states[2].head(3), candidate_states[3].head(3), optimal_sample_.head(3);
  // std::cout<<"here"<<std::endl;
    }
   

    // vector<VectorXd> fullsol_curr = opt.SolveFullSolution(state, delta, w);  //outputs full z
    // vector<VectorXd> optimalinputseq_curr = opt.OptimalInputSeq(fullsol_curr);  //outputs u over horizon
    // double cost_opt = opt.CalcCost(state, optimalinputseq);
    // std::cout << "real cost " << cost_opt << std::endl;
    // std::cout << "real state : " << state;

    // state << candidate_states[index];  //Uncomment when confirmed

  
 
  
  //Full state is 19 dimensions.
  //state_next.head(3) - first three elements --- this is the ee position
  //followed by ball orientation (4 elements as it is represented using quaternions) used for visualization probably
  //followed by last 16 elements of the vector which includes everything except ee position
  //fb ee MapVelocityToQDotball MapVelocityToQDotball angular velocity
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
  // std::cout << "here" << std::endl;
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
//  Vector3d average = Vector3d::Zero(3); state_next.head(3), orientation_d, state_next.tail(16), force_des.head(6), ball_xyz_d, ball_xyz, true_ball_xyz;

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

Eigen::Vector3d generate_next_position(std::vector<Eigen::Vector3d> points, double t){
  Eigen::Vector3d a0;
  Eigen::Vector3d a1;
  Eigen::Vector3d a2;
  Eigen::Vector3d a3;

  a0 = points[0];
  // std::cout<< " a0 = " << a0 <<std::endl;

  a1 = VectorXd::Zero(3);
  // std::cout<< " a1 = " << a1 <<std::endl;

  a2 = 3 * (points[1] - points[0]);
  // std::cout<< " a2 = " << a2 <<std::endl;

  a3 = -2 * (points[1] - points[0]);
  // std::cout<< " a3 = " << a3 <<std::endl;

  // std::cout<<a0 + a1 * t + a2 * std::pow(t,2) + a3 * std::pow(t,3)<<std::endl;
  return a0 + a1 * t + a2 * std::pow(t,2) + a3 * std::pow(t,3);
}

double generate_next_z(double current, double end, double t){
  return current + t * (end - current);
}