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


//#include
//"external/drake/common/_virtual_includes/autodiff/drake/common/eigen_autodiff_types.h"

using std::vector;

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::VectorXd;
using std::vector;

using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::Quaterniond;
using std::vector;
using drake::multibody::JacobianWrtVariable;
using drake::math::RotationMatrix;

// TODO: remove this (only used for Adam debugging)
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
  int num_positions = plant_.num_positions();
  int num_velocities = plant_.num_velocities();
  int num_inputs = plant_.num_actuators();
  int num_states = num_positions + num_velocities;


  state_input_port_ =
      this->DeclareVectorInputPort(
              "x, u, t",
              OutputVector<double>(14, 13, 7))
          .get_index();


  state_output_port_ = this->DeclareVectorOutputPort(
                                 "x_lambda, t", TimestampedVector<double>(28),
                                 &C3Controller_franka::CalcControl)
                             .get_index();

  // get c3_parameters
  param_ = drake::yaml::LoadYamlFile<C3Parameters>(
      "examples/franka_trajectory_following/parameters.yaml");

}

void C3Controller_franka::CalcControl(const Context<double>& context,
                               TimestampedVector<double>* state_contact_desired) const {
  

   // get values
   auto robot_output = (OutputVector<double>*)this->EvalVectorInput(context, state_input_port_);
   double timestamp = robot_output->get_timestamp();
   VectorXd state_franka(27);
   state_franka << robot_output->GetPositions(), robot_output->GetVelocities();

   //update franka context
   plant_franka_.SetPositions(&context_franka_, robot_output->GetPositions());
   plant_franka_.SetVelocities(&context_franka_, robot_output->GetVelocities());

   // forward kinematics
   Vector3d EE_offset_ = param_.EE_offset;
   const drake::math::RigidTransform<double> H_mat =
       plant_franka_.EvalBodyPoseInWorld(context_franka_, plant_franka_.GetBodyByName("panda_link8"));
   const RotationMatrix<double> Rotation = H_mat.rotation();
   Vector3d end_effector = H_mat.translation() + Rotation*EE_offset_;

   auto EE_frame_ = &plant_franka_.GetBodyByName("panda_link8").body_frame();
   auto world_frame_ = &plant_franka_.world_frame();

   MatrixXd J_fb (6, plant_franka_.num_velocities());
   plant_franka_.CalcJacobianSpatialVelocity(
       context_franka_, JacobianWrtVariable::kV,
       *EE_frame_, EE_offset_,
       *world_frame_, *world_frame_, &J_fb);
   MatrixXd J_franka = J_fb.block(0, 0, 6, 7);

   VectorXd end_effector_dot = ( J_franka * (robot_output->GetVelocities()).head(7) ).tail(3);



   VectorXd ball = robot_output->GetPositions().tail(7);
   VectorXd ball_dot = robot_output->GetVelocities().tail(6);
   VectorXd state(plant_.num_positions() + plant_.num_velocities());
   state << end_effector, ball, end_effector_dot, ball_dot;
   VectorXd q(10);
   q << end_effector, ball;
   VectorXd v(9);
   v << end_effector_dot, ball_dot;
   VectorXd u = VectorXd::Zero(3);

   VectorXd traj_desired_vector = pp_.value(timestamp);
   Vector3d ball_xyz_d(traj_desired_vector(7),  
                       traj_desired_vector(8),
                       traj_desired_vector(9));

  Vector3d ball_xyz(state[7], state[8], state[9]);
  Vector3d error_xy = ball_xyz_d - ball_xyz;
  error_xy(2) = 0;
  Vector3d error_hat = error_xy / error_xy.norm();

  double period = param_.period;
  double duty_cycle = param_.duty_cycle;
  double return_cycle = 1-duty_cycle;
  double settling_time = param_.stabilize_time1 + param_.move_time + param_.stabilize_time2;
  double shifted_time = timestamp - settling_time;
  double ts = shifted_time - period * floor((shifted_time / period));

  if (ts > period*duty_cycle && ts < period * (duty_cycle+0.33*return_cycle)){
    traj_desired_vector[0] = state[0]; //- 0.05;
    traj_desired_vector[1] = state[1]; //+ 0.01;
    //traj_desired_vector[2] = 0.075;
  }
  else{ // otherwise go to top of ball
    traj_desired_vector[0] = state[7] - 0.03*error_hat(0); //- 0.05;
    traj_desired_vector[1] = state[8] - 0.03*error_hat(1); //+ 0.01;
    traj_desired_vector[2] = 0.07;
  }

//  traj_desired_vector[0] = state[7] - 0.03*error_hat(0); //- 0.05;
//  traj_desired_vector[1] = state[8] - 0.03*error_hat(1); //+ 0.01;
//  traj_desired_vector[2] = 0.07;


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
   Qnew = Q_[0];




  //std::cout << "ts: " << ts << std::endl;

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

   ///calculate state and force
   drake::solvers::MobyLCPSolver<double> LCPSolver;
   VectorXd force;


  auto system_scaling_pair2 = solvers::LCSFactoryFranka::LinearizePlantToLCS(
      plant_f_, context_f_, plant_ad_f_, context_ad_f_, contact_pairs,
      num_friction_directions_, mu_, param_.dt);

  solvers::LCS system2_ = system_scaling_pair2.first;
  double scaling2 = system_scaling_pair2.second;


   auto flag = LCPSolver.SolveLcpLemkeRegularized(system2_.F_[0], system2_.E_[0] * scaling2 * state + system2_.c_[0] * scaling2 + system2_.H_[0] * scaling2 * input,
                                      &force);


   VectorXd state_next = system2_.A_[0] * state + system2_.B_[0] * input + system2_.D_[0] * force / scaling2 + system2_.d_[0];

 VectorXd force_des = VectorXd::Zero(6);
 force_des << force(0), force(2), force(4), force(5), force(6), force(7);

  VectorXd traj = pp_.value(timestamp);

 VectorXd st_desired(force_des.size() + state_next.size() + ball_xyz_d.size());
 st_desired << state_next, force_des.head(6), ball_xyz_d;

 std::vector<Eigen::Vector3d> target;
 if (timestamp <= settling_time){
   Eigen::Vector3d start = param_.initial_start;
   Eigen::Vector3d finish = param_.initial_finish;
   finish(0) = param_.x_c + param_.traj_radius * sin(param_.phase * 3.14159265 / 180);
   finish(1) = param_.y_c + param_.traj_radius * cos(param_.phase * 3.14159265 / 180) - 0.03;
   target = move_to_initial_position(start, finish, timestamp,
          param_.stabilize_time1, param_.move_time, param_.stabilize_time2);

   st_desired = VectorXd::Zero(28);
   st_desired.head(3) << target[0];
   st_desired(7) = finish(0);
   st_desired(8) = finish(1);
   st_desired(9) = param_.ball_radius;
   st_desired.tail(3) << ball_xyz_d;
 }


   state_contact_desired->SetDataVector(st_desired);
   state_contact_desired->set_timestamp(timestamp);


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