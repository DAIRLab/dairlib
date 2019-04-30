#include "systems/controllers/cp_lipm_traj_gen.h"

#include <algorithm>    // std::min
#include <math.h>
#include <chrono>   // measuring runtime
#define PI 3.14159265

using namespace std::chrono;

using Eigen::Vector3d;
using Eigen::Vector4d;

using std::cout;
using std::endl;

namespace dairlib {
namespace systems {

LIPMTrajGenerator::LIPMTrajGenerator(RigidBodyTree<double> * tree,
                                     double desiredCoMHeight,
                                     double stance_duration_per_leg,
                                     int left_stance_state,
                                     int right_stance_state,
                                     int left_foot_idx,
                                     int right_foot_idx):
  tree_(tree),
  desiredCoMHeight_(desiredCoMHeight),
  stance_duration_per_leg_(stance_duration_per_leg),
  left_stance_state_(left_stance_state),
  right_stance_state_(right_stance_state),
  left_foot_idx_(left_foot_idx),
  right_foot_idx_(right_foot_idx) {

  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(OutputVector<double>(
                  tree.get_num_positions(),
                  tree.get_num_velocities(),
                  tree.get_num_actuators())).get_index();

  FSM_port_ = this->DeclareVectorInputPort(
                TimestampedVector<double>(1)).get_index();

  this->DeclareAbstractOutputPort(&LIPMTrajGenerator::CalcTraj);

  // State variables inside this controller block
  this->DeclarePeriodicDiscreteUpdate(1.0 / 200.0); // Match with the publish rate

  // The time of the last touch down
  prev_td_time_idx_ = this->DeclareDiscreteState(1);
  // The last state of FSM
  prev_fsm_state_idx_ = this->DeclareDiscreteState(-VectorXd::Ones(1));
}


void LIPMTrajGenerator::DoCalcDiscreteVariableUpdates(
  const Context<double>& context,
  const std::vector<const DiscreteUpdateEvent<double>*>&,
  DiscreteValues<double>* discrete_state) const {

  // Read in finite state machine
  const TimestampedVector<double>* FSMOutput = (TimestampedVector<double>*)
      this->EvalVectorInput(context, FSM_port_);
  VectorXd fsm_state = FSMOutput->get_data();

  auto prev_td_time = discrete_state->get_mutable_vector(
                        prev_td_time_idx_).get_mutable_value();
  auto prev_fsm_state = discrete_state->get_mutable_vector(
                          prev_fsm_state_idx_).get_mutable_value();

  if (fsm_state(0) != prev_fsm_state(0)) {  //if at touchdown
    prev_fsm_state(0) = fsm_state(0);

    // Get time
    const OutputVector<double>* robotOutput = (OutputVector<double>*)
        this->EvalVectorInput(context, state_port_);
    double timestamp = robotOutput->get_timestamp();
    double current_sim_time = static_cast<double>(timestamp);
    prev_td_time(0) = current_sim_time;
  }
}


void LIPMTrajGenerator::CalcTraj(const Context<double>& context,
                                 ExponentialPlusPiecewisePolynomial<double>* traj) const {

  // Read in current state
  const OutputVector<double>* robotOutput = (OutputVector<double>*)
      this->EvalVectorInput(context, state_port_);
  VectorXd current_velocity = robotOutput->GetVelocities();

  // Read in finite state machine
  const TimestampedVector<double>* FSMOutput = (TimestampedVector<double>*)
      this->EvalVectorInput(context, FSM_port_);
  VectorXd fsm_state = FSMOutput->get_data();

  // Get discrete states
  const auto prev_td_time = context.get_discrete_state(
                              prev_td_time_idx_).get_value();

  // Get time
  double timestamp = robotOutput->get_timestamp();
  double current_sim_time = static_cast<double>(timestamp);

  double start_time_of_this_interval = prev_td_time(0);
  double end_time_of_this_interval = prev_td_time(0) + stance_duration_per_leg_;
  // Ensure current_sim_time < end_time_of_this_interval to avoid error in creating trajectory.
  if ((end_time_of_this_interval <= current_sim_time + 0.001)) {
    end_time_of_this_interval = current_sim_time + 0.002;
  }
  // cout << "current_sim_time = " << current_sim_time << ", end_time_of_this_interval = " << end_time_of_this_interval << endl;
  // std::cout<<"start_time_of_this_interval: "<<start_time_of_this_interval<<"\n";
  // std::cout<<"end_time_of_this_interval: "<<end_time_of_this_interval<<"\n";

  // Kinematics cache and indices
  KinematicsCache<double> cache = tree_->CreateKinematicsCache();
  VectorXd q = robotOutput->GetPositions();

  // Modify the quaternion in the begining when the state is not received from
  // the robot yet
  // Always remember to check 0-norm quaternion when using doKinematics
  if (q(3) == 0 && q(4) == 0 && q(5) == 0 && q(6) == 0)
    q(3) = 1;

  cache.initialize(q);
  tree_->doKinematics(cache);
  int stance_foot_index;
  int swing_foot_index;
  if (fsm_state(0) == 3) { // right stance
    stance_foot_index = right_foot_idx_;
    swing_foot_index = left_foot_idx_;
  }
  else {
    stance_foot_index = left_foot_idx_;
    swing_foot_index = right_foot_idx_;
  }

  // Get center of mass position and velocity
  Vector3d CoM = tree_->centerOfMass(cache);
  MatrixXd J = tree_->centerOfMassJacobian(cache);
  Vector3d dCoM = J * current_velocity;
  // std::cout<<"center of mass:\n"<<CoM<<"\n";
  // std::cout<<"dCoM:\n"<<dCoM<<"\n";

  // Stance foot position (Forward Kinematics)
  Eigen::Isometry3d stance_foot_pose =
    tree_->CalcBodyPoseInWorldFrame(cache, tree_->get_body(stance_foot_index));
  Eigen::Vector3d stance_foot_pos = stance_foot_pose.translation();
  // std::cout<<"stance_foot_pos^T = "<<stance_foot_pos.transpose()<<"\n";

  ////////////////////////// CoM Traj  /////////////////////////////////////////
  // set the CoM at touchdown event as initial CoM (same for dCoM)

  // Get CoMwrtFoot for LIPM
  // Prevent zCoM_wrt_foot from being non-positive (in simulation) cause of
  // calculation of omega
  const double xCoM_wrt_foot = CoM(0) - stance_foot_pos(0);
  const double yCoM_wrt_foot = CoM(1) - stance_foot_pos(1);
  const double zCoM_wrt_foot =
    (CoM(2) - stance_foot_pos(2)) ? (CoM(2) - stance_foot_pos(2)) : 1e-3;
  const double dxCoM_wrt_foot = dCoM(0);
  const double dyCoM_wrt_foot = dCoM(1);
  const double dzCoM_wrt_foot = dCoM(2);
  // std::cout<<"(x,y,z,dx,dy) of CoMwrtFoot = "<<xCoM_wrt_foot<<","<<yCoM_wrt_foot<<","<<zCoM_wrt_foot<<","<<dxCoM_wrt_foot<<","<<dyCoM_wrt_foot<<"\n";

  // create a 3D one-segment polynomial for ExponentialPlusPiecewisePolynomial
  std::vector<double> T_waypoint_com = {current_sim_time, end_time_of_this_interval};

  std::vector<MatrixXd> Y(T_waypoint_com.size(), MatrixXd::Zero(3, 1));
  Y[0](0, 0) = stance_foot_pos(0);
  Y[1](0, 0) = stance_foot_pos(0);
  Y[0](1, 0) = stance_foot_pos(1);
  Y[1](1, 0) = stance_foot_pos(1);
  Y[0](2, 0) = (CoM(2) > 0)? CoM(2) : 1e-3;
  // Y[0](2, 0) = desiredCoMHeight_;
  Y[1](2, 0) = desiredCoMHeight_;

  MatrixXd Y_dot_start = MatrixXd::Zero(3, 1);
  Y_dot_start(2, 0) = dzCoM_wrt_foot;
  MatrixXd Y_dot_end = MatrixXd::Zero(3, 1);

  PiecewisePolynomial<double> pp_part = PiecewisePolynomial<double>::Cubic(
                                          T_waypoint_com,
                                          Y, Y_dot_start,
                                          Y_dot_end);

  // Dynamics of LIPM
  // ddy = 9.81/zCoM_wrt_foot*y, which has an analytical solution
  double omega_td = sqrt(9.81 / zCoM_wrt_foot);
  double constx1 = 0.5 * (xCoM_wrt_foot + dxCoM_wrt_foot / omega_td);
  double constx2 = 0.5 * (xCoM_wrt_foot - dxCoM_wrt_foot / omega_td);
  double consty1 = 0.5 * (yCoM_wrt_foot + dyCoM_wrt_foot / omega_td);
  double consty2 = 0.5 * (yCoM_wrt_foot - dyCoM_wrt_foot / omega_td);
  // std::cout<<"omega_td = "<<omega_td<<"\n";

  // Sum of two exponential + one-segment 3D polynomial
  MatrixXd K = MatrixXd::Zero(3, 2);
  MatrixXd A = MatrixXd::Zero(2, 2);
  MatrixXd alpha = MatrixXd::Zero(2, 1);
  K << constx1, constx2,
  consty1, consty2,
  0,       0      ;
  A << omega_td,  0,
  0,     -omega_td;
  alpha << 1, 1;

  auto exp_traj = ExponentialPlusPiecewisePolynomial<double>(
                    K, A, alpha, pp_part);
  // std::cout<<"Notice that the trajectory is shifted in time (wrt current time) and space (wrt stance foot position)"
  // std::cout<<"First dimension:\n  (from traj, from math.h)\n";
  // for(double d = 0; d<=1; d+=0.1){
  //   double fromMath = K(0,0)*exp(A(0,0)*d) + K(0,1)*exp(A(1,1)*d);
  //   std::cout<<"  t="<<d<<": ("<<(exp_traj.value(d))(0) <<", "<<fromMath<<")\n";
  // }
  // std::cout<<"Second dimension:\n  (from traj, from math.h)\n";
  // for(double d = 0; d<=1; d+=0.1){
  //   double fromMath = K(1,0)*exp(A(0,0)*d) + K(1,1)*exp(A(1,1)*d);
  //   std::cout<<"  t="<<d<<": ("<<(exp_traj.value(d))(1) <<", "<<fromMath<<")\n";
  // }
  // std::cout<<std::endl;

  // Assign traj
  trajs->setCoMTraj(exp_traj);
}



} //namespace systems
} //namespace dairlib


