#include "examples/Cassie/osc/high_level_command.h"

#include <math.h>

#include <string>

#include "dairlib/lcmt_cassie_out.hpp"
#include "multibody/multibody_utils.h"

#include "drake/math/quaternion.h"
#include "drake/math/saturate.h"

using std::cout;
using std::endl;
using std::string;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using Eigen::Quaterniond;

using dairlib::systems::OutputVector;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::systems::LeafSystem;

using drake::multibody::JacobianWrtVariable;
using drake::trajectories::PiecewisePolynomial;

using drake::MatrixX;

namespace dairlib {
namespace cassie {
namespace osc {

HighLevelCommand::HighLevelCommand(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context, double vel_scale_rot,
    double vel_scale_trans_sagital, double vel_scale_trans_lateral)
    : HighLevelCommand(plant, context) {
  cassie_out_port_ =
      this->DeclareAbstractInputPort("lcmt_cassie_output",
                                     drake::Value<dairlib::lcmt_cassie_out>{})
          .get_index();
  //  use_radio_command_ = true;
  high_level_mode_ = radio;

  vel_scale_rot_ = vel_scale_rot;
  vel_scale_trans_sagital_ = vel_scale_trans_sagital;
  vel_scale_trans_lateral_ = vel_scale_trans_lateral;
}

HighLevelCommand::HighLevelCommand(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context, double kp_yaw, double kd_yaw,
    double vel_max_yaw, double kp_pos_sagital, double kd_pos_sagital,
    double vel_max_sagital, double kp_pos_lateral, double kd_pos_lateral,
    double vel_max_lateral, double target_pos_offset,
    const Vector2d& global_target_position,
    const Vector2d& params_of_no_turning)
    : HighLevelCommand(plant, context) {
  //  use_radio_command_ = false;
  high_level_mode_ = desired_xy_position;

  kp_yaw_ = kp_yaw;
  kd_yaw_ = kd_yaw;
  vel_max_yaw_ = vel_max_yaw;
  kp_pos_sagital_ = kp_pos_sagital;
  kd_pos_sagital_ = kd_pos_sagital;
  vel_max_sagital_ = vel_max_sagital;
  target_pos_offset_ = target_pos_offset;
  kp_pos_lateral_ = kp_pos_lateral;
  kd_pos_lateral_ = kd_pos_lateral;
  vel_max_lateral_ = vel_max_lateral;

  global_target_position_ = global_target_position;
  params_of_no_turning_ = params_of_no_turning;
}

HighLevelCommand::HighLevelCommand(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      pelvis_(plant_.GetBodyByName("pelvis")) {
  state_port_ = this->DeclareVectorInputPort(
                        "x, u, t", OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
                    .get_index();

  yaw_port_ =
      this->DeclareVectorOutputPort("pelvis_yaw", BasicVector<double>(1),
                                    &HighLevelCommand::CopyHeadingAngle)
          .get_index();
  xy_port_ =
      this->DeclareVectorOutputPort("pelvis_xy", BasicVector<double>(2),
                                    &HighLevelCommand::CopyDesiredHorizontalVel)
          .get_index();
  // Declare update event
  DeclarePerStepDiscreteUpdateEvent(&HighLevelCommand::DiscreteVariableUpdate);

  // Discrete state which stores the desired yaw velocity
  des_vel_idx_ = DeclareDiscreteState(VectorXd::Zero(3));
}

void HighLevelCommand::SetOpenLoopVelCommandTraj() {
  high_level_mode_ = open_loop_vel_command_traj;

  std::vector<double> breaks = {0};
  std::vector<MatrixXd> knots = {(MatrixX<double>(3, 1) << 0, 0, 0).finished()};
  breaks.push_back(breaks.back() + 2);
  knots.push_back((MatrixX<double>(3, 1) << 0, 0, 0).finished());
  breaks.push_back(breaks.back() + 5);
  knots.push_back((MatrixX<double>(3, 1) << 0, 2, 0).finished());
  breaks.push_back(breaks.back() + 2);
  knots.push_back((MatrixX<double>(3, 1) << 0.7, 2, 0).finished());
  breaks.push_back(breaks.back() + 1.5);
  knots.push_back((MatrixX<double>(3, 1) << 0.7, 2, 0).finished());
  breaks.push_back(breaks.back() + 2);
  knots.push_back((MatrixX<double>(3, 1) << 0, 2, 0).finished());
  breaks.push_back(breaks.back() + 2);
  knots.push_back((MatrixX<double>(3, 1) << 0, 2, 0).finished());
  breaks.push_back(breaks.back() + 1);
  knots.push_back((MatrixX<double>(3, 1) << 0, 0, 0).finished());
  breaks.push_back(breaks.back() + 10000000);
  knots.push_back((MatrixX<double>(3, 1) << 0, 0, 0).finished());

  // Construct the PiecewisePolynomial.
  desired_vel_command_traj_ =
      PiecewisePolynomial<double>::FirstOrderHold(breaks, knots);
};

void HighLevelCommand::SetDesiredXYTraj(
    const multibody::ViewFrame<double>* view_frame) {
  high_level_mode_ = desired_xy_traj;
  view_frame_ = view_frame;

  // Set x y set points
  /*std::vector<double> breaks = {0};
  std::vector<std::vector<double>> knots_vec = {{0, 0}};
  breaks.push_back(breaks.back() + 2);
  knots_vec.push_back({0, 0});
  breaks.push_back(breaks.back() + 2);
  knots_vec.push_back({2, 0});
  breaks.push_back(breaks.back() + 2);
  knots_vec.push_back({4, 2});
  breaks.push_back(breaks.back() + 2);
  knots_vec.push_back({4, 4});
  breaks.push_back(breaks.back() + 2);
  knots_vec.push_back({2, 6});
  breaks.push_back(breaks.back() + 2);
  knots_vec.push_back({0, 6});
  breaks.push_back(breaks.back() + 2);
  knots_vec.push_back({0, 6});
  breaks.back() = 10000000;*/

  // Traj: turn -> long strech -> turn
  /*std::vector<double> breaks = {0.000,  1.000,  1.437,  1.875,  2.312,  2.750,
                                3.187,  3.625,  4.062,  4.500,  4.937,  5.374,
                                10.374, 10.812, 11.249, 11.687, 12.124, 12.562,
                                12.999, 13.437, 13.874, 14.311, 14.749};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000}, {0.000, 0.000}, {0.347, 0.030}, {0.684, 0.121},
      {1.000, 0.268}, {1.286, 0.468}, {1.532, 0.714}, {1.732, 1.000},
      {1.879, 1.316}, {1.970, 1.653}, {2.000, 2.000}, {2.000, 4.500},
      {2.000, 7.000}, {1.970, 7.347}, {1.879, 7.684}, {1.732, 8.000},
      {1.532, 8.286}, {1.286, 8.532}, {1.000, 8.732}, {0.684, 8.879},
      {0.347, 8.970}, {0.000, 9.000}, {0.000, 9.000}};*/

  // Traj:  long strech -> 180 turn -> long strech
  //  0.000, dt=2.000: {0.000, 0.000},
  //  2.000, dt=3.500: {0.000, 0.000},
  //  5.500, dt=2.707: {2.500, 0.000},
  //  8.207, dt=0.414: {5.000, 0.000},
  //  8.621, dt=0.414: {5.329, 0.027},
  //  9.036, dt=0.414: {5.649, 0.108},
  //  9.450, dt=0.414: {5.952, 0.241},
  //  9.864, dt=0.414: {6.228, 0.422},
  //  10.279, dt=0.414: {6.471, 0.645},
  //  10.693, dt=0.414: {6.674, 0.906},
  //  11.107, dt=0.414: {6.832, 1.197},
  //  11.522, dt=0.414: {6.939, 1.509},
  //  11.936, dt=0.414: {6.993, 1.835},
  //  12.350, dt=0.414: {6.993, 2.165},
  //  12.765, dt=0.414: {6.939, 2.491},
  //  13.179, dt=0.414: {6.832, 2.803},
  //  13.593, dt=0.414: {6.674, 3.094},
  //  14.008, dt=0.414: {6.471, 3.355},
  //  14.422, dt=0.414: {6.228, 3.578},
  //  14.836, dt=0.414: {5.952, 3.759},
  //  15.250, dt=0.414: {5.649, 3.892},
  //  15.665, dt=0.414: {5.329, 3.973},
  //  16.079, dt=2.707: {5.000, 4.000},
  //  18.786, dt=3.500: {2.500, 4.000},
  //  22.286, dt=0.000: {0.000, 4.000}
  /*std::vector<double> breaks = {
      0.000,  2.000,  5.500,  8.207,  8.621,  9.036,  9.450,  9.864,  10.279,
      10.693, 11.107, 11.522, 11.936, 12.350, 12.765, 13.179, 13.593, 14.008,
      14.422, 14.836, 15.250, 15.665, 16.079, 18.786, 22.286};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000}, {0.000, 0.000}, {2.500, 0.000}, {5.000, 0.000},
      {5.329, 0.027}, {5.649, 0.108}, {5.952, 0.241}, {6.228, 0.422},
      {6.471, 0.645}, {6.674, 0.906}, {6.832, 1.197}, {6.939, 1.509},
      {6.993, 1.835}, {6.993, 2.165}, {6.939, 2.491}, {6.832, 2.803},
      {6.674, 3.094}, {6.471, 3.355}, {6.228, 3.578}, {5.952, 3.759},
      {5.649, 3.892}, {5.329, 3.973}, {5.000, 4.000}, {2.500, 4.000},
      {0.000, 4.000}};*/

  // Traj:  long strech -> 180 turn -> long strech
  // LIP cannot keep up the speed of this
  //    0.000, dt=2.000: {0.000, 0.000},
  //    2.000, dt=2.500: {0.000, 0.000},
  //    4.500, dt=2.500: {2.500, 0.000},
  //    7.000, dt=0.330: {5.000, 0.000},
  //    7.330, dt=0.330: {5.329, 0.027},
  //    7.661, dt=0.330: {5.649, 0.108},
  //    7.991, dt=0.330: {5.952, 0.241},
  //    8.321, dt=0.330: {6.228, 0.422},
  //    8.652, dt=0.330: {6.471, 0.645},
  //    8.982, dt=0.330: {6.674, 0.906},
  //    9.312, dt=0.330: {6.832, 1.197},
  //    9.643, dt=0.330: {6.939, 1.509},
  //    9.973, dt=0.330: {6.993, 1.835},
  //    10.303, dt=0.330: {6.993, 2.165},
  //    10.633, dt=0.330: {6.939, 2.491},
  //    10.964, dt=0.330: {6.832, 2.803},
  //    11.294, dt=0.330: {6.674, 3.094},
  //    11.624, dt=0.330: {6.471, 3.355},
  //    11.955, dt=0.330: {6.228, 3.578},
  //    12.285, dt=0.330: {5.952, 3.759},
  //    12.615, dt=0.330: {5.649, 3.892},
  //    12.946, dt=0.330: {5.329, 3.973},
  //    13.276, dt=2.500: {5.000, 4.000},
  //    15.776, dt=2.500: {2.500, 4.000},
  //    18.276, dt=0.000: {0.000, 4.000}
  std::vector<double> breaks = {
      0.000,  2.000,  4.500,  7.000,  7.330,  7.661,  7.991,  8.321,  8.652,
      8.982,  9.312,  9.643,  9.973,  10.303, 10.633, 10.964, 11.294, 11.624,
      11.955, 12.285, 12.615, 12.946, 13.276, 15.776, 18.276};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000}, {0.000, 0.000}, {2.500, 0.000}, {5.000, 0.000},
      {5.329, 0.027}, {5.649, 0.108}, {5.952, 0.241}, {6.228, 0.422},
      {6.471, 0.645}, {6.674, 0.906}, {6.832, 1.197}, {6.939, 1.509},
      {6.993, 1.835}, {6.993, 2.165}, {6.939, 2.491}, {6.832, 2.803},
      {6.674, 3.094}, {6.471, 3.355}, {6.228, 3.578}, {5.952, 3.759},
      {5.649, 3.892}, {5.329, 3.973}, {5.000, 4.000}, {2.500, 4.000},
      {0.000, 4.000}};

  /*std::vector<double> breaks = {
      0.000,  2.000,  7.000,  12.000, 12.661, 13.321, 13.982, 14.643, 15.303,
      15.964, 16.624, 17.285, 17.946, 18.606, 19.267, 19.928, 20.588, 21.249,
      21.910, 22.570, 23.231, 23.891, 24.552, 29.552, 34.552};
  std::vector<std::vector<double>> knots_vec = {
      {0.000, 0.000}, {0.000, 0.000}, {2.500, 0.000}, {5.000, 0.000},
      {5.329, 0.027}, {5.649, 0.108}, {5.952, 0.241}, {6.228, 0.422},
      {6.471, 0.645}, {6.674, 0.906}, {6.832, 1.197}, {6.939, 1.509},
      {6.993, 1.835}, {6.993, 2.165}, {6.939, 2.491}, {6.832, 2.803},
      {6.674, 3.094}, {6.471, 3.355}, {6.228, 3.578}, {5.952, 3.759},
      {5.649, 3.892}, {5.329, 3.973}, {5.000, 4.000}, {2.500, 4.000},
      {0.000, 4.000}};*/

  // Convert vector to MatrixXd
  std::vector<MatrixXd> knots(knots_vec.size(), MatrixX<double>(2, 1));
  for (int i = 0; i < knots_vec.size(); i++) {
    knots.at(i) << knots_vec.at(i)[0], knots_vec.at(i)[1];
  }

  // Construct the cubic splines for x y trajectory.
  desired_xy_traj_ = PiecewisePolynomial<double>::FirstOrderHold(breaks, knots);
  //  desired_xy_traj_ =
  //      PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
  //          breaks, knots, MatrixX<double>::Zero(2, 1),
  //          MatrixX<double>::Zero(2, 1));

  /*for (int i = 0; i < breaks.size(); i++) {
    cout << breaks.at(i) << ", " << knots.at(i).transpose() << endl;
  }
  cout << "desired_xy_traj_ = \n";
  for (int i = 0; i < 60; i++) {
    cout << desired_xy_traj_.value(i * 0.1).transpose() << endl;
  }*/
};

EventStatus HighLevelCommand::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  if (high_level_mode_ == radio) {
    const auto& cassie_out = this->EvalInputValue<dairlib::lcmt_cassie_out>(
        context, cassie_out_port_);
    // TODO(yangwill) make sure there is a message available
    // des_vel indices: 0: yaw_vel (right joystick left/right)
    //                  1: saggital_vel (left joystick up/down)
    //                  2: lateral_vel (left joystick left/right)
    Vector3d des_vel;
    des_vel << vel_scale_rot_ * cassie_out->pelvis.radio.channel[3],
        vel_scale_trans_sagital_ * cassie_out->pelvis.radio.channel[0],
        vel_scale_trans_lateral_ * cassie_out->pelvis.radio.channel[1];
    des_vel(1) += vel_command_offset_x_;  // hack: help rom_iter=300 walk in
                                          // place at start
    discrete_state->get_mutable_vector(des_vel_idx_).set_value(des_vel);
  } else if (high_level_mode_ == desired_xy_position) {
    discrete_state->get_mutable_vector(des_vel_idx_)
        .set_value(CalcCommandFromTargetPosition(context));
  } else if (high_level_mode_ == open_loop_vel_command_traj) {
    discrete_state->get_mutable_vector(des_vel_idx_)
        .set_value(desired_vel_command_traj_.value(context.get_time()));
  } else if (high_level_mode_ == desired_xy_traj) {
    discrete_state->get_mutable_vector(des_vel_idx_)
        .set_value(CalcCommandFromDesiredXYTraj(context));
  }

  return EventStatus::Succeeded();
}

VectorXd HighLevelCommand::CalcCommandFromTargetPosition(
    const Context<double>& context) const {
  // Read in current state
  const OutputVector<double>* robotOutput =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  VectorXd q = robotOutput->GetPositions();
  VectorXd v = robotOutput->GetVelocities();

  plant_.SetPositions(context_, q);

  // Get center of mass position and velocity
  Vector3d com_pos = plant_.CalcCenterOfMassPositionInWorld(*context_);
  MatrixXd J(3, plant_.num_velocities());
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      *context_, JacobianWrtVariable::kV, world_, world_, &J);
  Vector3d com_vel = J * v;

  //////////// Get desired yaw velocity ////////////
  // Get approximated heading angle of pelvis
  Vector3d pelvis_heading_vec =
      plant_.EvalBodyPoseInWorld(*context_, pelvis_).rotation().col(0);
  double approx_pelvis_yaw =
      atan2(pelvis_heading_vec(1), pelvis_heading_vec(0));

  // Get desired heading angle of pelvis
  Vector2d global_com_pos_to_target_pos =
      global_target_position_ - com_pos.segment(0, 2);
  double desired_yaw =
      atan2(global_com_pos_to_target_pos(1), global_com_pos_to_target_pos(0));

  // Get current yaw velocity
  double yaw_vel = v(2);

  // yaw error
  double heading_error =
      std::remainder(desired_yaw - approx_pelvis_yaw, 2 * M_PI);

  // PD position control
  double des_yaw_vel = kp_yaw_ * heading_error + kd_yaw_ * (-yaw_vel);
  des_yaw_vel = drake::math::saturate(des_yaw_vel, -vel_max_yaw_, vel_max_yaw_);
  /*cout << "desired_yaw= " << desired_yaw << endl;
  cout << "approx_pelvis_yaw= " << approx_pelvis_yaw << endl;
  cout << "heading_error= " << heading_error << endl;
  cout << "des_yaw_vel= " << des_yaw_vel << endl;
  cout << "\n";*/

  // Convex combination of 0 and desired yaw velocity
  double weight = 1 / (1 + exp(-params_of_no_turning_(0) *
                               (global_com_pos_to_target_pos.norm() -
                                params_of_no_turning_(1))));
  double desired_filtered_yaw_vel = (1 - weight) * 0 + weight * des_yaw_vel;

  //////////// Get desired horizontal vel ////////////
  // Calculate the current-desired yaw angle difference
  // filtered_heading_error is the convex combination of 0 and heading_error
  double filtered_heading_error = weight * heading_error;

  // Apply walking speed control only when the robot is facing the target
  // position.
  double des_sagital_vel = 0;
  double des_lateral_vel = 0;
  if (abs(filtered_heading_error) < M_PI / 2) {
    // Extract quaternion from floating base position
    Quaterniond Quat(q(0), q(1), q(2), q(3));
    Quaterniond Quat_conj = Quat.conjugate();
    Vector4d quat(q.head(4));
    Vector4d quad_conj(Quat_conj.w(), Quat_conj.x(), Quat_conj.y(),
                       Quat_conj.z());

    // Calculate local target position and com velocity
    Vector3d global_com_pos_to_target_pos_3d;
    global_com_pos_to_target_pos_3d << global_com_pos_to_target_pos, 0;
    Vector3d local_com_pos_to_target_pos =
        drake::math::quatRotateVec(quad_conj, global_com_pos_to_target_pos_3d);
    Vector3d local_com_vel = drake::math::quatRotateVec(quad_conj, com_vel);

    // Sagital plane position PD control
    double com_vel_sagital = local_com_vel(0);
    des_sagital_vel = kp_pos_sagital_ * (local_com_pos_to_target_pos(0) +
                                         target_pos_offset_) +
                      kd_pos_sagital_ * (-com_vel_sagital);
    des_sagital_vel = drake::math::saturate(des_sagital_vel, -vel_max_sagital_,
                                            vel_max_sagital_);

    // Frontal plane position PD control.  TODO(yminchen): tune this
    double com_vel_lateral = local_com_vel(1);
    des_lateral_vel = kp_pos_lateral_ * (local_com_pos_to_target_pos(1)) +
                      kd_pos_lateral_ * (-com_vel_lateral);
    des_lateral_vel = drake::math::saturate(des_lateral_vel, -vel_max_lateral_,
                                            vel_max_lateral_);
  }
  Vector3d des_vel;
  des_vel << desired_filtered_yaw_vel, des_sagital_vel, des_lateral_vel;

  return des_vel;
}

VectorXd HighLevelCommand::CalcCommandFromDesiredXYTraj(
    const Context<double>& context) const {
  VectorXd q = dynamic_cast<const OutputVector<double>*>(
                   this->EvalVectorInput(context, state_port_))
                   ->GetPositions();
  VectorXd v = dynamic_cast<const OutputVector<double>*>(
                   this->EvalVectorInput(context, state_port_))
                   ->GetVelocities();
  Eigen::Matrix2d view_frame_rot_T_ =
      view_frame_->CalcWorldToFrameRotation(plant_, *context_)
          .topLeftCorner<2, 2>();

  // Advance the time for desired traj if the tracking error is not too big
  double dt_sim = (prev_t_ == 0) ? 0 : context.get_time() - prev_t_;
  if (!tracking_error_too_big_) {
    t_traj_ += dt_sim;
  }

  bool reach_the_end_of_traj = desired_xy_traj_.end_time() <= t_traj_;

  // Compute desired x y vel
  Vector2d des_xy = desired_xy_traj_.value(t_traj_);
  Vector2d cur_xy = q.segment<2>(4);
  Vector2d local_delta_xy = view_frame_rot_T_ * (des_xy - cur_xy);

  Vector2d des_xy_dot =
      desired_xy_traj_.has_derivative()
          ? desired_xy_traj_.EvalDerivative(t_traj_, 1)
          : desired_xy_traj_.MakeDerivative(1)->value(t_traj_);
  Vector2d local_des_xy_dot = view_frame_rot_T_ * des_xy_dot;

  if (reach_the_end_of_traj) local_des_xy_dot.setZero();
  Vector2d command_xy_vel = 2 * local_delta_xy + local_des_xy_dot;

  // Compute yaw traj by taking derivaties of x y traj
  // yaw = arctan(dy,dx)
  Quaterniond cur_quat(q(0), q(1), q(2), q(3));
  Quaterniond des_quat =
      des_xy_dot.norm() < 0.01
          ? cur_quat
          : Quaterniond::FromTwoVectors(
                Vector3d(1, 0, 0), Vector3d(des_xy_dot(0), des_xy_dot(1), 0));
  Eigen::AngleAxis<double> angle_axis_err(des_quat * cur_quat.inverse());
  double yaw_err = (angle_axis_err.angle() * angle_axis_err.axis())(2);

  double delta_t =
      0.5;  // dt cannot be small, because our desired xy traj is not smooth
  Vector2d des_xy_dot_dt_later =
      desired_xy_traj_.has_derivative()
          ? desired_xy_traj_.EvalDerivative(t_traj_ + delta_t, 1)
          : desired_xy_traj_.MakeDerivative(1)->value(t_traj_ + delta_t);
  Quaterniond des_quat_dt_later =
      des_xy_dot_dt_later.norm() < 0.01
          ? cur_quat
          : Quaterniond::FromTwoVectors(
                Vector3d(1, 0, 0),
                Vector3d(des_xy_dot_dt_later(0), des_xy_dot_dt_later(1), 0));
  Eigen::AngleAxis<double> angle_axis_delta(des_quat_dt_later *
                                            des_quat.inverse());
  double delta_yaw = (angle_axis_delta.angle() * angle_axis_delta.axis())(2);
  double des_yaw_dot = delta_yaw / delta_t;

  if (reach_the_end_of_traj) des_yaw_dot = 0;
  double command_yaw_vel = 10 * yaw_err + des_yaw_dot;

  // Assign x, y and yaw vel
  Vector3d des_vel;
  // des_vel << command_yaw_vel, command_xy_vel(0), 0;

  // Instead of tracking des_vel_y here, we change the orientation to get closer
  // to the trajectory "center line"
  if (std::abs(command_xy_vel(1)) > 1) {
    command_yaw_vel += 0.4 * command_xy_vel(1);
  }
  des_vel << command_yaw_vel, command_xy_vel(0), 0;

  // Add low pass filter to the command
  if (prev_t_ == 0) {
    filtered_vel_command_.setZero();
    //    filtered_vel_command_ = des_vel;
  } else {
    //cutoff_freq_ = (t_traj_ < 4) ? 0.1 : 1;
    if (reach_the_end_of_traj) {
      cutoff_freq_ = 0.1;
      //      filtered_vel_command_.setZero();
    }

    double alpha = 2 * M_PI * dt_sim * cutoff_freq_ /
                   (2 * M_PI * dt_sim * cutoff_freq_ + 1);
    filtered_vel_command_ =
        alpha * des_vel + (1 - alpha) * filtered_vel_command_;
  }

  // Update the flag `tracking_error_too_big_`
  tracking_error_too_big_ =
      ((local_delta_xy.norm() > 0.3) || (yaw_err > M_PI / 4));

  prev_t_ = context.get_time();

  /*cout << "t sime = " << context.get_time() << endl;
  cout << "t traj = " << t_traj_ << endl;
  cout << "des_xy = " << des_xy.transpose() << endl;
  cout << "local_delta_xy = " << local_delta_xy.transpose() << endl;
  cout << "local_des_xy_dot = " << local_des_xy_dot.transpose() << endl;

  cout << "des_xy_dot = " << des_xy_dot.transpose() << endl;
  cout << "des_xy_dot_dt_later = " << des_xy_dot_dt_later.transpose() << endl;
  cout << "---\n";
  cout << "cur_quat = " << cur_quat.w() << " " << cur_quat.vec().transpose()
       << endl;
  cout << "des_quat = " << des_quat.w() << " " << des_quat.vec().transpose()
       << endl;
  cout << "des_quat_dt_later = " << des_quat_dt_later.w() << " "
       << des_quat_dt_later.vec().transpose() << endl;
  cout << "angle_axis_err = " << angle_axis_err.angle() << ", "
       << angle_axis_err.axis().transpose() << endl;
  cout << "angle_axis_delta = " << angle_axis_delta.angle() << ", "
       << angle_axis_delta.axis().transpose() << endl;
  cout << "yaw_err = " << yaw_err << endl;
  cout << "des_yaw_dot = " << des_yaw_dot << endl;

  cout << "des_vel = " << des_vel.transpose() << endl;
  cout << "filtered_vel_command_ = " << filtered_vel_command_.transpose()
       << endl;
  cout << "=============\n";*/

  //  return des_vel;
  return filtered_vel_command_;
  //  return Vector3d(filtered_vel_command_(0), des_vel(1), 0);
}

void HighLevelCommand::CopyHeadingAngle(const Context<double>& context,
                                        BasicVector<double>* output) const {
  double desired_heading_pos =
      context.get_discrete_state(des_vel_idx_).get_value()(0);
  // Assign
  output->get_mutable_value() << desired_heading_pos;
}

void HighLevelCommand::CopyDesiredHorizontalVel(
    const Context<double>& context, BasicVector<double>* output) const {
  auto delta_CP_3D_global =
      context.get_discrete_state(des_vel_idx_).get_value().tail(2);

  // Assign
  output->get_mutable_value() = delta_CP_3D_global;
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
