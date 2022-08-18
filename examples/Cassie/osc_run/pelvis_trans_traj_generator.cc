#include "examples/Cassie/osc_run/pelvis_trans_traj_generator.h"

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"
#include <iostream>

using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
// using drake::common::Polynomial;
using drake::multibody::JacobianWrtVariable;
using drake::trajectories::Trajectory;

namespace dairlib::examples::osc {

PelvisTransTrajGenerator::PelvisTransTrajGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    drake::trajectories::PiecewisePolynomial<double>& traj,
    const std::unordered_map<
        int, std::vector<std::pair<const Eigen::Vector3d,
                                   const drake::multibody::Frame<double>&>>>&
        feet_contact_points,
    bool relative_pelvis)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      pelvis_(plant_.GetBodyByName("pelvis")),
      pelvis_frame_(pelvis_.body_frame()),
      traj_(traj),
      feet_contact_points_(feet_contact_points),
      relative_pelvis_(relative_pelvis) {
  this->set_name("pelvis_trans_traj_generator");
  // Input/Output Setup
  state_port_ = this->DeclareVectorInputPort(
                        "x, u, t", OutputVector<double>(plant_.num_positions(),
                                                        plant_.num_velocities(),
                                                        plant_.num_actuators()))
                    .get_index();
  fsm_port_ =
      this->DeclareVectorInputPort("fsm", BasicVector<double>(1)).get_index();
  clock_port_ = this->DeclareVectorInputPort("t_clock", BasicVector<double>(1))
                    .get_index();
  contact_scheduler_port_ =
      this->DeclareVectorInputPort("t_mode", BasicVector<double>(6))
          .get_index();

  PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->DeclareAbstractOutputPort("pelvis_trans_traj", traj_inst,
                                  &PelvisTransTrajGenerator::CalcTraj);

  DeclarePerStepDiscreteUpdateEvent(
      &PelvisTransTrajGenerator::DiscreteVariableUpdate);
}

EventStatus PelvisTransTrajGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  return EventStatus::Succeeded();
}

PiecewisePolynomial<double> PelvisTransTrajGenerator::GeneratePelvisTraj(
    const VectorXd& x, double t, int fsm_state) const {
  return traj_;
}

PiecewisePolynomial<double> PelvisTransTrajGenerator::GenerateSLIPTraj(
    const VectorXd& x, double t0, double tf, int fsm_state) const {
  // fsm_state should be unused
  if (fsm_state >= 2) {
    // flight phase trajectory should be unused
    return PiecewisePolynomial<double>();
  }

  Vector3d f_g = {0, 0, -9.81};
  Vector3d foot_pos = Vector3d::Zero();
  Vector3d pelvis_pos = Vector3d::Zero();
  Vector3d pelvis_vel = Vector3d::Zero();
  plant_.CalcPointsPositions(*context_,
                             feet_contact_points_.at(fsm_state)[0].second,
                             Vector3d::Zero(), world_, &foot_pos);
  //  plant_.CalcPointsPositions(*context_, pelvis_frame_, Vector3d::Zero(),
  //  world_,
  //                             &pelvis_pos);
  pelvis_pos = plant_.CalcCenterOfMassPositionInWorld(*context_);
  pelvis_vel =
      plant_.EvalBodySpatialVelocityInWorld(*context_, pelvis_).translational();
  Vector3d leg_length = pelvis_pos - foot_pos;

  double compression = leg_length.norm() - rest_length_;
  Vector3d f_leg =
      k_leg_ * compression * leg_length.normalized() + b_leg_ * pelvis_vel;
  VectorXd rddot = f_g + f_leg;

  //  double dt = 1e-3;
  Eigen::Vector2d breaks;
  //  if (t <= 0.3) {
  //    breaks << 0, 0.25, 0.4;
  //  } else {
  //    breaks << 0.4, 0.65, 0.8;
  //  }
  breaks << t0, tf;
  MatrixXd samples(3, 2);
  MatrixXd samples_dot(3, 2);

//  std::cout << "t0:" << t0 << std::endl;
//  std::cout << "tf:" << tf << std::endl;
  //  samples << pelvis_pos, pelvis_pos + 0.5 * rddot * dt * dt;
  //  samples_dot << pelvis_vel, pelvis_vel + rddot * dt;

  //  return PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
  //      breaks, samples, pelvis_vel, pelvis_vel + rddot * dt);
  double y_dist_des = 0;
  if (fsm_state == 0) {
    y_dist_des = -0.15;
  } else if (fsm_state == 1) {
    y_dist_des = 0.15;
  }
  //   samples << 0, 0, 0, y_dist_des, y_dist_des,  y_dist_des, rest_length_,
  //   rest_length_ + 0.05, rest_length_;
  samples << 0, 0, y_dist_des, y_dist_des, rest_length_, rest_length_ + rest_length_offset_;
//  samples_dot << 0, 0, 0, 0, 0.25, 0.0;
  //  return PiecewisePolynomial<double>(Vector3d{0, y_dist_des, rest_length_});
  return PiecewisePolynomial<double>::FirstOrderHold(breaks, samples);
  //  return PiecewisePolynomial<double>::CubicHermite(
  //      breaks, samples, samples_dot);
  //  return PiecewisePolynomial<double>::CubicShapePreserving(breaks, samples);
  //  return PiecewisePolynomial<double>::CubicWithContinuousSecondDerivatives(
  //      breaks, samples, VectorXd::Zero(3), VectorXd::Zero(3));
}

void PelvisTransTrajGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const auto robot_output =
      this->template EvalVectorInput<OutputVector>(context, state_port_);
  // Read in finite state machine
  const auto& fsm_state =
      this->EvalVectorInput(context, fsm_port_)->get_value()(0);
  const auto& clock =
      this->EvalVectorInput(context, clock_port_)->get_value()(0);
  const auto& mode_length =
      this->EvalVectorInput(context, contact_scheduler_port_)->get_value();

  //  std::cout << "fsm_state: " << fsm_state << std::endl;
  //  std::cout << "clock: " << clock << std::endl;
  //  std::cout << "mode_length start: " << mode_length[0] << std::endl;
  //  std::cout << "mode_length end: " << mode_length[1] << std::endl;

  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  //  const drake::VectorX<double>& x = robot_output->GetState();
  if (fsm_state == 0 || fsm_state == 1) {
    if (relative_pelvis_) {
      *casted_traj = GenerateSLIPTraj(robot_output->GetState(), mode_length[0],
                                      mode_length[1], fsm_state);
    } else {
      *casted_traj =
          GeneratePelvisTraj(robot_output->GetState(), clock, fsm_state);
    }
  }
}

}  // namespace dairlib::examples::osc