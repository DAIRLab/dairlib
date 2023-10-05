#include "examples/Cassie/osc_run/pelvis_trans_traj_generator.h"

#include <iostream>

#include "examples/Cassie/contact_scheduler/contact_scheduler.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib::examples::osc {

PelvisTransTrajGenerator::PelvisTransTrajGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    const std::unordered_map<
        int, std::vector<std::pair<const Eigen::Vector3d,
                                   const drake::multibody::Frame<double>&>>>&
        feet_contact_points)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      pelvis_(plant_.GetBodyByName("pelvis")),
      feet_contact_points_(feet_contact_points){
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

  PiecewisePolynomial<double> empty_traj(VectorXd(0));
  Trajectory<double>& traj_inst = empty_traj;
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

PiecewisePolynomial<double> PelvisTransTrajGenerator::GenerateSLIPTraj(
    const VectorXd& x, double t0, double tf, int fsm_state) const {
  DRAKE_DEMAND(fsm_state < 2);

  Vector3d f_g =
      drake::multibody::UniformGravityFieldElement<double>().gravity_vector();
  Vector3d foot_pos = Vector3d::Zero();
  Vector3d pelvis_pos = Vector3d::Zero();
  Vector3d pelvis_vel = Vector3d::Zero();
  plant_.CalcPointsPositions(*context_,
                             feet_contact_points_.at(fsm_state)[0].second,
                             Vector3d::Zero(), world_, &foot_pos);
  pelvis_pos = plant_.CalcCenterOfMassPositionInWorld(*context_);
  pelvis_vel =
      plant_.EvalBodySpatialVelocityInWorld(*context_, pelvis_).translational();
  Vector3d leg_length = pelvis_pos - foot_pos;

  double compression = leg_length.norm() - rest_length_;
  Vector3d f_leg =
      k_leg_ * compression * leg_length.normalized() + b_leg_ * pelvis_vel;
  // ignoring f_leg, spring forces handled by OSC gains
  Vector3d rddot = f_g;

  double dt = 1e-2;
  Eigen::Vector2d breaks;
  breaks << t0, tf;
  MatrixXd samples(3, 2);
  MatrixXd samples_dot(3, 2);

  double y_dist_des = 0;
  if (fsm_state == 0) {
    y_dist_des = -0.1;
  } else if (fsm_state == 1) {
    y_dist_des = 0.1;
  }

  samples << 0, 0 + 0.5 * rddot[0] * dt * dt, y_dist_des,
      y_dist_des + 0.5 * rddot[1] * dt * dt, rest_length_,
      rest_length_ + 0.5 * rddot[2] * dt * dt;
  samples_dot << 0, 0 + rddot[0] * dt, 0, 0 + rddot[1] * dt, 0,
      0 + rddot[2] * dt;

  return PiecewisePolynomial<double>::CubicHermite(breaks, samples,
                                                   samples_dot);
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

  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  if (fsm_state == kLeftStance || fsm_state == kRightStance) {
    *casted_traj = GenerateSLIPTraj(robot_output->GetState(), mode_length[0],
                                    mode_length[1], fsm_state);
  }
}

}  // namespace dairlib::examples::osc