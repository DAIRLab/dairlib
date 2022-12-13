#include "kinematic_trajectory_generator.h"

#include <utility>

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/systems/framework/leaf_system.h"

using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::Trajectory;

namespace dairlib {

KinematicTrajectoryGenerator::KinematicTrajectoryGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    const std::string& body_name,
    const Vector3d& point_on_body) : KinematicTrajectoryGenerator(plant,
                                                                  context,
                                                                  body_name,
                                                                  point_on_body,
                                                                  body_name,
                                                                  point_on_body) {
  is_relative_ = false;
}

KinematicTrajectoryGenerator::KinematicTrajectoryGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    std::string body_one_name,
    Vector3d point_on_body_one,
    const std::string& body_two_name,
    Vector3d point_on_body_two)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      body_one_name_(std::move(body_one_name)),
      point_on_body_one_(std::move(point_on_body_one)),
      body_two_name_(body_two_name),
      point_on_body_two_(std::move(point_on_body_two)) {
  if (is_relative_)
    this->set_name(body_two_name + "_rel");
  else
    this->set_name(body_two_name);
  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort("x, u, t",
                                   OutputVector<double>(plant_.num_positions(),
                                                        plant_.num_velocities(),
                                                        plant_.num_actuators()))
          .get_index();
  fsm_port_ =
      this->DeclareVectorInputPort("fsm", BasicVector<double>(1)).get_index();
  state_trajectory_port_ =
      this->DeclareAbstractInputPort(
              "state_reference",
              drake::Value<PiecewisePolynomial<double>>(PiecewisePolynomial<
                  double>()))
          .get_index();

  PiecewisePolynomial<double> empty_pp_traj(Eigen::VectorXd(0));
  Trajectory<double>& traj_inst = empty_pp_traj;
  this->DeclareAbstractOutputPort(body_two_name_, traj_inst,
                                  &KinematicTrajectoryGenerator::CalcTraj);

}

EventStatus KinematicTrajectoryGenerator::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {

  return EventStatus::Succeeded();
}

void KinematicTrajectoryGenerator::CalcTraj(
    const drake::systems::Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const auto& state_traj =
      this->EvalInputValue<drake::trajectories::PiecewisePolynomial<double>>(
          context, state_trajectory_port_);

  // (TODO) yangwill this assumes a 3d reference target_point
  const std::vector<double>& segment_times = state_traj->get_segment_times();
  std::vector<MatrixXd> samples_target_point;
  std::vector<MatrixXd> samples_dot_target_point;
  std::vector<MatrixXd> samples_reference_point;
  std::vector<MatrixXd> samples_dot_reference_point;
  MatrixXd J_target_point(3, plant_.num_velocities());
  MatrixXd J_reference_point(3, plant_.num_velocities());
  Vector3d target_point;
  Vector3d reference_point;

  for (int i = 0; i<segment_times.size(); ++i) {
    const VectorXd& x = state_traj->value(segment_times[i]);
    plant_.SetPositionsAndVelocities(context_, x);
    plant_.CalcPointsPositions(*context_,
                               plant_.GetBodyByName(body_two_name_).body_frame(),
                               point_on_body_two_, world_, &target_point);
    plant_.CalcJacobianTranslationalVelocity(*context_,
                                             JacobianWrtVariable::kV,
                                             plant_.GetBodyByName(body_two_name_).body_frame(),
                                             point_on_body_two_,
                                             world_,
                                             world_,
                                             &J_target_point);
    plant_.CalcPointsPositions(*context_,
                               plant_.GetBodyByName(body_one_name_).body_frame(),
                               point_on_body_one_, world_, &target_point);
    plant_.CalcJacobianTranslationalVelocity(*context_,
                                             JacobianWrtVariable::kV,
                                             plant_.GetBodyByName(body_one_name_).body_frame(),
                                             point_on_body_one_,
                                             world_,
                                             world_,
                                             &J_reference_point);
    samples_target_point.emplace_back(target_point);
    samples_dot_target_point.emplace_back(
        J_target_point * x.tail(plant_.num_velocities()));
    samples_reference_point.emplace_back(reference_point);
    samples_dot_reference_point.emplace_back(
        J_reference_point * x.tail(plant_.num_velocities()));
  }

  // (TODO):yangwill use the proper spline construction according to the integration choice of the MPC
  auto* casted_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  auto target_trajectory = PiecewisePolynomial<double>::CubicHermite(segment_times,
                                                                        samples_target_point,
                                                                        samples_dot_target_point);
  auto reference_point_trajectory = PiecewisePolynomial<double>::CubicHermite(segment_times,
                                                                        samples_reference_point,
                                                                        samples_dot_reference_point);
//  auto target_trajectory =
//      PiecewisePolynomial<double>::FirstOrderHold(segment_times,
//                                                  samples_target_point);
//  auto reference_point_trajectory =
//      PiecewisePolynomial<double>::FirstOrderHold(segment_times,
//                                                  samples_reference_point);
  if (is_relative_) {
    *casted_traj = reference_point_trajectory - target_trajectory;
  } else {
    *casted_traj = target_trajectory;
  }

}

}  // namespace dairlib