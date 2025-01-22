#include "joint_pd_controller.h"

#include "lcm/lcm_trajectory.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"
#include "dairlib/lcmt_timestamped_saved_traj.hpp"

namespace dairlib::systems::controllers::id_mpc {

using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::multibody::JointIndex;
using drake::trajectories::PiecewisePolynomial;

JointPDController::JointPDController(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::unordered_map<std::string, double>& kp,
    const std::unordered_map<std::string, double>& kd) : plant_(plant) {

  DRAKE_DEMAND(kp.size() == plant.num_actuators());
  DRAKE_DEMAND(kd.size() == plant.num_actuators());

  // Map the position and velocity errors to the corresponding joints
  Kp_ = MatrixXd::Zero(plant.num_actuators(), plant.num_positions());
  Kd_ = MatrixXd::Zero(plant.num_actuators(), plant.num_velocities());

  auto pos_map = multibody::MakeNameToPositionsMap(plant);
  auto vel_map = multibody::MakeNameToVelocitiesMap(plant);

  for (const JointIndex& idx : plant.GetJointIndices()) {
    if (kp.contains(plant.get_joint(idx).name())) {
      int jq = pos_map.at(plant.get_joint(idx).name());
      int jv = vel_map.at(plant.get_joint(idx).name() + "dot");
      VectorXd selection_matrix = plant.MakeActuatorSelectorMatrix(
          std::vector<JointIndex>({idx}));
      Kp_.col(jq) = kp.at(plant.get_joint(idx).name()) * selection_matrix;
      Kd_.col(jv) = kd.at(plant.get_joint(idx).name()) * selection_matrix;
    }
  }

  input_port_state_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(plant)).get_index();

  input_port_lcm_traj_ = DeclareAbstractInputPort(
      "lcmt_timestamped_saved_traj",
      drake::Value<dairlib::lcmt_timestamped_saved_traj>{}).get_index();

  trajectory_cache_ = this->DeclareCacheEntry(
      "trajectory", &JointPDController::CalcTrajs,
      {input_port_ticket(input_port_lcm_traj_)}).cache_index();

  DeclareVectorOutputPort(
      "u, t",
      TimestampedVector<double>(plant_.num_actuators()),
      &JointPDController::CalcTorques);

  context_ = plant.CreateDefaultContext();
}

void JointPDController::CalcTrajs(
    const drake::systems::Context<double>& context,
    JointPDController::ReferenceTraj* trajs) const {
  const auto& lcm_traj = get_input_port_lcm_traj().Eval<lcmt_timestamped_saved_traj>(context);
  LcmTrajectory trajectories(lcm_traj.saved_traj);

  VectorXd breaks = trajectories.GetTrajectory("q").time_vector;
  MatrixXd q_knots = trajectories.GetTrajectory("q").datapoints;
  MatrixXd v_knots = trajectories.GetTrajectory("v").datapoints;
  MatrixXd qdot_knots = MatrixXd::Zero(q_knots.rows(), q_knots.cols());

  for (int i = 0; i < q_knots.cols(); ++i) {
    plant_.SetPositions(context_.get(), q_knots.col(i));
    plant_.SetVelocities(context_.get(), v_knots.col(i));
    VectorXd qdot = VectorXd::Zero(plant_.num_positions());
    plant_.MapVelocityToQDot(*context_, v_knots.col(i), &qdot);
    qdot_knots.col(i) = qdot;
  }

  trajs->q = PiecewisePolynomial<double>::CubicHermite(breaks, q_knots, qdot_knots);
  trajs->v = PiecewisePolynomial<double>::FirstOrderHold(breaks, v_knots);
  trajs->u = PiecewisePolynomial<double>::ZeroOrderHold(
      trajectories.GetTrajectory("u").time_vector,
      trajectories.GetTrajectory("u").datapoints);
}

void JointPDController::CalcTorques(
    const drake::systems::Context<double>& context,
    TimestampedVector<double> *u) const {
  const auto& state = get_input_port_state().Eval<OutputVector<double>>(context);
  const auto& trajs = get_cache_entry(
      trajectory_cache_).Eval<JointPDController::ReferenceTraj>(context);

  double t = state.get_timestamp();
  VectorXd q = state.GetPositions();
  VectorXd v = state.GetVelocities();

  VectorXd u_fb = Kp_ * (trajs.q.value(t) - q) + Kd_ * (trajs.v.value(t) - v);
  VectorXd u_ff = trajs.u.value(t);

  u->set_timestamp(t);
  u->SetDataVector(u_ff + u_fb);
}


}
