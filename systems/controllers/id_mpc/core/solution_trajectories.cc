#include "solution_trajectories.h"


namespace dairlib::systems::controllers::id_mpc {

using Eigen::VectorXd;
using Eigen::MatrixXd;


using drake::trajectories::PiecewisePolynomial;

SolutionTraj SolutionTraj::FromLcmTrajectory(
    const LcmTrajectory& traj,
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context) {

  DRAKE_DEMAND(context != nullptr);

  VectorXd breaks = traj.GetTrajectory("q").time_vector;
  MatrixXd q_knots = traj.GetTrajectory("q").datapoints;
  MatrixXd v_knots = traj.GetTrajectory("v").datapoints;
  MatrixXd qdot_knots = MatrixXd::Zero(q_knots.rows(), q_knots.cols());

  for (int i = 0; i < q_knots.cols(); ++i) {
    plant.SetPositions(context, q_knots.col(i));
    plant.SetVelocities(context, v_knots.col(i));
    VectorXd qdot = VectorXd::Zero(plant.num_positions());
    plant.MapVelocityToQDot(*context, v_knots.col(i), &qdot);
    qdot_knots.col(i) = qdot;
  }

  SolutionTraj trajs;
  trajs.q = PiecewisePolynomial<double>::CubicHermite(
      breaks, q_knots, qdot_knots);
  trajs.v = PiecewisePolynomial<double>::FirstOrderHold(breaks, v_knots);
  trajs.u = PiecewisePolynomial<double>::ZeroOrderHold(
      traj.GetTrajectory("u").time_vector,
      traj.GetTrajectory("u").datapoints);

  trajs.lambda = PiecewisePolynomial<double>::ZeroOrderHold(
      traj.GetTrajectory("lambda").time_vector,
      traj.GetTrajectory("lambda").datapoints);
  return trajs;
}

}