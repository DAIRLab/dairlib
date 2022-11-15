#include <iostream>
#include "reference_generation_utils.h"

#include "multibody/multibody_utils.h"

drake::trajectories::PiecewisePolynomial<double> GenerateComTrajectory(
    const Eigen::Vector3d& current_com,
    const std::vector<Eigen::Vector3d>& vel_ewrt_w,
    const std::vector<double>& time_points) {
  DRAKE_DEMAND(vel_ewrt_w.size() == (time_points.size() - 1));
  auto n_points = time_points.size();

  std::vector<drake::MatrixX<double>> samples(n_points);
  samples[0] = current_com;
  for (int i = 1; i < n_points; i++) {
    samples[i] = samples[i - 1] +
                 (time_points[i] - time_points[i - 1]) * vel_ewrt_w[i - 1];
  }
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      time_points, samples);
}

drake::trajectories::PiecewisePolynomial<double> GenerateMomentumTrajectory(const std::vector<Eigen::Vector3d> &vel_ewrt_w,
                                                                            const std::vector<double> &time_points,
                                                                            double m) {
  DRAKE_DEMAND(vel_ewrt_w.size() == (time_points.size() - 1));
  auto n_points = time_points.size();
  std::vector<drake::MatrixX<double>> samples(n_points);
  for (int i = 0; i < n_points; i++) {
    samples[i] = drake::Vector6d();
    samples[i] << drake::Vector3<double>::Zero(3), vel_ewrt_w[i] * m;
  }
  return drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(
      time_points, samples);
}


drake::trajectories::PiecewisePolynomial<double>
GenerateGeneralizedPosTrajectory(
    const Eigen::VectorXd& nominal_stand, const Eigen::Vector3d& p_ScmBase_W,
    const drake::trajectories::PiecewisePolynomial<double>& com_traj,
    int base_pos_start) {
  auto n_points = com_traj.get_segment_times().size();
  std::vector<drake::MatrixX<double>> samples(n_points);
  for (int i = 0; i < n_points; i++) {
    samples[i] = nominal_stand;
    samples[i].block<3, 1>(base_pos_start, 0, 3, 1) =
        com_traj.value(com_traj.get_segment_times()[i]) + p_ScmBase_W;
  }
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      com_traj.get_segment_times(), samples);
}

drake::trajectories::PiecewisePolynomial<double>
GenerateGeneralizedVelTrajectory(
    const drake::trajectories::PiecewisePolynomial<double>& com_traj, int n_v,
    int base_vel_start) {
  auto n_points = com_traj.get_segment_times().size();
  std::vector<drake::MatrixX<double>> samples(n_points);
  for (int i = 0; i < n_points; i++) {
    samples[i] = drake::VectorX<double>::Zero(n_v);
    samples[i].block<3, 1>(base_vel_start, 0, 3, 1) =
        com_traj.derivative().value(com_traj.get_segment_times()[i]);
  }
  return drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(
      com_traj.get_segment_times(), samples);
}

drake::trajectories::PiecewisePolynomial<double> GenerateModeSequence(
    const std::vector<Gait>& gait_sequence,
    const std::vector<double>& time_points) {
  DRAKE_DEMAND(gait_sequence.size() == time_points.size()-1);

  auto traj = gait_sequence[0].ToTrajectory(time_points[0], time_points[1]);
  for (int i = 1; i < gait_sequence.size(); i++) {
    traj.ConcatenateInTime(
        gait_sequence[i].ToTrajectory(time_points[i], time_points[i + 1]));
  }
  return traj;
}

drake::trajectories::PiecewisePolynomial<double> GenerateGrfReference(
    const drake::trajectories::PiecewisePolynomial<double>& mode_trajectory,
    double m) {
  std::vector<drake::MatrixX<double>> samples;
  const int n_contact_points = mode_trajectory.rows();

  for (const auto& time : mode_trajectory.get_segment_times()) {
    const auto& mode = mode_trajectory.value(time);
    double num_in_contact = mode.sum();
    auto& grf =
        samples.emplace_back(Eigen::VectorXd::Zero(3 * n_contact_points));
    for (int i = 0; i < n_contact_points; i++) {
      if (mode.coeff(i)) {
        grf.coeffRef(2 + 3 * i) = m * 9.81 / num_in_contact;
      }
    }
  }
  return drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(
      mode_trajectory.get_segment_times(), samples);
}

drake::trajectories::PiecewisePolynomial<double> GenerateContactPointReference(
    const drake::multibody::MultibodyPlant<double>& plant,
    const std::vector<dairlib::multibody::WorldPointEvaluator<double>>&
        contacts,
    const drake::trajectories::PiecewisePolynomial<double>& q_traj,
    const drake::trajectories::PiecewisePolynomial<double>& v_traj) {
  auto context = plant.CreateDefaultContext();
  std::vector<double> break_points = q_traj.get_segment_times();
  std::vector<drake::MatrixX<double>> samples;
  int n_contact = contacts.size();
  for (const auto& time : break_points) {
    dairlib::multibody::SetPositionsIfNew<double>(plant, q_traj.value(time),
                                                  context.get());
    dairlib::multibody::SetVelocitiesIfNew<double>(plant, v_traj.value(time),
                                                   context.get());
    auto& knot_point_value =
        samples.emplace_back(Eigen::VectorXd::Zero(6 * n_contact));
    for (int i = 0; i < n_contact; i++) {
      knot_point_value.block(i * 3, 0, 3, 1) = contacts[i].EvalFull(*context);
      knot_point_value.coeffRef(i * 3 + 2) = 0;
      knot_point_value.block(3 * n_contact + i * 3, 0, 3, 1) =
          contacts[i].EvalFullTimeDerivative(*context);
    }
  }
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      break_points, samples);
}
