#include "reference_generation_utils.h"

drake::trajectories::PiecewisePolynomial<double> GenerateComTrajectory(const Eigen::Vector3d& current_com,
                                                                       const std::vector<Eigen::Vector3d>& vel_ewrt_w,
                                                                       const std::vector<double>& time_points){
  DRAKE_DEMAND(vel_ewrt_w.size() == time_points.size());
  auto n_points = vel_ewrt_w.size();

  std::vector<drake::MatrixX<double>> samples(n_points);
  samples[0] = current_com;
  for(int i = 1; i<n_points; i++){
    samples[i] = samples[i-1] + (time_points[i] - time_points[i-1]) * vel_ewrt_w[i-1];
  }
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(time_points, samples);
}

drake::trajectories::PiecewisePolynomial<double> GenerateGeneralizedPosTrajectory(const Eigen::VectorXd& nominal_stand,
                                                                                  const Eigen::Vector3d& base_rt_com_ewrt_w,
                                                                                  const drake::trajectories::PiecewisePolynomial<double>& com_traj,
                                                                                  int base_pos_start){
  auto n_points = com_traj.get_segment_times().size();
  std::vector<drake::MatrixX<double>> samples(n_points);
  for(int i = 0; i < n_points; i++){
    samples[i] = nominal_stand;
    samples[i].block<3,1>(base_pos_start,0, 3, 1) = com_traj.value(com_traj.get_segment_times()[i]) + base_rt_com_ewrt_w;
  }
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(com_traj.get_segment_times(), samples);
}

drake::trajectories::PiecewisePolynomial<double> GenerateGeneralizedVelTrajectory(const drake::trajectories::PiecewisePolynomial<double>& com_traj,
                                                                                  int n_v,
                                                                                  int base_vel_start){
  auto n_points = com_traj.get_segment_times().size();
  std::vector<drake::MatrixX<double>> samples(n_points);
  for(int i = 0; i < n_points; i++){
    samples[i] = drake::VectorX<double>::Zero(n_v);
    samples[i].block<3,1>(base_vel_start,0, 3, 1) = com_traj.derivative().value(com_traj.get_segment_times()[i]);
  }
  return drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(com_traj.get_segment_times(), samples);
}


drake::trajectories::PiecewisePolynomial<double> GenerateModeSequence(const std::vector<Gait>& gait_sequence,
                                                                      const std::vector<double>& time_points){
  DRAKE_DEMAND(gait_sequence.size() == time_points.size());

  auto traj = gait_sequence[0].ToTrajectory(time_points[0], time_points[1]);
  for(int i = 1; i<gait_sequence.size() - 1 ; i++){
    traj.ConcatenateInTime(gait_sequence[i].ToTrajectory(time_points[i], time_points[i + 1]));
  }
  return traj;
}
