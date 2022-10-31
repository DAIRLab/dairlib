#include "kcmpc_reference_generator.h"
#include "multibody/multibody_utils.h"


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

int FindCurrentMode(const Gait& active_gait, double time_now){
  double phase_now = fmod(time_now/active_gait.period, 1);
  for(int i = 0; i < active_gait.gait_pattern.size(); i++){
    const auto& mode = active_gait.gait_pattern[i];
    if(mode.start_phase <= phase_now && mode.end_phase > phase_now){
      return i;
    }
  }
  DRAKE_ASSERT(false);
  return 0;
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

drake::trajectories::PiecewisePolynomial<double> Gait::ToTrajectory(double current_time, double end_time) const {
  std::vector<double> break_points;
  std::vector<drake::MatrixX<double>> samples;

  // Calculate initial mode index, and phase
  int current_mode = FindCurrentMode(*this, current_time);
  double current_phase = fmod(current_time/period, 1);

  // Loop until time is greater than end time
  while(current_time < end_time){
    // Add the break point for the current time
    break_points.push_back(current_time);
    samples.emplace_back(gait_pattern[current_mode].contact_status.cast<double>());

    // Update time based on how much longer the current mode will last
    current_time += (gait_pattern[current_mode].end_phase - current_phase) * period;
    // Update the mode and mod if necessary
    current_mode = (current_mode + 1) % gait_pattern.size();
    // The new phase is the start phase of the updated mode
    current_phase = gait_pattern[current_mode].start_phase;
  }
  break_points.push_back(end_time);
  samples.push_back(samples[samples.size()-1]);
  return drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(break_points, samples);
}
void Gait::Is_Valid() const {
  DRAKE_ASSERT(period > 0);
  DRAKE_ASSERT(gait_pattern[0].start_phase == 0);
  DRAKE_ASSERT(gait_pattern[gait_pattern.size()-1].end_phase == 1);
  for(int i = 0; i < gait_pattern.size() - 1; i++){
    DRAKE_ASSERT(gait_pattern[i].end_phase == gait_pattern[i+1].start_phase);
  }
}

double TimeNextValue(const drake::trajectories::PiecewisePolynomial<double>& trajectory,
                     double current_time,
                     int index,
                     double value){
  if (trajectory.value(current_time).coeff(index) == value)
    return current_time;

  const auto& segment_times = trajectory.get_segment_times();

  for(int i = trajectory.get_segment_index(current_time) + 1; i<segment_times.size(); i++){
    if(trajectory.value(segment_times[i]).coeff(i) == value)
      return segment_times[i];
  }
  return trajectory.end_time();
}

drake::trajectories::PiecewisePolynomial<double> GenerateGrfReference(const drake::trajectories::PiecewisePolynomial<double>& mode_trajectory,
                                                                      double m){
  std::vector<drake::MatrixX<double>> samples;
  const int n_contact_points = mode_trajectory.rows();

  for(const auto& time: mode_trajectory.get_segment_times()){
    const auto& mode = mode_trajectory.value(time);
    double num_in_contact = mode.sum();
    auto& grf = samples.emplace_back(Eigen::VectorXd::Zero(3 * n_contact_points));
    for(int i = 0; i<n_contact_points; i++){
      if(mode.coeff(i)){
        grf.coeffRef(2 + 3 * i) = m * 9.81 /num_in_contact;
      }
    }
  }
  return drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(mode_trajectory.get_segment_times(), samples);
}

drake::trajectories::PiecewisePolynomial<double> GenerateContactPointReference(const drake::multibody::MultibodyPlant<double> &plant,
                                                                               const std::vector<dairlib::multibody::WorldPointEvaluator<
                                                                                   double>> &contacts,
                                                                               const drake::trajectories::PiecewisePolynomial<
                                                                                   double> &q_traj,
                                                                               const drake::trajectories::PiecewisePolynomial<
                                                                                   double> &v_traj){
  auto context =  plant.CreateDefaultContext();
  std::vector<double> break_points = q_traj.get_segment_times();
  std::vector<drake::MatrixX<double>> samples;
  int n_contact = contacts.size();
  for(const auto& time : break_points){
    dairlib::multibody::SetPositionsIfNew<double>(plant, q_traj.value(time), context.get());
    dairlib::multibody::SetVelocitiesIfNew<double>(plant, v_traj.value(time), context.get());
    auto& knot_point_value = samples.emplace_back(Eigen::VectorXd::Zero(6 * n_contact));
    for(int i = 0; i <n_contact; i++){
      knot_point_value.block(i * 3, 0,  3, 1) = contacts[i].EvalFull(*context);
      knot_point_value.coeffRef(i*3 + 2) = 0;
      knot_point_value.block(3 * n_contact + i * 3, 0, 3, 1) = contacts[i].EvalFullJacobianDotTimesV(*context);
    }
  }
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(break_points, samples);
}

KcmpcReferenceGenerator::KcmpcReferenceGenerator(const drake::multibody::MultibodyPlant<double> &plant,
                                                 const Eigen::VectorXd& nominal_stand,
                                                 const std::vector<dairlib::multibody::WorldPointEvaluator<double>> &contacts):
                                                 plant_(plant),
                                                 contacts_(contacts),
                                                 nominal_stand_(nominal_stand),
                                                 context_(plant.CreateDefaultContext()){
  dairlib::multibody::SetPositionsIfNew<double>(plant_, nominal_stand_, context_.get());
  base_rt_com_ewrt_w = nominal_stand_.segment(4, 3) - plant_.CalcCenterOfMassPositionInWorld(*context_);
  m_ = plant_.CalcTotalMass(*context_);
}


void KcmpcReferenceGenerator::Build() {
  Build(nominal_stand_.segment(4, 3) - base_rt_com_ewrt_w);
}

void KcmpcReferenceGenerator::Build(const Eigen::Vector3d& com) {
  com_trajectory_ = GenerateComTrajectory(com,
                                              com_vel_knot_points_.samples,
                                              com_vel_knot_points_.times);
  q_trajectory_ = GenerateGeneralizedPosTrajectory(nominal_stand_,
                                                       base_rt_com_ewrt_w,
                                                       com_trajectory_,
                                                       4);
  v_trajectory_ = GenerateGeneralizedVelTrajectory(com_trajectory_,
                                                       plant_.num_velocities(),
                                                       3);
  contact_sequence_ = GenerateModeSequence(gait_knot_points_.samples, gait_knot_points_.times);
  grf_traj_ = GenerateGrfReference(contact_sequence_, m_);
  contact_traj_ = GenerateContactPointReference(plant_,contacts_, q_trajectory_, v_trajectory_);
}

void KcmpcReferenceGenerator::SetComKnotPoints(const KnotPoints<Eigen::Vector3d> &com_knot_points) {
  com_vel_knot_points_ = com_knot_points;
}

void KcmpcReferenceGenerator::SetGaitSequence(const KnotPoints<Gait> &gait_knot_points) {
  for(const auto& gait : gait_knot_points.samples){
    gait.Is_Valid();
  }
  gait_knot_points_ = gait_knot_points;
}
