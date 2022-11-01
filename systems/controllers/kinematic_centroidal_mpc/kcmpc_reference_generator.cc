#include "kcmpc_reference_generator.h"
#include "multibody/multibody_utils.h"
#include "systems/controllers/kinematic_centroidal_mpc/reference_generation_utils.h"

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
