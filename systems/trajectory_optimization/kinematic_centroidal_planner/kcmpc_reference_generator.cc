#include "systems/trajectory_optimization/kinematic_centroidal_planner/kcmpc_reference_generator.h"

#include "multibody/multibody_utils.h"
#include "systems/trajectory_optimization/kinematic_centroidal_planner/reference_generation_utils.h"

KcmpcReferenceGenerator::KcmpcReferenceGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* context,
    const std::vector<dairlib::multibody::WorldPointEvaluator<double>>&
        contacts)
    : plant_(plant), context_(context), contacts_(contacts) {
  m_ = plant_.CalcTotalMass(*context_);
}

void KcmpcReferenceGenerator::Generate() {
  dairlib::multibody::SetPositionsIfNew<double>(plant_, q_ref_, context_);
  p_ScmBase_W_ =
      q_ref_.segment(4, 3) - plant_.CalcCenterOfMassPositionInWorld(*context_);
  Eigen::Vector3d com = q_ref_.segment(4, 3) - p_ScmBase_W_;
  com_trajectory_ = GenerateComTrajectory(com, com_vel_knot_points_.samples,
                                          com_vel_knot_points_.times);
  momentum_trajectory_ = GenerateMomentumTrajectory(
      com_vel_knot_points_.samples, com_vel_knot_points_.times, m_);
  q_trajectory_ = GenerateGeneralizedPosTrajectory(q_ref_, p_ScmBase_W_,
                                                   com_trajectory_, 4);
  v_trajectory_ = GenerateGeneralizedVelTrajectory(com_trajectory_,
                                                   plant_.num_velocities(), 3);
  contact_sequence_ =
      GenerateModeSequence(gait_knot_points_.samples, gait_knot_points_.times);
  grf_traj_ = GenerateGrfReference(contact_sequence_, m_);
  contact_traj_ = GenerateContactPointReference(plant_, contacts_,
                                                q_trajectory_, v_trajectory_);
}

void KcmpcReferenceGenerator::SetComKnotPoints(
    const KnotPoints<Eigen::Vector3d>& com_knot_points) {
  DRAKE_DEMAND(com_knot_points.samples.size() ==
               (com_knot_points.times.size() - 1));
  com_vel_knot_points_ = com_knot_points;
}

void KcmpcReferenceGenerator::SetGaitSequence(
    const KnotPoints<Gait>& gait_knot_points) {
  DRAKE_DEMAND(gait_knot_points.samples.size() ==
               (gait_knot_points.times.size() - 1));
  for (const auto& gait : gait_knot_points.samples) {
    gait.check_valid();
  }
  gait_knot_points_ = gait_knot_points;
}

std::vector<double> KcmpcReferenceGenerator::GenerateTimePoints(
    const std::vector<double>& duration_scaling,
    std::vector<Gait> gait_sequence) {
  std::vector<double> durations = std::vector<double>(gait_sequence.size());
  for (int i = 0; i < gait_sequence.size(); ++i) {
    durations[i] = duration_scaling[i] * gait_sequence[i].period;
  }
  std::vector<double> time_points;
  double start_time = 0;
  time_points.push_back(start_time);
  for (auto duration : durations) {
    time_points.push_back(time_points.back() + duration);
  }
  return time_points;
}
