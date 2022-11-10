#include "kcmpc_reference_generator.h"

#include "multibody/multibody_utils.h"
#include "systems/controllers/kinematic_centroidal_mpc/reference_generation_utils.h"

KcmpcReferenceGenerator::KcmpcReferenceGenerator(
    const drake::multibody::MultibodyPlant<double>& plant,
    const Eigen::VectorXd& nominal_stand,
    const std::vector<dairlib::multibody::WorldPointEvaluator<double>>&
        contacts)
    : plant_(plant),
      contacts_(contacts),
      nominal_stand_(nominal_stand),
      context_(plant.CreateDefaultContext()) {
  dairlib::multibody::SetPositionsIfNew<double>(plant_, nominal_stand_,
                                                context_.get());
  p_ScmBase_W_ = nominal_stand_.segment(4, 3) -
                 plant_.CalcCenterOfMassPositionInWorld(*context_);
  m_ = plant_.CalcTotalMass(*context_);
}

void KcmpcReferenceGenerator::Build() {
  Build(nominal_stand_.segment(4, 3) - p_ScmBase_W_);
}

void KcmpcReferenceGenerator::Build(const Eigen::Vector3d& com) {
  com_trajectory_ = GenerateComTrajectory(com, com_vel_knot_points_.samples,
                                          com_vel_knot_points_.times);
  q_trajectory_ = GenerateGeneralizedPosTrajectory(nominal_stand_, p_ScmBase_W_,
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
