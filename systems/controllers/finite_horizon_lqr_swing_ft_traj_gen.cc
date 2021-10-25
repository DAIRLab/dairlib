#include "finite_horizon_lqr_swing_ft_traj_gen.h"

namespace dairlib::systems {

using drake::multibody::MultibodyPlant;
using drake::systems::LinearSystem;
using drake::systems::Context;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Eigen::Matrix;

FiniteHorizonLqrSwingFootTrajGenerator::FiniteHorizonLqrSwingFootTrajGenerator(
    const MultibodyPlant<double>& plant,
    const std::vector<int> left_right_support_fsm_states,
    const std::vector<double> left_right_support_durations,
    const std::vector<std::pair<const Eigen::Vector3d,
                          const drake::multibody::Frame<double>&>> pts,
    const SwingFootTajGenOptions opts) :
    plant_(plant),
    plant_context_(plant_.CreateDefaultContext()),
    double_integrator_(LinearSystem<double>(
        A_, B_, Vector4d::Identity(), Matrix<double, 4, 2>::Zero())),
    double_integrator_context_(double_integrator_.CreateDefaultContext()),
    opts_(opts),
    left_right_support_fsm_states_(left_right_support_fsm_states),
    left_right_support_durations_(left_right_support_durations),
    pts_(pts) {

  }
}