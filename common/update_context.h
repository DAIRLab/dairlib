#include <Eigen/Dense>

#include "multibody/multibody_utils.h"

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using Eigen::VectorXd;

namespace dairlib {

/// Updates the context of a MultibodyPlant with the given state vector.
///
/// This function updates both the double and AutoDiff contexts of a MultibodyPlant
/// using the provided state vector. It is useful for setting the state of the
/// plant in simulation or optimization tasks.
///
/// @param n_q Number of generalized positions (q).
/// @param n_v Number of generalized velocities (v).
/// @param n_u Number of control inputs (u).
/// @param plant The MultibodyPlant to update.
/// @param context The context for the MultibodyPlant.
/// @param plant_ad The MultibodyPlant with AutoDiff support.
/// @param context_ad The context for the MultibodyPlant with AutoDiff support.
/// @param x The state vector to set in the contexts.
void UpdateContext(const int& n_q, const int& n_v, const int& n_u,
                   drake::multibody::MultibodyPlant<double>& plant,
                   drake::systems::Context<double>* context,
                   drake::multibody::MultibodyPlant<AutoDiffXd>& plant_ad,
                   drake::systems::Context<AutoDiffXd>* context_ad,
                   Eigen::VectorXd x);

}  // namespace dairlib
