#include <Eigen/Dense>

#include "multibody/multibody_utils.h"

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using Eigen::VectorXd;

namespace dairlib {

void UpdateContext(const int& n_q, const int& n_v, const int& n_u,
                   drake::multibody::MultibodyPlant<double>& plant,
                   drake::systems::Context<double>* context,
                   drake::multibody::MultibodyPlant<AutoDiffXd>& plant_ad,
                   drake::systems::Context<AutoDiffXd>* context_ad,
                   Eigen::VectorXd lcs_state);

}  // namespace dairlib
