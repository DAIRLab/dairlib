#include "update_context.h"

#include <Eigen/Dense>

using Eigen::VectorXd;

namespace dairlib {

void UpdateContext(const int& n_q, const int& n_v, const int& n_u,
                   drake::multibody::MultibodyPlant<double>& plant,
                   drake::systems::Context<double>* context,
                   drake::multibody::MultibodyPlant<AutoDiffXd>& plant_ad,
                   drake::systems::Context<AutoDiffXd>* context_ad,
                   const Eigen::VectorXd& x) {
  // Update autodiff.
  VectorXd xu_test(n_q + n_v + n_u);

  // TODO u has always been set to a vector of 1000s here but unsure why or if
  // it matters.
  VectorXd test_u = 1000 * VectorXd::Ones(n_u);

  // Update context with respect to positions and velocities associated with
  // the candidate state.
  VectorXd test_q = x.head(n_q);
  VectorXd test_v = x.tail(n_v);
  xu_test << test_q, test_v, test_u;
  auto xu_ad_test = drake::math::InitializeAutoDiff(xu_test);
  plant_ad.SetPositionsAndVelocities(context_ad, xu_ad_test.head(n_q + n_v));
  multibody::SetInputsIfNew<AutoDiffXd>(plant_ad, xu_ad_test.tail(n_u),
                                        context_ad);

  plant.SetPositions(context, test_q);
  plant.SetVelocities(context, test_v);
  multibody::SetInputsIfNew<double>(plant, test_u, context);
}

}  // namespace dairlib
