#include "drake/common/default_scalars.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/joints/floating_base_types.h"

namespace dairlib {
namespace multibody {
namespace utils {


drake::VectorX<drake::AutoDiffXd> CalcMVdot(drake::systems::RigidBodyPlant<double>* plant, 
                                     drake::VectorX<drake::AutoDiffXd> q,
                                     drake::VectorX<drake::AutoDiffXd> v,
                                     drake::VectorX<drake::AutoDiffXd> u,
                                     drake::VectorX<drake::AutoDiffXd> lambda);

drake::VectorX<double> CalcMVdot(drake::systems::RigidBodyPlant<double>* plant, 
                                 drake::VectorX<double> q,
                                 drake::VectorX<double> v,
                                 drake::VectorX<double> u,
                                 drake::VectorX<double> lambda);

Eigen::VectorXd CalcTimeDerivativesUsingLambda(drake::systems::RigidBodyPlant<double>* plant, 
                                               Eigen::VectorXd x,
                                               Eigen::VectorXd u,
                                               Eigen::VectorXd lambda);



} // namespace utils
} // namespace multibody
} // namespace dairlib

