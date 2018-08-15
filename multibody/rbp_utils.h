#include "drake/common/default_scalars.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/joints/floating_base_types.h"

namespace dairlib {
namespace multibody {
namespace utils {

template<typename T>
drake::VectorX<T> CalcMVdot(const drake::systems::RigidBodyPlant<double>& plant, 
                            drake::VectorX<T> q,
                            drake::VectorX<T> v,
                            drake::VectorX<T> u,
                            drake::VectorX<T> lambda);


template<typename T>
drake::VectorX<T> CalcTimeDerivativesUsingLambda(const drake::systems::RigidBodyPlant<T>& plant, 
                                                 drake::VectorX<T> x,
                                                 drake::VectorX<T> u,
                                                 drake::VectorX<T> lambda);



} // namespace utils
} // namespace multibody
} // namespace dairlib

