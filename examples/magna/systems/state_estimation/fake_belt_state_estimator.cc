#include "examples/magna/systems/state_estimation/fake_belt_state_estimator.h"

#include "systems/framework/output_vector.h"

namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace state_estimation {

using dairlib::systems::OutputVector;

FakeBeltStateEstimator::FakeBeltStateEstimator(int num_keypoints) {
  franka_state_input_port_ =
      this->DeclareVectorInputPort(
              "franka_state",
              OutputVector<double>(num_keypoints * 3, num_keypoints * 3, 0))
          .get_index();
}

}  // namespace state_estimation
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
