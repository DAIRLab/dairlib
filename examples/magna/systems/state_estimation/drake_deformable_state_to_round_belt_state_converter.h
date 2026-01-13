#include "dairlib/lcmt_round_belt_state.hpp"

#include "drake/math/rigid_transform.h"
#include "drake/systems/framework/leaf_system.h"
namespace dairlib {
namespace examples {
namespace magna {
namespace systems {
namespace state_estimation {

class DrakeDeformableStateToRoundBeltStateConverter
    : public drake::systems::LeafSystem<double> {
 public:
  explicit DrakeDeformableStateToRoundBeltStateConverter(
      std::vector<double> taskboard_position,
      std::vector<double> taskboard_orientation);

  const drake::systems::InputPort<double>& get_input_port_deformable_state()
      const {
    return this->get_input_port(deformable_state_input_port_);
  }
  const drake::systems::OutputPort<double>& get_output_port_round_belt_state()
      const {
    return this->get_output_port(round_belt_state_output_port_);
  }

 private:
  void CopyRoundBeltStateToOutput(
      const drake::systems::Context<double>& context,
      dairlib::lcmt_round_belt_state* output) const;
  drake::systems::InputPortIndex deformable_state_input_port_;
  drake::systems::OutputPortIndex round_belt_state_output_port_;
  std::vector<double> taskboard_position_;
  std::vector<double> taskboard_orientation_;
  drake::math::RigidTransform<double> taskboard_transform_;
};

}  // namespace state_estimation
}  // namespace systems
}  // namespace magna
}  // namespace examples
}  // namespace dairlib
