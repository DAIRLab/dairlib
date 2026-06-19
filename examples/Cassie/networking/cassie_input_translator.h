#pragma once

#include <map>
#include <string>
#include <vector>

#include "examples/Cassie/datatypes/cassie_user_in_t.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"
#include "systems/framework/timestamped_vector.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/// Translates from a TimestamedVector of cassie torque commands into
/// a cassie_user_in_t struct for transmission to the real robot.
class CassieInputTranslator : public drake::systems::LeafSystem<double> {
 public:
  explicit CassieInputTranslator(
      const drake::multibody::MultibodyPlant<double>&);

 private:
  void Output(const drake::systems::Context<double>& context,
              cassie_user_in_t* output) const;

  std::vector<int> userin_to_uvector_index_;
};

}  // namespace systems
}  // namespace dairlib
