#include <memory>
#include <string>

#include "drake/systems/framework/leaf_system.h"

#include "examples/Cassie/cassie_utils.h"
#include "systems/robot_lcm_systems.h"

namespace dairlib {
namespace multibody {

/*
 * StateToOutputVectorSystem class that inherits from LeafSystem and serves as a
 * connector that takes in a BasicVector input of the state and copies the
 * contents to an output port of type OutputVector.
 * This is useful to connect to systems that take in an OutputVector type input
 * for the system state
 */
class StateToOutputVectorSystem : public drake::systems::LeafSystem<double> {
 public:
  StateToOutputVectorSystem(int num_positions, int num_velocities,
                            int num_efforts);

 private:
  /*
   * Function to copy the input state to the state part of an OutputVector
   * pointer.
   * @param context context of the system from which the current value of the
   * inputs may be obtained.
   * @param output OutputVector type pointer into which the input state is
   * copied to.
   */
  void CopyOut(const drake::systems::Context<double>& context,
               dairlib::systems::OutputVector<double>* output) const;

  const int num_states_;
};

}  // namespace multibody
}  // namespace dairlib

