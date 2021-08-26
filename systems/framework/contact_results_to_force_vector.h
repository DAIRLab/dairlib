#pragma once

#include <memory>

#include <drake/multibody/plant/multibody_plant.h>

#include "drake/common/drake_copyable.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

/** A System that encodes ContactResults into a vector valued output port
 message. It has a single input port with type ContactResults<T> and a single
 vector-valued output port.

 @tparam_default_scalar
 */
class ContactResultsToForceVector final
    : public drake::systems::LeafSystem<double> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ContactResultsToForceVector)

  /** Constructs a ContactResultsToForceVector.
   @param plant The MultibodyPlant that the ContactResults are generated from.
   @pre The `plant` must be finalized already. The input port of this system
        must be connected to the corresponding output port of `plant`
        (either directly or from an exported port in a Diagram).
  */
  explicit ContactResultsToForceVector(
      const drake::multibody::MultibodyPlant<double>& plant);

  //  /** Scalar-converting copy constructor.  */
  //  template <typename U>
  //  explicit ContactResultsToLcmSystem(const ContactResultsToLcmSystem<U>&
  //  other)
  //      : drake::systems::LeafSystem<T>(), body_names_(other.body_names_) {}

  const drake::systems::InputPort<double>& get_contact_result_input_port()
      const {
    return this->get_input_port(contact_result_input_port_index_);
  }
  const drake::systems::OutputPort<double>& get_contact_forces_output_port()
      const {
    return this->get_output_port(contact_forces_output_port_index_);
  }

 private:
  void ConvertContactResultsToVectorOutput(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const;

  // Named indices for the i/o ports.
  drake::systems::InputPortIndex contact_result_input_port_index_;
  drake::systems::OutputPortIndex contact_forces_output_port_index_;

  // A mapping from geometry IDs to body indices.
  std::unordered_map<drake::geometry::GeometryId, std::string>
      geometry_id_to_body_name_map_;

  // A mapping from body index values to body names.
  std::vector<std::string> body_names_;
};
}  // namespace systems
}  // namespace dairlib
