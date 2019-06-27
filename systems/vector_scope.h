#pragma once

namespace dairlib {
namespace systems {

using drake::systems::Context;
using drake::systems::BasicVector;
using drake::systems::EventStatus;

class VectorScope : public drake::systems::LeafSystem<double> {
 public:
  VectorScope(int size, std::string debugMessage, double publishRate);

 private:
  const std::string msg_;

  EventStatus PrintOutput(const Context<double>& context) const;

};

} // namespace systems
} // namespace dairlib
