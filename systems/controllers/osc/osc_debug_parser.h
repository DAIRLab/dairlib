#pragma once
#include "drake/systems/framework/leaf_system.h"

namespace dairlib::systems::controllers {
class OscDebugParser : public drake::systems::LeafSystem<double> {
 public:
  OscDebugParser(std::string traj_name_of_interest);
  const drake::systems::OutputPort<double>& get_error_y_output_port() const {
    return this->get_output_port(error_y_idx_);
  }

 private:
  void CopyErrorY(const drake::systems::Context<double>& context,
                  drake::systems::BasicVector<double>* output) const;

  drake::systems::OutputPortIndex error_y_idx_;
  std::string traj_name_;
};
}