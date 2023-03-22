#pragma once

#include <unordered_map>

#include <std_msgs/String.h>
#include "drake/systems/framework/leaf_system.h"

/// Small system to output the frame id of the current stance foot
/// for use with the perception stack

namespace dairlib::perceptive_locomotion {
class StanceFootRosSender : public drake::systems::LeafSystem<double> {
 public:
  StanceFootRosSender(
      std::unordered_map<int, std::string> fsm_to_stance_frame_id_map);
 private:
  void CopyFrame(const drake::systems::Context<double>& context,
                 std_msgs::String* msg) const;
  const std::unordered_map<int, std::string> fsm_to_stance_frame_id_map_;
};
}

