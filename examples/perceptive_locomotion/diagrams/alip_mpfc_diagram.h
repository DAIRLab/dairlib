#pragma once

#include "drake/systems/framework/diagram.h"

namespace dairlib {
namespace perceptive_locomotion {

class AlipMPFCDiagram  : public drake::systems::Diagram<double> {

 public:
  AlipMPFCDiagram(const std::string& gains_filename, double debug_publish_period=0);

 private:

  // finite state machine
  int left_stance_state = 0;
  int right_stance_state = 1;
  int post_left_double_support_state = 3;
  int post_right_double_support_state = 4;

};

} // dairlib
} // perceptive_locomotion

