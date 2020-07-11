#pragma once

#include "drake/common/trajectories/exponential_plus_piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace goldilocks_models {

class RomMPC {
 public:
  RomMPC ();

  void Solve();

  bool is_solved() const {return is_solved_;};

 private:
  bool is_solved_;
};

}  // namespace goldilocks_models
}  // namespace dairlib
