#pragma once

#include <string>
#include "attic/systems/trajectory_optimization/hybrid_dircon.h"

namespace dairlib {
namespace goldilocks_models  {

std::shared_ptr<systems::trajectory_optimization::HybridDircon<double>> sgdIter(
    double stride_length, double duration, int iter, std::string directory,
    std::string init_file, std::string weights_file, std::string output_prefix);

}  // namespace goldilocks_models
}  // namespace dairlib
