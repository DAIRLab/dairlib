#pragma once

#include "systems/trajectory_optimization/hybrid_dircon.h"

using std::string;
using drake::systems::trajectory_optimization::HybridDircon;

namespace drake{
namespace goldilocks_walking {

std::shared_ptr<HybridDircon<double>> sgdIter(double stride_length, double duration, int iter,
    string directory, string init_file, string weights_file, string output_prefix);

}
}