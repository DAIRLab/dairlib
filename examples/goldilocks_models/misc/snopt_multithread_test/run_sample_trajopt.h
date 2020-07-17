#pragma once

#include <string>

namespace dairlib {
namespace goldilocks_models  {
namespace misc {

void runSampleTrajopt(
    /*const MultibodyPlant<double> & plant,
    const MultibodyPlant<AutoDiffXd> & plant_autoDiff,*/
    double stride_length, double ground_incline,
    std::string directory, std::string init_file, std::string prefix);

}  // namespace misc
}  // namespace goldilocks_models
}  // namespace dairlib
