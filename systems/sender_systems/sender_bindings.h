#include "systems/sender_systems/build_functions.h"
#include "systems/sender_systems/sender_systems.h"

namespace dairlib {
namespace systems {

using IsC3ModeSender = GenericSender<dairlib::lcmt_timestamped_saved_traj,
                               Eigen::VectorXd,
                               &BuildC3ModeSender>;

using SampleCostSender = GenericSender<dairlib::lcmt_timestamped_saved_traj,
                                     std::vector<double>,
                                     &BuildSampleCostSender>;
                                     
using SampleLocationSender = GenericSender<dairlib::lcmt_timestamped_saved_traj,
                                     std::vector<Eigen::Vector3d>,
                                     &BuildSampleLocationSender>;
}  // namespace systems
}  // namespace dairlib