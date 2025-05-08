#include "systems/sender_systems/build_functions.h"
#include "systems/sender_systems/sender_systems.h"

namespace dairlib {
namespace systems {

using DynamicallyFeasiblePlanSenderActor = GenericSender<dairlib::lcmt_timestamped_saved_traj,
                                     std::vector<Eigen::VectorXd>,
                                     &BuildDynamicallyFeasiblePlanSenderActor>;
using DynamicallyFeasiblePlanSenderObject = GenericSender<dairlib::lcmt_timestamped_saved_traj,
                                     std::vector<Eigen::VectorXd>,
                                     &BuildDynamicallyFeasiblePlanSenderObject>;
using TrackingTrajectoryActorSender = GenericSender<dairlib::lcmt_timestamped_saved_traj,
                                        LcmTrajectory,
                                        &BuildTrackingTrajectoryActorSender>;
using TrackingTrajectoryObjectSender = GenericSender<dairlib::lcmt_timestamped_saved_traj,
                                        LcmTrajectory,
                                        &BuildTrackingTrajectoryObjectSender>;
using C3TrajectoryActorSender = GenericSender<dairlib::lcmt_timestamped_saved_traj,
                                        C3Output::C3Solution,
                                        &BuildC3TrackingTrajectoryActorSender>;
using C3TrajectoryObjectSender = GenericSender<dairlib::lcmt_timestamped_saved_traj,
                                        C3Output::C3Solution,
                                        &BuildC3TrackingTrajectoryObjectSender>;
}  // namespace systems
}  // namespace dairlib