#include "control_refine_sender.h"
#include <iostream>


using dairlib::systems::OutputVector;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::EventStatus;
using Eigen::VectorXd;

namespace dairlib {
namespace systems {

ControlRefineSender::ControlRefineSender(
        const drake::multibody::MultibodyPlant<double>& plant,
        drake::systems::Context<double>& context,
        const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
        drake::systems::Context<drake::AutoDiffXd>& context_ad,
        const std::vector<drake::SortedPair<drake::geometry::GeometryId>> contact_geoms,
        C3Options c3_options) {
    // INPUT PORTS
    c3_solution_port_ =
    this->DeclareVectorInputPort(
                    "c3_solution", OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
            .get_index();

    lcs_state_port_ =
            this->DeclareVectorInputPort(
                            "lcs_state", OutputVector<double>(plant.num_positions(),
                                                              plant.num_velocities(),
                                                              plant.num_actuators()))
                    .get_index();


    // OUTPUT PORTS
    target_port_ =
      this->DeclareVectorOutputPort(
              "track_target", BasicVector<double>(7),
              &ControlRefineSender::CalcTrackTarget)
          .get_index();

    contact_torque_port_ =
            this->DeclareVectorOutputPort(
                            "contact_torque", BasicVector<double>(7),
                            &ControlRefineSender::CalcTrackTarget)
                    .get_index();
}

void ControlRefineSender::SetParameters(const SimulateFrankaParams& sim_param,
                                        const BallRollingTrajectoryParams& traj_param) {
}

void ControlRefineSender::CalcTrackTarget(
    const drake::systems::Context<double>& context,
    drake::systems::BasicVector<double>* target) const {
}
}  // namespace systems
}  // namespace dairlib