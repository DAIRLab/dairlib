#pragma once

#include "dairlib/lcmt_target_standing_height.hpp"
#include "systems/controllers/control_utils.h"
#include "systems/framework/output_vector.h"

#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace cassie {
namespace osc {

static constexpr double kMaxHeight = 1.2;
static constexpr double kMinHeight = 0.3;
static constexpr double kHeightScale = 0.2;
static constexpr double kCoMXScale = 0.05;
static constexpr double kCoMYScale = -0.05;
static constexpr double kMaxTargetHeight = 0.9;
static constexpr double kMinTargetHeight = 0.35;
static constexpr double kTargetHeightMean = (kMinTargetHeight + kMaxTargetHeight) / 2.0;
static constexpr double kTargetHeightScale = (kMaxTargetHeight - kMinTargetHeight) / 2.0;

class StandingComTraj : public drake::systems::LeafSystem<double> {
 public:
  StandingComTraj(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context,
      const std::vector<std::pair<const Eigen::Vector3d,
                                  const drake::multibody::Frame<double>&>>&
          feet_contact_points,
      double height = 0.9,
      bool set_target_height_by_radio = false);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_target_height()
      const {
    return this->get_input_port(target_height_port_);
  }

  const drake::systems::InputPort<double>& get_input_port_radio()
      const {
    return this->get_input_port(radio_port_);
  }

 private:
  void CalcDesiredTraj(const drake::systems::Context<double>& context,
                       drake::trajectories::Trajectory<double>* traj) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;

  int state_port_;
  int target_height_port_;
  int radio_port_;

  // A list of pairs of contact body frame and contact point
  const std::vector<
      std::pair<const Eigen::Vector3d, const drake::multibody::Frame<double>&>>&
      feet_contact_points_;

  double height_;
  bool set_target_height_by_radio_;


  // Testing -- filtering for center of support polygon
  double cutoff_freq_ = 0.05;  // in Hz.
  mutable Eigen::Vector3d filtered_feet_center_pos_ = Eigen::Vector3d::Zero();
  mutable double last_timestamp_ = 0;
};

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
