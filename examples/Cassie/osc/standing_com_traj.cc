#include "examples/Cassie/osc/standing_com_traj.h"

#include <algorithm>
#include <math.h>

#include <dairlib/lcmt_cassie_out.hpp>

#include "multibody/multibody_utils.h"

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

using dairlib::systems::OutputVector;
using drake::multibody::Frame;
using drake::multibody::MultibodyPlant;
using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::LeafSystem;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib {
namespace cassie {
namespace osc {

StandingComTraj::StandingComTraj(
    const MultibodyPlant<double>& plant, Context<double>* context,
    const std::vector<std::pair<const Vector3d, const Frame<double>&>>&
        feet_contact_points,
    double height, bool set_target_height_by_radio)
    : plant_(plant),
      context_(context),
      world_(plant_.world_frame()),
      feet_contact_points_(feet_contact_points),
      height_(height),
      set_target_height_by_radio_(set_target_height_by_radio){
  // Input/Output Setup
  state_port_ =
      this->DeclareVectorInputPort("x, u, t",
                                   OutputVector<double>(plant.num_positions(),
                                                        plant.num_velocities(),
                                                        plant.num_actuators()))
          .get_index();
  target_height_port_ =
      this->DeclareAbstractInputPort(
              "lcmt_target_standing_height",
              drake::Value<dairlib::lcmt_target_standing_height>{})
          .get_index();
  radio_port_ =
      this->DeclareAbstractInputPort("radio_out",
                                     drake::Value<dairlib::lcmt_radio_out>{})
          .get_index();
  // Provide an instance to allocate the memory first (for the output)
  PiecewisePolynomial<double> pp(VectorXd(0));
  drake::trajectories::Trajectory<double>& traj_inst = pp;
  this->DeclareAbstractOutputPort("com_xyz", traj_inst,
                                  &StandingComTraj::CalcDesiredTraj);
}

void StandingComTraj::CalcDesiredTraj(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  const auto& radio_out =
      this->EvalInputValue<dairlib::lcmt_radio_out>(context, radio_port_);


  double target_height = height_;

  // Get target height from radio or lcm
  if (set_target_height_by_radio_) {
    target_height = kTargetHeightMean + kTargetHeightScale * radio_out->channel[6];
  } else {
    if (this->EvalInputValue<dairlib::lcmt_target_standing_height>(
        context, target_height_port_)->timestamp > 1e-3) {
      target_height = this->EvalInputValue<dairlib::lcmt_target_standing_height>(
          context, target_height_port_)->target_height;
    }
  }

  // Add offset position from sticks
  target_height += kHeightScale * radio_out->channel[0];

  // Saturate based on min and max height
  target_height = std::clamp(target_height, kMinHeight, kMaxHeight);
  double x_offset = kCoMXScale * radio_out->channel[4];
  double y_offset = kCoMYScale * radio_out->channel[5];
  VectorXd q = robot_output->GetPositions();

  multibody::SetPositionsIfNew<double>(plant_, q, context_);

  // Get center of left/right feet contact points positions
  Vector3d contact_pos_sum = Vector3d::Zero();
  Vector3d position;
  MatrixXd J(3, plant_.num_velocities());
  for (const auto& point_and_frame : feet_contact_points_) {
    plant_.CalcPointsPositions(*context_, point_and_frame.second,
                               point_and_frame.first, world_, &position);
    contact_pos_sum += position;
  }
  Vector3d feet_center_pos = contact_pos_sum / 4;

  // Testing -- filtering feet_center_pos
  if (filtered_feet_center_pos_.norm() == 0) {
    // Initialize
    filtered_feet_center_pos_ = feet_center_pos;
  }
  if (robot_output->get_timestamp() != last_timestamp_) {
    double dt = robot_output->get_timestamp() - last_timestamp_;
    last_timestamp_ = robot_output->get_timestamp();
    double alpha =
        2 * M_PI * dt * cutoff_freq_ / (2 * M_PI * dt * cutoff_freq_ + 1);
    filtered_feet_center_pos_ =
        alpha * feet_center_pos + (1 - alpha) * filtered_feet_center_pos_;
  }
  feet_center_pos = filtered_feet_center_pos_;

  // Desired com pos
  Vector3d desired_com_pos(feet_center_pos(0) + x_offset,
                           feet_center_pos(1) + y_offset,
                           feet_center_pos(2) + target_height);

  // Assign traj
  PiecewisePolynomial<double>* pp_traj =
      (PiecewisePolynomial<double>*)dynamic_cast<PiecewisePolynomial<double>*>(
          traj);
  *pp_traj = PiecewisePolynomial<double>(desired_com_pos);
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
