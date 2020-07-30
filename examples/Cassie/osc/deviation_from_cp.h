#pragma once

#include "systems/framework/output_vector.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace cassie {
namespace osc {

// TODO(yminchen): we can replace cp with raibert style control. (feedforward
//  term is v*T/2)
// TODO(yminchen): we can make global target position an input port of the
//  the system if it's needed in the future.

/// DeviationFromCapturePoint calculates and outputs the deviation from capture
/// point in order to track a desired velocity of center of mass.
///
/// Controller desciprtion:
///  Let delta_r (2 dimensional) be the output of DeviationFromCapturePoint.
///  Since we apply the same control law to both sagital and lateral plane, we
///  will only explain for the case of sagital plane. I.e., consider delta_r is
///  1D here.
///  There are two levels of control, position control and velocity control.
///  The position control is a PD controller which output a desired
///  velocity
///    v_des = k_p * (r_des - r) + k_d * (- rdot),
///  where r/r_des is the current/desired center of mass position, and
///  k_p/k_d is the p/d gain of PD control.
///  The desired velocity is further fed to the velocity controller.
///  The velocity controller is similar to Raibert's foot placement controller.
///  In math, the output of the velocity controller is
///    delta_r = k_ff * (-v_des) + k_fb * (v - v_des),
///  where v/v_des is the current/desired center of mass velocity, and
///  k_ff/k_fb is the gain of the feedforward/feedback term.
///  (Raibert controller uses k_ff = T/2 where T is the stride duration.)
///
///  Additionally, we only apply the above control law when the robot is facing
///  the target position. Otherwise, delta_r = [0; 0].
///
/// Input:
///  - State of the robot
///
/// Output:
///  - A 2D vector, delta_r.
///
/// Requirement: quaternion floating-based Cassie only
class DeviationFromCapturePoint : public drake::systems::LeafSystem<double> {
 public:
  DeviationFromCapturePoint(
      const drake::multibody::MultibodyPlant<double>& plant,
      drake::systems::Context<double>* context);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_des_hor_vel() const {
    return this->get_input_port(xy_port_);
  }

 private:
  void CalcFootPlacement(const drake::systems::Context<double>& context,
                         drake::systems::BasicVector<double>* output) const;

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;
  const drake::multibody::BodyFrame<double>& world_;
  const drake::multibody::Body<double>& pelvis_;
  Eigen::Vector2d global_target_position_;

  Eigen::Vector2d params_of_no_turning_;

  int state_port_;
  int xy_port_;

  // Foot placement control (Sagital) parameters
  double k_fp_ff_sagital_ = 0.16;  // TODO(yminchen): these are for going forward.
  // Should have parameters for going backward
  double k_fp_fb_sagital_ = 0.04;

  // Foot placement control (Lateral) parameters
  double k_fp_ff_lateral_ = 0.08;
  double k_fp_fb_lateral_ = 0.02;
};

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib
