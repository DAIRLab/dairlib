#pragma once

// Ground truth for the jamming study: does the object actually go anywhere when
// the end effector executes a candidate sample's plan?
//
// Everything in jamming_metrics.h is derived from the LCS, while this file
// generates a more ground-truth measurement.  This replays a sample's retimed
// EE plan through the demo's Drake sim and measures what the object does.
//
// The label is a difference between the object's motion when the EE stays still
// versus when the EE tracks its C3 plan.

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/geometry/scene_graph.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"

namespace dairlib {
namespace systems {

/// What the real sim did with one candidate sample's plan.  Distances in
/// meters, rotations in radians, forces in Newtons.
struct GroundTruthLabel {
  /// How far the object's origin got from where it started while the end
  /// effector tracked the plan.
  double sim_object_travel = std::numeric_limits<double>::quiet_NaN();
  /// Its largest reorientation over the same window, about whatever axis.
  double sim_object_rotation = std::numeric_limits<double>::quiet_NaN();
  /// The same travel with the end effector held still: what the scene does on
  /// its own, which is the baseline the push has to beat.
  double sim_object_travel_passive = std::numeric_limits<double>::quiet_NaN();
  /// sim_object_travel - sim_object_travel_passive.  What the push achieved.
  /// Can go slightly negative when the end effector holds the cone in place.
  double sim_object_progress = std::numeric_limits<double>::quiet_NaN();
  /// Peak distance between where the plan asked the end effector to be and
  /// where it actually was, over the tracked run.  A large value means the
  /// printer's PD could not follow the plan, in which case the label describes
  /// a push that was never made -- so this is the column that says whether the
  /// rest of the row means anything.
  double sim_ee_tracking_error = std::numeric_limits<double>::quiet_NaN();
  /// How far the plan asked the end effector to travel in the first place, so a
  /// plan that simply did not ask for much is distinguishable from one that
  /// asked and was refused.
  double plan_ee_displacement = std::numeric_limits<double>::quiet_NaN();
  /// Peak total contact force anywhere in the scene during the tracked run.
  /// Unclassified -- it includes the object's own weight on the build plate --
  /// so it is a diagnostic, not the label.
  double sim_max_contact_force = std::numeric_limits<double>::quiet_NaN();
  /// 1.0 when the plan was a real one and still achieved nothing: the jam.
  /// NaN when the caller could not say whether the plan was real.
  double jammed = std::numeric_limits<double>::quiet_NaN();
};

/// The scene's velocities at the instant a rollout starts.  The offline sweep
/// freezes the scene and leaves these all zero; a caller scoring a live
/// candidate passes what the object and end effector are actually doing, since
/// a rollout that pretends the scene is at rest describes a different push.
struct RolloutInitialVelocities {
  Eigen::Vector3d ee_velocity = Eigen::Vector3d::Zero();
  /// Both expressed in the world frame, matching Drake's floating body
  /// velocities and the LCS state layout alike.
  Eigen::Vector3d object_angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d object_linear_velocity = Eigen::Vector3d::Zero();
};

/// Progress below which a push counts as having achieved nothing, in meters.
/// The only tuned constant in the label, and it is a length rather than a
/// force: a millimeter is well inside the noise of where the cone ends up, and
/// two orders of magnitude below the ~10 cm a working push moves it.
constexpr double kJammedProgressThreshold = 1e-3;

/// Replays candidate plans through the demo's real sim.  Builds its plant once
/// and reuses it, so labelling a sample costs one reset and two short rollouts.
class JammingGroundTruthSim {
 public:
  /// @param object_models the sim's object SDFs, from its sim_params.yaml.
  /// @param sim_dt the sim's discrete time step, likewise from sim_params.
  /// @param settle_fraction how long to hold the plan's last end effector
  ///        position after the plan runs out, as a fraction of the plan's own
  ///        duration, so the object's response finishes inside the window.
  JammingGroundTruthSim(const std::vector<std::string>& object_models,
                        double sim_dt, double settle_fraction = 1.0);

  /// Simulates @p ee_plan from the frozen scene and reports what the object
  /// did.  @p knot_dt is the plan's knot spacing.  @p plan_is_real says whether
  /// the plan commanded anything at all (JammingMetrics::no_op_plan == 0); a
  /// sample whose solve produced nothing cannot be jammed by it, and passing
  /// the flag through keeps that judgement where it was already made rather
  /// than re-deriving it here.  Pass a negative value to leave `jammed` NaN.
  GroundTruthLabel Label(const Eigen::Vector4d& object_quaternion,
                         const Eigen::Vector3d& object_position,
                         const std::vector<Eigen::Vector3d>& ee_plan,
                         double knot_dt, int plan_is_real) const;

  /// One rollout: drive the printer to each entry of @p ee_plan in turn, then
  /// hold the last one through the settle window.  Returns the object's peak
  /// travel and rotation, and writes the peak contact force, the peak end
  /// effector tracking error, and the per-knot state if asked.
  ///
  /// @p knot_states, when given, receives the scene state at the start and
  /// after every step, so it is one longer than the number of steps taken --
  /// exactly ee_plan.size() entries when settle_fraction is zero.  Each entry
  /// is in the controller's single-object LCS layout,
  /// [ee_position(3), object_quaternion(4), object_position(3),
  ///  ee_velocity(3), object_angular_velocity(3), object_linear_velocity(3)],
  /// so a caller can score it against the same Q the LCS-based cost types use.
  ///
  /// Holds no state of its own: every rollout builds its own context, and the
  /// only plant methods it calls are const ones writing into that context.  So
  /// concurrent rollouts on one instance are safe, and the controller's
  /// parallel cost loop relies on it.
  void Rollout(const Eigen::Vector4d& object_quaternion,
               const Eigen::Vector3d& object_position,
               const std::vector<Eigen::Vector3d>& ee_plan, double knot_dt,
               double* travel, double* rotation, double* max_contact_force,
               double* max_ee_tracking_error,
               const RolloutInitialVelocities& initial_velocities = {},
               std::vector<Eigen::VectorXd>* knot_states = nullptr) const;

  /// The offset between an end effector world position and the printer joint
  /// coordinates that put the tip there, resolved from the plant at
  /// construction rather than assumed.
  const Eigen::Vector3d& ee_to_joint_offset() const {
    return ee_to_joint_offset_;
  }

  static std::vector<std::string> ColumnNames();
  static Eigen::VectorXd AsRow(const GroundTruthLabel& label);

 private:
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  drake::multibody::MultibodyPlant<double>* plant_ = nullptr;
  drake::multibody::ModelInstanceIndex printer_index_;
  drake::multibody::BodyIndex object_body_index_;
  Eigen::Vector3d ee_to_joint_offset_ = Eigen::Vector3d::Zero();
  double settle_fraction_ = 1.0;
};

}  // namespace systems
}  // namespace dairlib
