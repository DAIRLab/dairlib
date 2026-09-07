#pragma once

// A deliberately cheap approximation of the label in jamming_ground_truth.h.
//
// The ground truth labeller is the reference and is not touched.  It costs two
// 1.5 s rollouts at a 1 ms step per sample, which is far more than a controller
// can afford per candidate sample.  This file trades accuracy for speed along
// four independent axes -- the passive rollout, the time step, the contact
// model, and the settle window -- so that the trade can be measured against the
// ground truth rather than assumed.
//
// The jam definition is deliberately identical to the ground truth's: travel is
// the largest distance the object's origin gets from where it started, taken
// over the whole window.  An object that moves well and only then wedges scores
// high and does not read as jammed, here or there.  Matching the reference is
// the goal, so that blind spot is reproduced rather than fixed.
//
// What is NOT reproduced is the passive baseline.  The ground truth subtracts a
// rollout with the end effector held still, which in the cone sweep contributes
// a constant 9.9938e-05 m of gravity settle to every sample -- verified to
// 5e-11 across a 2000-sample sweep -- so subtracting it is exactly a shift of
// the threshold.  Travel is thresholded raw against that shifted value instead,
// which halves the work for a label that is identical by construction rather
// than merely close.

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

namespace dairlib {
namespace systems {

/// The knobs, all in one place so a study is a loop over configurations rather
/// than a rebuild.
///
/// The defaults are the configuration the cone sweep measured as the best
/// speed-for-agreement trade: 4 ms, a quarter settle window, the early exit,
/// and the reference threshold shifted by the passive settle.  That came to
/// 0.9 ms per sample against the ground truth's 178 ms -- about a fourteenth
/// of the work per sample (7.2x from the knobs, 2x from dropping the passive
/// rollout), the rest of the wall-clock gap being parallelism --
/// and disagreed with it on 0.6% and 1.1% of real plans across two independent
/// 1000-sample draws.  Construct with FastJammingLabelConfig::Reference() for
/// the configuration that reproduces the reference label instead.
///
/// Every figure quoted on these knobs is one draw's measurement and moves by a
/// few tenths of a point between draws, so treat them as ranges, not
/// constants.  Regenerate them with jamming_sweep --label_variants.
struct FastJammingLabelConfig {
  /// The plant's discrete step.  Must be positive: the printer's actuator PD
  /// gains, which are what makes the end effector track a plan at all, are only
  /// installed on a discrete plant.  Stability is not what bounds this -- the
  /// plant is discrete/SAP, so the stiff terms (the axes' joint damping, the PD
  /// gains, and the 1e7 N/m contact stiffness on a 34.5 g cone) are all handled
  /// implicitly -- accuracy is.  Plan targets change only every knot_dt, so
  /// even 8 ms leaves several steps per knot.  4 ms is the measured sweet
  /// spot: ~3.7x cheaper than 1 ms for 0.6-0.8% disagreement with the ground
  /// truth.  8 ms is another 1.8x but is the least stable knob across draws
  /// (0.7% on one, 1.8% on the next, with the missed jams growing faster than
  /// the false ones), which is what keeps the default at 4.
  double sim_dt = 0.004;

  /// Use point contact instead of Drake's default hydroelastic-with-fallback.
  /// The cone declares compliant hydroelastic properties on a *mesh*, so the
  /// default builds tet meshes and computes contact surfaces every step for
  /// what is geometrically a 7-vertex cone.  Every collision in the scene also
  /// declares a point contact stiffness, so the point model is fully
  /// parameterised; it is a different contact model, though, not a cheaper
  /// solve of the same one, so it changes the physics.
  ///
  /// Measured at exactly 1.0x on the cone sweep -- the hydroelastic work this
  /// was meant to avoid is not where the time goes -- so it is off by default.
  /// It costs nothing in agreement either, which is why it is still offered.
  bool point_contact = false;

  /// How long to hold the plan's last end effector position after the plan runs
  /// out, as a fraction of the plan's own duration.  The reference holds for a
  /// full extra plan duration so the object's response finishes inside the
  /// measurement; shortening it can only lower travel, since travel is a
  /// running maximum.
  ///
  /// A quarter buys 1.5x for 0.2-0.5% disagreement; dropping it entirely buys
  /// 1.9x for 0.4-0.5%, but starts costing samples whose response had not
  /// finished, which is why a quarter is the default.
  double settle_fraction = 0.25;

  /// Travel below which the push counts as having achieved nothing, in meters.
  ///
  /// This is the reference's own kJammedProgressThreshold (1e-3) plus the
  /// passive gravity settle that the reference subtracts and this class does
  /// not (9.9938e-05 for the cone scene) -- a derived quantity, not a tuned
  /// one, since subtracting a constant and shifting the threshold by it are
  /// the same operation.
  ///
  /// It is deliberately NOT coarsened to cover sim noise, which was the
  /// expectation going in.  The measurement says the fast rollout's travel is
  /// faithful enough not to need it: at this threshold the reference physics
  /// reproduces the ground truth exactly, 0 errors in 846 real plans, while at
  /// 2e-3 the same rollout disagrees on 7.4-8.0% -- raising the threshold
  /// turns samples that genuinely moved a millimetre or two into false jams.
  /// Agreement and speed both want it low; a lower threshold also makes the
  /// early exit fire sooner.
  ///
  /// The label is not hypersensitive nearby (0.4% at 1.0e-3, 0.6% here), so a
  /// scene whose settle differs somewhat does not need this re-derived.  Above
  /// ~1.5e-3 it degrades quickly.
  double travel_threshold = 1.10e-3;

  /// Stop the rollout as soon as travel crosses the threshold.  Travel is a
  /// running maximum, so a crossing can never be undone and the label is
  /// already decided: this is exact under the definition above, not an
  /// approximation.  It does mean the reported travel is only a lower bound on
  /// a sample that exits early, which is why FastJammingLabel reports whether
  /// it did.  Turn it off to measure travel itself.
  bool early_exit = true;

  /// Replace the PD-tracked printer with an end effector whose position is
  /// written directly into the state each step.  This drops the three actuated
  /// axes, the stiff PD, and the joint damping -- the stiffest part of the
  /// system, and what really bounds sim_dt.  It also makes the end effector
  /// infinitely stiff, so it ploughs through the object instead of stalling
  /// against it, and a jam *is* the end effector stalling.
  ///
  /// Measured on the cone sweep it fails on both counts: 1.0x speed, and the
  /// worst agreement of any knob (5-6%, rank correlation 0.85-0.89, and the
  /// only knob whose disagreement reaches samples that plainly moved).  Kept
  /// only so the result stays reproducible.
  bool prescribed_ee = false;

  /// The ground truth's physics: its step, contact model and settle window.
  /// Differs from the defaults above only in those, so the gap between the two
  /// measures what the cheap physics costs and nothing else.
  ///
  /// Its threshold is the ground truth's own, WITHOUT the passive settle the
  /// default folds in -- that settle belongs to a scene, not to this class, so
  /// a caller reproducing the reference label exactly has to add its own.
  static FastJammingLabelConfig Reference();

  /// A short slug naming the knob settings, for column names in a study.
  std::string Describe() const;
};

/// What the fast sim made of one candidate sample's plan.
struct FastJammingLabel {
  /// Farthest the object's origin got from where it started, in meters.  A
  /// lower bound rather than the maximum when exited_early is true.
  double travel = std::numeric_limits<double>::quiet_NaN();
  /// 1.0 when the plan was a real one and still moved the object less than
  /// travel_threshold.  NaN when the caller could not say whether the plan was
  /// real, matching GroundTruthLabel::jammed.
  double jammed = std::numeric_limits<double>::quiet_NaN();
  /// Whether the rollout stopped as soon as the label was decided, in which
  /// case travel is a lower bound and must not be used as a measurement.
  bool exited_early = false;
};

/// The fast labeller.  Builds its plant once and holds no mutable state, so
/// Label() is const and safe to call concurrently on one instance -- which is
/// the point: this is meant to run inside the controller's per-sample OpenMP
/// loop, where nothing can be hoisted out and shared.
class FastJammingLabelSim {
 public:
  /// @param object_models the sim's object SDFs, from its sim_params.yaml.
  /// @param config the knobs above.
  FastJammingLabelSim(const std::vector<std::string>& object_models,
                      const FastJammingLabelConfig& config);

  /// Simulates @p ee_plan from the frozen scene and labels what the object did.
  /// @p knot_dt is the plan's knot spacing.  @p plan_is_real says whether the
  /// plan commanded anything at all; a negative value leaves `jammed` NaN.
  /// Const, and touches no member state, so it may be called from many threads
  /// at once.
  FastJammingLabel Label(const Eigen::Vector4d& object_quaternion,
                         const Eigen::Vector3d& object_position,
                         const std::vector<Eigen::Vector3d>& ee_plan,
                         double knot_dt, int plan_is_real) const;

  /// The offset between an end effector world position and the printer joint
  /// coordinates that put the tip there, resolved from the plant.
  const Eigen::Vector3d& ee_to_joint_offset() const {
    return ee_to_joint_offset_;
  }

  const FastJammingLabelConfig& config() const { return config_; }

 private:
  FastJammingLabelConfig config_;
  std::unique_ptr<drake::systems::Diagram<double>> diagram_;
  const drake::multibody::MultibodyPlant<double>* plant_ = nullptr;
  drake::multibody::ModelInstanceIndex printer_index_;
  drake::multibody::BodyIndex object_body_index_;
  Eigen::Vector3d ee_to_joint_offset_ = Eigen::Vector3d::Zero();
};

}  // namespace systems
}  // namespace dairlib
