#pragma once

// Predictors of "how hard will the end effector have to push" for a candidate
// sample location, used to study whether jamming (a wedged object driving the
// commanded EE force high) can be detected before it happens.  See the
// three_d_printer/test/jamming_sweep.cc binary for the offline experiment these
// feed.
//
// Two independent predictors, both derived from the same per-sample C3 solve
// the sampling controller already performs:
//
//   A. The C3 plan itself: the planned input trajectory u (an EE force in
//      Newtons for the 3D printer's three prismatic actuators) and the planned
//      complementarity forces lambda.
//   B. A crude PD rollout of the C3 plan through the LCS, taking the resulting
//      tracking input u and the EE tracking error delta_x.
//
// Both are measured on the *retimed* plan: the C3 solution slowed to the
// printer's EE velocity limits and resampled back onto its own knot times.
// That is the plan the execution path publishes and the one a
// kSimImpedanceRetimedObjectCostOnly cost scores, so measuring it keeps these
// predictors describing the same trajectory as the c3_cost they sit beside.
// Retiming trades the plan's extra duration back for a shorter path: the
// reference covers the same amount of time, but only as far along it as the
// machine can actually get.
//
// When the EE velocity limits are unconfigured there is nothing to retime, and
// the metrics fall back to the plan as solved -- the same no-op
// SamplingC3Controller::RetimeAndResampleC3PlanForCost takes on the cost path.

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "core/c3.h"
#include "core/lcs.h"
#include "multibody/lcs_factory.h"

namespace dairlib {
namespace systems {

/// Peak predicted end effector effort over one C3 planning horizon, for a
/// single candidate sample location.  All force quantities are in Newtons; the
/// delta_x quantities are in meters.
struct JammingMetrics {
  // --- Predictor A: straight from the C3 plan. ---
  double max_u_norm_c3 = 0.0;
  double max_u_xy_c3 = 0.0;
  double max_u_z_c3 = 0.0;
  /// Peak Cartesian force on the EE implied by the planned lambda, summed over
  /// the lambda entries belonging to the EE's contacts.
  double max_ee_contact_force_c3 = 0.0;

  // --- Predictor B: from the PD rollout, at the fine LCS resolution. ---
  double max_u_norm_pd = 0.0;
  double max_u_xy_pd = 0.0;
  double max_u_z_pd = 0.0;
  /// The same peak, but from the coarse/fine rollout overload the controller
  /// itself calls, which downsamples the input trajectory back to the planning
  /// resolution before returning.  Kept alongside max_u_norm_pd to show how
  /// much of the peak that downsampling hides.
  double max_u_norm_pd_coarse = 0.0;

  // Peak EE position tracking error of that same rollout, in METERS: the
  // P-term's error signal with no gain applied.  (A Kd = 0 rollout is not an
  // option -- SimulatePDControlWithLCS requires exactly n_u non-zero entries in
  // both Kp and Kd -- so this comes from the primary rollout.)  Reported
  // gain-free so its magnitude means the same thing regardless of how Kp is
  // tuned; multiply by Kp to recover the P-term force.
  double max_delta_x_ee_norm = 0.0;
  double max_delta_x_ee_xy = 0.0;
  double max_delta_x_ee_z = 0.0;

  /// Peak EE contact force from the PD rollout's lambda.  Getting lambda out of
  /// a rollout needs TrajectoryEvaluator::SimulatePDControlWithLCSAndForces,
  /// which does not exist at the c3 commit dairlib pins, so this stays NaN
  /// until that pin moves.
  double max_ee_contact_force_pd = std::numeric_limits<double>::quiet_NaN();

  // --- Predictor A, continued: how long the plan sits against its u bound. ---
  // The peaks above are censored by the input constraint -- max_u_z_c3 can
  // never exceed u_vertical_limits -- so a plan that wants far more force than
  // allowed is indistinguishable from one that merely reaches the bound.  These
  // record the *fraction of knots* pinned there, which keeps rising after the
  // peak has saturated.  NaN when the caller passed no input limits.
  double frac_knots_u_xy_at_limit = std::numeric_limits<double>::quiet_NaN();
  double frac_knots_u_z_at_limit = std::numeric_limits<double>::quiet_NaN();

  // --- Object-side: does the thing being pushed actually go anywhere? ---
  // Everything above is measured on the end effector, so it says how hard the
  // push is and nothing about whether the push accomplishes anything.  Jamming
  // is high force with no object progress, and these are the progress half.
  //
  // Both are direction-free magnitudes on purpose.  One leg of the cone demo
  // wedges horizontally against the ramp lip, but other legs jam in other
  // directions, so a metric that singles out an axis would only detect the jam
  // it was written for.  NaN when the caller supplied no object state layout.

  /// Farthest the object's origin gets from where it started, over the PD
  /// rollout, in METERS.
  double object_travel_in_rollout = std::numeric_limits<double>::quiet_NaN();
  /// Largest reorientation of the object over the same rollout, in RADIANS:
  /// the angle of the rotation taking its knot-0 orientation to its knot-i one,
  /// about whatever axis that happens to be.
  double object_rotation_in_rollout = std::numeric_limits<double>::quiet_NaN();
  /// 1.0 when the plan commands essentially no effort -- less than
  /// kNoOpInputFraction of the input budget -- i.e. the solve produced nothing
  /// to execute.  A principled stand-in for the "everything at the same
  /// degenerate c3_cost" test: such samples are failed or empty plans, not safe
  /// ones, and must be masked out of any color scale or ranking rather than
  /// read as low force.  NaN when the caller passed no input limits.
  ///
  /// Deliberately NOT a test on object travel, which is what it looks like it
  /// should be.  The object settles about a millimeter under gravity whatever
  /// the plan does, so a travel threshold low enough to clear that settle never
  /// fires -- and one high enough to catch it also catches a plan that pushes
  /// hard and moves nothing, which is the jam this whole file exists to find.
  /// Measuring the plan's own effort separates "no plan" from "a plan that
  /// achieves nothing" cleanly: in the cone sweep the degenerate samples
  /// command 0.24-0.41 N against a 15 N budget while every real plan commands
  /// over 4.7 N.
  double no_op_plan = std::numeric_limits<double>::quiet_NaN();

  /// Peak Cartesian force between the object and the world, from the planned
  /// lambda -- the ramp and ground being welded to the world, this is what the
  /// scene pushes back with.  The wedging signal: a jammed object braces
  /// against the world instead of moving.  Zero when the caller supplied no
  /// object-ground basis.
  double max_object_ground_force_c3 = 0.0;

  /// The jam index: newtons the world pushes back with per METER the object
  /// actually moves, max_object_ground_force_c3 / object_travel_in_rollout.
  /// An object going where it is meant to go covers ground for a moderate
  /// reaction; a wedged one absorbs a large reaction and stays put.  The
  /// denominator is floored at kMinObjectTravelForJamIndex, so it stays finite
  /// even for a plan that moves nothing at all.
  double object_ground_force_per_travel =
      std::numeric_limits<double>::quiet_NaN();
};

/// Floor on the jam index's denominator, in meters, so a plan that pushes hard
/// and moves nothing reads as a large finite number instead of dividing by
/// zero.  Set below the millimeter or so the object settles under gravity on
/// its own, so in practice it only ever guards the division.
constexpr double kMinObjectTravelForJamIndex = 1e-4;

/// Fraction of the input budget below which a plan counts as commanding
/// nothing.  The budget is the norm of the per-axis limits, so this scales with
/// whatever input box a demo configures rather than hard-coding a force.
constexpr double kNoOpInputFraction = 0.1;

/// The printer's EE speed limits plus where the EE velocity block sits in the
/// LCS state, i.e. everything the retiming needs that cannot be read off the
/// LCS itself.  Default-constructed (or non-positive) speeds mean the limits
/// are unconfigured and the plan is measured unretimed; IsValid() says so.
struct EEVelocityLimits {
  double v_xy_max = 0.0;
  double v_z_max = 0.0;
  /// n_q: the index of the EE velocity block within the LCS state.
  int ee_velocity_offset = 0;

  bool IsValid() const {
    return v_xy_max > 0.0 && v_z_max > 0.0 && ee_velocity_offset >= 3;
  }
};

/// The bounds the C3 solve imposed on u, collapsed from the options' [lo, hi]
/// pairs to the scalar magnitude a plan can reach on each axis.  Only needed to
/// tell whether a knot's input is pinned against its constraint;
/// default-constructed (or non-positive) values mean the caller did not supply
/// them and the saturation fractions come back NaN.
struct UInputLimits {
  double u_xy_max = 0.0;
  double u_z_max = 0.0;

  bool IsValid() const { return u_xy_max > 0.0 && u_z_max > 0.0; }
};

/// Collapses the SC3 options' [lo, hi] input-limit pairs into the scalar
/// magnitudes UInputLimits carries, the same way the controller collapses the
/// EE velocity pair: min(|lo|, |hi|).  A pair with fewer than two entries, or
/// one that collapses to a non-positive magnitude, comes back invalid rather
/// than throwing, so a demo that leaves the limits unconfigured just gets NaN
/// saturation columns.
UInputLimits MakeUInputLimits(const std::vector<double>& u_horizontal_limits,
                              const std::vector<double>& u_vertical_limits);

/// Indices of the lambda entries belonging to one group of contacts, together
/// with the force basis that maps each to a Cartesian force.
struct ContactForceBasis {
  std::vector<int> lambda_indices;
  std::vector<Eigen::Vector3d> force_bases;
};

/// Builds the force basis for the @p num_entries lambda entries starting at
/// @p first_entry.  Under 'anitescu' every lambda entry carries a Cartesian
/// force_basis, so the same machinery works for any contact group; slack
/// entries within the block are skipped.  Size the block with
/// c3::multibody::LCSFactory::GetNumContactVariables over exactly the contacts
/// in it, rather than assuming every contact owns an equal share -- contacts
/// flagged planar carry fewer friction directions than the rest.
ContactForceBasis MakeContactForceBasis(
    const std::vector<c3::multibody::LCSContactDescription>&
        contact_descriptions,
    int first_entry, int num_entries);

/// The end effector's block, which is the leading one: contact pairs are
/// resolved in group order (EE-ground, EE-object, object-ground, object-object,
/// object-wall), so the EE's contacts own the first @p num_ee_lambda_entries
/// entries of lambda.
ContactForceBasis MakeEEContactForceBasis(
    const std::vector<c3::multibody::LCSContactDescription>&
        contact_descriptions,
    int num_ee_lambda_entries);

/// Peak Cartesian force across one contact group at one knot point, from that
/// knot's lambda.
double ContactForceMagnitude(const Eigen::VectorXd& lambda,
                             const ContactForceBasis& basis);

/// The contact groups Predictor A measures forces across.  A default-
/// constructed member contributes no entries, and its metric comes back zero.
struct JammingForceBases {
  /// The EE's contacts: how hard the end effector pushes.
  ContactForceBasis ee;
  /// The object's contacts against the welded world -- ground and ramp: how
  /// hard the scene pushes back.
  ContactForceBasis object_ground;
};

/// Where one object's pose sits inside the LCS state.  The LCS stacks the EE's
/// 3 prismatic positions first and then 7 positions per object (quaternion,
/// then position), which is the layout ShellSampling and the controller's own
/// x_lcs.segment(7 + 7 * i, ...) object-distance code both assume.
struct ObjectStateLayout {
  /// Index of the object's quaternion (w, x, y, z) within the LCS state.
  int quaternion_offset = -1;
  /// Index of the object's position within the LCS state.
  int position_offset = -1;

  bool IsValid() const {
    return quaternion_offset >= 0 && position_offset == quaternion_offset + 4;
  }
};

/// The layout above for object @p object_index, behind a 3-DOF end effector.
ObjectStateLayout MakeObjectStateLayout(int object_index = 0);

/// Splits a solved C3 object's GetFullSolution() into its per knot pieces.
/// Each entry of @p z_plan is laid out [x (n_x); lambda (n_lambda); u (n_u)].
/// Read from z rather than from GetInputSolution()/GetForceSolution(), which
/// disagree with z when end_on_qp_step is false (as the 3D printer demo sets).
/// All three outputs come back with one entry per knot.
void UnpackC3Plan(const std::vector<Eigen::VectorXd>& z_plan, int n_x,
                  int n_lambda, int n_u, std::vector<Eigen::VectorXd>* x_plan,
                  std::vector<Eigen::VectorXd>* lambda_plan,
                  std::vector<Eigen::VectorXd>* u_plan);

/// Predictor A, over an unpacked plan.  @p u_plan and @p lambda_plan have one
/// entry per knot and the same length.
///
/// Valid @p u_limits additionally fill in the saturation fractions and the
/// no_op_plan flag; invalid ones (the default) leave them NaN.  A knot counts
/// as saturated at a relative tolerance rather than on equality: the C3+
/// projection step carries no u bounds, and with end_on_qp_step false the
/// solution is read out of a z that a half-step rollout has overwritten, so the
/// u seen here can sit a solver tolerance past its bound.
void ComputeC3PlanMetrics(const std::vector<Eigen::VectorXd>& u_plan,
                          const std::vector<Eigen::VectorXd>& lambda_plan,
                          const JammingForceBases& bases,
                          JammingMetrics* metrics,
                          const UInputLimits& u_limits = UInputLimits{});

/// Predictor B.  Runs the PD rollout twice: once at the fine LCS's resolution
/// (so peaks between planning knots survive), and once through the coarse/fine
/// overload the controller uses, which downsamples.  The gain-free EE tracking
/// error is read off the fine rollout.
///
/// @p x_plan has length N+1 and @p u_plan length N, both at @p lcs_for_plan's
/// resolution, as unpacked from the C3 solution.
///
/// A valid @p object_layout additionally fills in the object travel and
/// rotation columns off the same rollout states; an invalid one (the default)
/// leaves them NaN.
void ComputePdRolloutMetrics(
    const std::vector<Eigen::VectorXd>& x_plan,
    const std::vector<Eigen::VectorXd>& u_plan, const Eigen::VectorXd& Kp,
    const Eigen::VectorXd& Kd, const c3::LCS& lcs_for_plan,
    const c3::LCS& lcs_for_cost, const c3::LCSSimulateConfig& simulate_config,
    JammingMetrics* metrics,
    const ObjectStateLayout& object_layout = ObjectStateLayout{});

/// Fills in the metrics that combine a Predictor A quantity with a Predictor B
/// one -- currently just the jam index -- once both halves have been computed.
/// Leaves it NaN if the object travel it divides by was never measured.
void ComputeDerivedMetrics(JammingMetrics* metrics);

/// Runs both predictors for one solved sample, on the plan retimed to @p
/// limits.  @p c3_solution must already have been Solve()d.  Non-const because
/// C3's solution getters are non-const.
///
/// Invalid @p limits mean the EE speed limits are unconfigured, in which case
/// the plan is measured as solved.  Invalid @p u_limits mean the input bounds
/// were not supplied, in which case the saturation fractions stay NaN.  An
/// invalid @p object_layout likewise leaves the object-side columns NaN.
///
/// A non-null @p retimed_ee_plan additionally receives the EE positions of the
/// plan these metrics were measured on, one per knot -- what a ground truth sim
/// has to replay for its label to describe the same trajectory the metrics
/// scored.
JammingMetrics ComputeJammingMetrics(
    c3::C3* c3_solution, const c3::LCS& lcs_for_plan,
    const c3::LCS& lcs_for_cost, const Eigen::VectorXd& Kp,
    const Eigen::VectorXd& Kd, const JammingForceBases& bases,
    const c3::LCSSimulateConfig& simulate_config,
    const EEVelocityLimits& limits = EEVelocityLimits{},
    const UInputLimits& u_limits = UInputLimits{},
    const ObjectStateLayout& object_layout = ObjectStateLayout{},
    std::vector<Eigen::Vector3d>* retimed_ee_plan = nullptr);

/// Counts C3 solver fallbacks, per thread, while it is alive.
///
/// c3::C3 exposes no solve status at the commit dairlib pins: C3::Solve returns
/// void, the MathematicalProgramResult is discarded, and a failed QP is only
/// announced by a drake::log()->warn from C3::SetFallbackSolution.  That
/// fallback holds the current state with zero inputs, which reads as a *low*
/// predicted force -- so a sweep that ignores it would paint failed solves as
/// safe samples.  This watcher installs a log sink to catch those warnings.
///
/// Counts are bucketed by omp_get_thread_num(), and a sample's whole Solve()
/// runs on one thread, so taking the difference of that thread's count across a
/// solve attributes the failure to the right sample even under OpenMP.  The
/// sink is removed on destruction.
class C3FallbackWatcher {
 public:
  C3FallbackWatcher();
  ~C3FallbackWatcher();
  C3FallbackWatcher(const C3FallbackWatcher&) = delete;
  C3FallbackWatcher& operator=(const C3FallbackWatcher&) = delete;

  /// Fallbacks seen so far on the calling thread.
  int CountForThisThread() const;

  /// Fallbacks seen so far across all threads.
  int TotalCount() const;

 private:
  class Sink;
  std::shared_ptr<Sink> sink_;
};

/// Column names of the flattened metrics, in the order AsRow() writes them.
std::vector<std::string> JammingMetricsColumnNames();

/// The metrics as a flat row, for dumping to a np.loadtxt-friendly file.
Eigen::VectorXd JammingMetricsAsRow(const JammingMetrics& metrics);

}  // namespace systems
}  // namespace dairlib
