#include "examples/sampling_c3/jamming_metrics.h"

#include <algorithm>
#include <mutex>
#include <string>
#include <unordered_map>

#include <omp.h>
#include <spdlog/sinks/base_sink.h>

#include "core/traj_eval.h"
#include "examples/sampling_c3/ee_plan_retiming.h"

#include "drake/common/drake_throw.h"
#include "drake/common/text_logging.h"

namespace dairlib {
namespace systems {

using c3::LCS;
using c3::LCSSimulateConfig;
using c3::multibody::LCSContactDescription;
using c3::traj_eval::TrajectoryEvaluator;
using Eigen::Vector3d;
using Eigen::VectorXd;
using std::vector;

namespace {

// Accumulates the peak of |u| over a trajectory into the three ways we slice
// an EE force: full norm, horizontal norm, and vertical magnitude.
void AccumulateInputPeaks(const vector<VectorXd>& u_traj, double* max_norm,
                          double* max_xy, double* max_z) {
  for (const VectorXd& u : u_traj) {
    DRAKE_THROW_UNLESS(u.size() >= 3);
    *max_norm = std::max(*max_norm, u.head(3).norm());
    *max_xy = std::max(*max_xy, u.head(2).norm());
    *max_z = std::max(*max_z, std::abs(u(2)));
  }
}

// The object's orientation at one state, as a unit quaternion.  The LCS is a
// linearization and integrates the quaternion as four independent coordinates,
// so a rollout state's quaternion drifts off the unit sphere and has to be
// renormalized before it means a rotation.  A state whose quaternion has
// collapsed to zero -- which a badly conditioned rollout can produce -- comes
// back as the identity rather than as a NaN rotation that would poison the max.
Eigen::Quaterniond UnitQuaternionAt(const VectorXd& x,
                                    const ObjectStateLayout& layout) {
  const Eigen::Vector4d wxyz = x.segment<4>(layout.quaternion_offset);
  if (wxyz.norm() < 1e-12) {
    return Eigen::Quaterniond::Identity();
  }
  return Eigen::Quaterniond(wxyz(0), wxyz(1), wxyz(2), wxyz(3)).normalized();
}

// How close to its bound an input has to be to count as pinned there.
// Relative, and generous enough to survive the solver tolerance the QP leaves
// behind -- and the fact that the C3+ projection imposes no u bounds at all, so
// a u read out of the solution z can land marginally outside them.
constexpr double kUSaturationRelativeTolerance = 1e-3;

}  // namespace

UInputLimits MakeUInputLimits(const vector<double>& u_horizontal_limits,
                              const vector<double>& u_vertical_limits) {
  if (u_horizontal_limits.size() < 2 || u_vertical_limits.size() < 2) {
    return UInputLimits{};
  }
  UInputLimits limits;
  limits.u_xy_max = std::min(std::abs(u_horizontal_limits[0]),
                             std::abs(u_horizontal_limits[1]));
  limits.u_z_max =
      std::min(std::abs(u_vertical_limits[0]), std::abs(u_vertical_limits[1]));
  return limits.IsValid() ? limits : UInputLimits{};
}

ContactForceBasis MakeContactForceBasis(
    const vector<LCSContactDescription>& contact_descriptions, int first_entry,
    int num_entries) {
  DRAKE_THROW_UNLESS(first_entry >= 0);
  DRAKE_THROW_UNLESS(num_entries >= 0);
  DRAKE_THROW_UNLESS(first_entry + num_entries <=
                     static_cast<int>(contact_descriptions.size()));

  ContactForceBasis basis;
  for (int i = first_entry; i < first_entry + num_entries; ++i) {
    if (contact_descriptions[i].is_slack) {
      continue;
    }
    basis.lambda_indices.push_back(i);
    basis.force_bases.push_back(contact_descriptions[i].force_basis);
  }
  return basis;
}

ContactForceBasis MakeEEContactForceBasis(
    const vector<LCSContactDescription>& contact_descriptions,
    int num_ee_lambda_entries) {
  // The contact pairs are resolved in group order (EE-ground, EE-object,
  // object-ground, object-object, object-wall), so the EE's contacts are the
  // leading ones and own the leading entries of lambda.
  return MakeContactForceBasis(contact_descriptions, /*first_entry=*/0,
                               num_ee_lambda_entries);
}

ObjectStateLayout MakeObjectStateLayout(int object_index) {
  DRAKE_THROW_UNLESS(object_index >= 0);
  // Three EE prismatic positions, then 7 per object: quaternion, then position.
  const int base = 3 + 7 * object_index;
  return ObjectStateLayout{base, base + 4};
}

double ContactForceMagnitude(const VectorXd& lambda,
                             const ContactForceBasis& basis) {
  // Each lambda entry contributes force_basis * lambda along its own basis
  // vector; the net Cartesian force is their sum.  Matches the idiom in c3's
  // systems/lcmt_generators/contact_force_generator.cc.
  Vector3d force = Vector3d::Zero();
  for (size_t i = 0; i < basis.lambda_indices.size(); ++i) {
    const int index = basis.lambda_indices[i];
    DRAKE_THROW_UNLESS(index < lambda.size());
    force += basis.force_bases[i] * lambda(index);
  }
  return force.norm();
}

void UnpackC3Plan(const vector<VectorXd>& z_plan, int n_x, int n_lambda,
                  int n_u, vector<VectorXd>* x_plan,
                  vector<VectorXd>* lambda_plan, vector<VectorXd>* u_plan) {
  DRAKE_THROW_UNLESS(!z_plan.empty());
  DRAKE_THROW_UNLESS(x_plan != nullptr && lambda_plan != nullptr &&
                     u_plan != nullptr);

  x_plan->resize(z_plan.size());
  lambda_plan->resize(z_plan.size());
  u_plan->resize(z_plan.size());
  for (size_t i = 0; i < z_plan.size(); ++i) {
    DRAKE_THROW_UNLESS(z_plan[i].size() >= n_x + n_lambda + n_u);
    x_plan->at(i) = z_plan[i].segment(0, n_x);
    lambda_plan->at(i) = z_plan[i].segment(n_x, n_lambda);
    u_plan->at(i) = z_plan[i].segment(n_x + n_lambda, n_u);
  }
}

void ComputeC3PlanMetrics(const vector<VectorXd>& u_plan,
                          const vector<VectorXd>& lambda_plan,
                          const JammingForceBases& bases,
                          JammingMetrics* metrics,
                          const UInputLimits& u_limits) {
  DRAKE_THROW_UNLESS(metrics != nullptr);
  DRAKE_THROW_UNLESS(!u_plan.empty());
  DRAKE_THROW_UNLESS(u_plan.size() == lambda_plan.size());

  const bool count_saturation = u_limits.IsValid();
  const double xy_threshold =
      (1.0 - kUSaturationRelativeTolerance) * u_limits.u_xy_max;
  const double z_threshold =
      (1.0 - kUSaturationRelativeTolerance) * u_limits.u_z_max;
  int num_xy_at_limit = 0;
  int num_z_at_limit = 0;

  for (size_t i = 0; i < u_plan.size(); ++i) {
    const VectorXd& u = u_plan[i];
    DRAKE_THROW_UNLESS(u.size() >= 3);
    const double u_xy = u.head(2).norm();
    const double u_z = std::abs(u(2));
    metrics->max_u_norm_c3 = std::max(metrics->max_u_norm_c3, u.head(3).norm());
    metrics->max_u_xy_c3 = std::max(metrics->max_u_xy_c3, u_xy);
    metrics->max_u_z_c3 = std::max(metrics->max_u_z_c3, u_z);
    metrics->max_ee_contact_force_c3 =
        std::max(metrics->max_ee_contact_force_c3,
                 ContactForceMagnitude(lambda_plan[i], bases.ee));
    metrics->max_object_ground_force_c3 =
        std::max(metrics->max_object_ground_force_c3,
                 ContactForceMagnitude(lambda_plan[i], bases.object_ground));

    // Counted with >=, so a knot that overshoots its bound still reads as
    // saturated rather than falling through as if it were slack.
    if (count_saturation) {
      if (u_xy >= xy_threshold) ++num_xy_at_limit;
      if (u_z >= z_threshold) ++num_z_at_limit;
    }
  }

  if (count_saturation) {
    const double num_knots = static_cast<double>(u_plan.size());
    metrics->frac_knots_u_xy_at_limit = num_xy_at_limit / num_knots;
    metrics->frac_knots_u_z_at_limit = num_z_at_limit / num_knots;

    // A solve that produced nothing to execute leaves the whole horizon
    // commanding a small fraction of what it was allowed to.  Measured against
    // the budget rather than an absolute force so it travels to demos with a
    // different input box.
    const double u_budget =
        Eigen::Vector2d(u_limits.u_xy_max, u_limits.u_z_max).norm();
    metrics->no_op_plan =
        metrics->max_u_norm_c3 < kNoOpInputFraction * u_budget ? 1.0 : 0.0;
  }
}

void ComputePdRolloutMetrics(const vector<VectorXd>& x_plan,
                             const vector<VectorXd>& u_plan, const VectorXd& Kp,
                             const VectorXd& Kd, const LCS& lcs_for_plan,
                             const LCS& lcs_for_cost,
                             const LCSSimulateConfig& simulate_config,
                             JammingMetrics* metrics,
                             const ObjectStateLayout& object_layout) {
  DRAKE_THROW_UNLESS(metrics != nullptr);
  DRAKE_THROW_UNLESS(x_plan.size() == u_plan.size() + 1);

  const int upsample_rate =
      TrajectoryEvaluator::CheckCoarseAndFineLCSCompatibility(lcs_for_plan,
                                                              lcs_for_cost);

  // Do the zero-order hold ourselves and simulate against the fine LCS
  // directly.  The coarse/fine overload downsamples its returned input
  // trajectory before handing it back, which would drop exactly the peaks
  // between planning knots that this metric is looking for.
  auto [x_plan_fine, u_plan_fine] =
      TrajectoryEvaluator::ZeroOrderHoldTrajectories(x_plan, u_plan,
                                                     upsample_rate);
  auto [x_sim_fine, u_sim_fine] = TrajectoryEvaluator::SimulatePDControlWithLCS(
      x_plan_fine, u_plan_fine, Kp, Kd, lcs_for_cost,
      /*use_feedforward=*/true, simulate_config);

  AccumulateInputPeaks(u_sim_fine, &metrics->max_u_norm_pd,
                       &metrics->max_u_xy_pd, &metrics->max_u_z_pd);

  // The EE tracking error of that same rollout, reported with no gain applied
  // so its magnitude is meaningful in meters regardless of how Kp is tuned.
  for (size_t i = 0; i < u_sim_fine.size(); ++i) {
    const Vector3d delta = x_plan_fine[i].head(3) - x_sim_fine[i].head(3);
    metrics->max_delta_x_ee_norm =
        std::max(metrics->max_delta_x_ee_norm, delta.norm());
    metrics->max_delta_x_ee_xy =
        std::max(metrics->max_delta_x_ee_xy, delta.head(2).norm());
    metrics->max_delta_x_ee_z =
        std::max(metrics->max_delta_x_ee_z, std::abs(delta(2)));
  }

  // Whether the rollout moves the object at all.  Measured on the simulated
  // states rather than the planned ones: the question is what the plan is
  // predicted to achieve, not what it asked for.
  if (object_layout.IsValid()) {
    DRAKE_THROW_UNLESS(x_sim_fine.front().size() >=
                       object_layout.position_offset + 3);
    const Vector3d start_position =
        x_sim_fine.front().segment<3>(object_layout.position_offset);
    const Eigen::Quaterniond start_orientation =
        UnitQuaternionAt(x_sim_fine.front(), object_layout);

    double max_travel = 0.0;
    double max_rotation = 0.0;
    for (const VectorXd& x : x_sim_fine) {
      const Vector3d position = x.segment<3>(object_layout.position_offset);
      max_travel = std::max(max_travel, (position - start_position).norm());
      // angularDistance is the angle of the relative rotation about whatever
      // axis it happens to be, which is the direction-free measure wanted here.
      max_rotation =
          std::max(max_rotation, start_orientation.angularDistance(
                                     UnitQuaternionAt(x, object_layout)));
    }
    metrics->object_travel_in_rollout = max_travel;
    metrics->object_rotation_in_rollout = max_rotation;
  }

  // The same rollout as the controller performs, whose downsampled inputs hide
  // some of the peak.  Kept for comparison against max_u_norm_pd.
  auto [x_sim_coarse, u_sim_coarse] =
      TrajectoryEvaluator::SimulatePDControlWithLCS(
          x_plan, u_plan, Kp, Kd, lcs_for_plan, lcs_for_cost,
          /*use_feedforward=*/true, simulate_config);
  double unused_xy = 0.0;
  double unused_z = 0.0;
  AccumulateInputPeaks(u_sim_coarse, &metrics->max_u_norm_pd_coarse, &unused_xy,
                       &unused_z);
}

void ComputeDerivedMetrics(JammingMetrics* metrics) {
  DRAKE_THROW_UNLESS(metrics != nullptr);
  if (std::isnan(metrics->object_travel_in_rollout)) {
    return;
  }
  // Floored rather than guarded: a plan that pushes hard and moves nothing is
  // the jam this index is for, so it should read as a large number, not drop
  // out as undefined.  Callers mask on no_op_plan to drop the samples whose
  // solve produced no plan at all, which is a separate question.
  metrics->object_ground_force_per_travel =
      metrics->max_object_ground_force_c3 /
      std::max(metrics->object_travel_in_rollout, kMinObjectTravelForJamIndex);
}

JammingMetrics ComputeJammingMetrics(
    c3::C3* c3_solution, const LCS& lcs_for_plan, const LCS& lcs_for_cost,
    const VectorXd& Kp, const VectorXd& Kd, const JammingForceBases& bases,
    const LCSSimulateConfig& simulate_config, const EEVelocityLimits& limits,
    const UInputLimits& u_limits, const ObjectStateLayout& object_layout,
    vector<Vector3d>* retimed_ee_plan) {
  DRAKE_THROW_UNLESS(c3_solution != nullptr);

  const int N = lcs_for_plan.N();
  const vector<VectorXd> z_plan = c3_solution->GetFullSolution();
  DRAKE_THROW_UNLESS(static_cast<int>(z_plan.size()) == N);

  vector<VectorXd> x_plan;
  vector<VectorXd> u_plan;
  vector<VectorXd> lambda_plan;
  UnpackC3Plan(z_plan, lcs_for_plan.num_states(), lcs_for_plan.num_lambdas(),
               lcs_for_plan.num_inputs(), &x_plan, &lambda_plan, &u_plan);

  // z holds only N states; the rollout needs N+1, so roll the last knot forward
  // through the LCS the same way CalcCost does.  This uses the plan's own
  // dynamics, so it happens before the retiming rewrites the path.
  x_plan.push_back(lcs_for_plan.A().back() * x_plan.back() +
                   lcs_for_plan.B().back() * u_plan.back() +
                   lcs_for_plan.D().back() * lambda_plan.back() +
                   lcs_for_plan.d().back());

  // Measure the plan the execution path would publish and the retimed cost
  // would score.  Unconfigured limits leave nothing to retime, matching the
  // no-op RetimeAndResampleC3PlanForCost takes in the same situation.
  if (limits.IsValid()) {
    RetimeAndResampleEEPlan(lcs_for_plan.dt(), limits.v_xy_max, limits.v_z_max,
                            limits.ee_velocity_offset, &x_plan, &u_plan,
                            &lambda_plan);
  }

  // Handed back after the retiming above, so a sim replaying it drives the EE
  // along the same path the metrics below are measured on.
  if (retimed_ee_plan != nullptr) {
    retimed_ee_plan->clear();
    retimed_ee_plan->reserve(x_plan.size());
    for (const VectorXd& x : x_plan) {
      retimed_ee_plan->push_back(x.head(3));
    }
  }

  JammingMetrics metrics;
  ComputeC3PlanMetrics(u_plan, lambda_plan, bases, &metrics, u_limits);
  ComputePdRolloutMetrics(x_plan, u_plan, Kp, Kd, lcs_for_plan, lcs_for_cost,
                          simulate_config, &metrics, object_layout);
  ComputeDerivedMetrics(&metrics);
  return metrics;
}

// The warning C3::SetFallbackSolution emits when a QP solve fails.  Matching on
// the function name rather than the full sentence keeps this robust to wording
// changes upstream.
constexpr char kFallbackMarker[] = "SetFallbackSolution";

class C3FallbackWatcher::Sink : public spdlog::sinks::base_sink<std::mutex> {
 public:
  int CountForThread(int thread_id) const {
    std::lock_guard<std::mutex> guard(counts_mutex_);
    auto it = counts_.find(thread_id);
    return it == counts_.end() ? 0 : it->second;
  }

  int TotalCount() const {
    std::lock_guard<std::mutex> guard(counts_mutex_);
    int total = 0;
    for (const auto& [thread_id, count] : counts_) {
      total += count;
    }
    return total;
  }

 protected:
  void sink_it_(const spdlog::details::log_msg& msg) override {
    const std::string payload(msg.payload.data(), msg.payload.size());
    if (payload.find(kFallbackMarker) == std::string::npos) {
      return;
    }
    std::lock_guard<std::mutex> guard(counts_mutex_);
    ++counts_[omp_get_thread_num()];
  }

  void flush_() override {}

 private:
  mutable std::mutex counts_mutex_;
  std::unordered_map<int, int> counts_;
};

C3FallbackWatcher::C3FallbackWatcher() : sink_(std::make_shared<Sink>()) {
  drake::log()->sinks().push_back(sink_);
}

C3FallbackWatcher::~C3FallbackWatcher() {
  auto& sinks = drake::log()->sinks();
  sinks.erase(std::remove(sinks.begin(), sinks.end(), sink_), sinks.end());
}

int C3FallbackWatcher::CountForThisThread() const {
  return sink_->CountForThread(omp_get_thread_num());
}

int C3FallbackWatcher::TotalCount() const { return sink_->TotalCount(); }

vector<std::string> JammingMetricsColumnNames() {
  return {"max_u_norm_c3",
          "max_u_xy_c3",
          "max_u_z_c3",
          "max_ee_contact_force_c3",
          "max_u_norm_pd",
          "max_u_xy_pd",
          "max_u_z_pd",
          "max_u_norm_pd_coarse",
          "max_delta_x_ee_norm",
          "max_delta_x_ee_xy",
          "max_delta_x_ee_z",
          "max_ee_contact_force_pd",
          "frac_knots_u_xy_at_limit",
          "frac_knots_u_z_at_limit",
          "object_travel_in_rollout",
          "object_rotation_in_rollout",
          "no_op_plan",
          "max_object_ground_force_c3",
          "object_ground_force_per_travel"};
}

VectorXd JammingMetricsAsRow(const JammingMetrics& metrics) {
  VectorXd row(19);
  row << metrics.max_u_norm_c3, metrics.max_u_xy_c3, metrics.max_u_z_c3,
      metrics.max_ee_contact_force_c3, metrics.max_u_norm_pd,
      metrics.max_u_xy_pd, metrics.max_u_z_pd, metrics.max_u_norm_pd_coarse,
      metrics.max_delta_x_ee_norm, metrics.max_delta_x_ee_xy,
      metrics.max_delta_x_ee_z, metrics.max_ee_contact_force_pd,
      metrics.frac_knots_u_xy_at_limit, metrics.frac_knots_u_z_at_limit,
      metrics.object_travel_in_rollout, metrics.object_rotation_in_rollout,
      metrics.no_op_plan, metrics.max_object_ground_force_c3,
      metrics.object_ground_force_per_travel;
  DRAKE_THROW_UNLESS(row.size() ==
                     static_cast<int>(JammingMetricsColumnNames().size()));
  return row;
}

}  // namespace systems
}  // namespace dairlib
