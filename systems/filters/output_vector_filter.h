#pragma once

#include "systems/framework/output_vector.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/leaf_system.h"

/// OutputVectorFilter implements a first order vector-valued linear
/// lowpass filter.  Since the control loops we use generally do not have
/// a fixed sampling rate, we implement this filter using timestamps
/// to achieve the  desired time constant / cutoff frequency

using drake::systems::Context;
using drake::systems::LeafSystem;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace dairlib::systems {

/// Filter system -
/// tau is the cutoff frequency for each index listed in filter idxs
/// n_y is the length of the base vector that will be filtered
/// filter_idxs is the indexes of the input vector to be filtered
/// Note: If filter_idxs is not supplied, then the length of tau must be n_y
class OutputVectorFilter : public LeafSystem<double> {
 public:
  // For cutoff frequency w_c (in rad/s), tau = 1/w_c,
  // For cutoff frequency f in Hz, tau = 1/(2*pi*f)
  OutputVectorFilter(const drake::multibody::MultibodyPlant<double>& plant,
                     const std::vector<double>& tau,
                     std::optional<std::vector<int>> filter_idxs);

 private:
  void CopyFilterValues(const drake::systems::Context<double>& context,
                        OutputVector<double>* y) const;

  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;

  std::vector<int> filter_idxs_;
  const int n_y_filt_;
  const std::vector<double>& tau_;
  int prev_val_idx_;
  int prev_time_idx_;
};

namespace filtering_utils {
/// Struct storing the transfer function of a second order discrete LTI filter
/// H(z) = b_[0]z^{-2} + b_[1]z^{-1} + b[2]
///        --------------------------------
///        a_[0]z^{-2} + a_[1]z^{-1} + a[2]
struct FilterSection {
  Eigen::Vector3d a_;
  Eigen::Vector3d b_;

  /// Apply the second order filter with the last element of x and y being the
  /// most recent input/output
  void ApplyFilter(Eigen::Vector3d &y, const Eigen::Vector3d &x) const {
    double bx = b_.dot(x);
    y(0) = y(1);
    y(1) = y(2);
    y(2) = (1 / a_(2)) * bx - a_.head<2>().dot(y.head<2>());
  }
  static void AddMeasurement(Eigen::Vector3d &x, double u) {
    x(0) = x(1);
    x(1) = x(2);
    x(2) = u;
  }
};

typedef std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> FilterState;

struct CascadedFilter {
  std::vector<FilterSection> sections;
  FilterState make_state() {
    return {sections.size(),
            {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()}};
  };
  double UpdateFilter(double x, FilterState &state) const {
    FilterSection::AddMeasurement(state.front().second, x);
    sections.front().ApplyFilter(state.front().first, state.front().second);
    for (int i = 1; i < sections.size(); i++) {
      FilterSection::AddMeasurement(
          state.at(i).second, state.at(i - 1).first(2));
      sections.at(i).ApplyFilter(state.at(i).first, state.at(i).second);
    }
    return state.back().first(2);
  }
};

CascadedFilter butter(int order, double w_c);
inline CascadedFilter butter(int order, double f_s, double f_c) {
  DRAKE_DEMAND(f_s > f_c);
  return butter(order, f_c / f_s);
}
}

class OutputVectorButterworthFilter : public LeafSystem<double> {
 public:
  OutputVectorButterworthFilter(
      const drake::multibody::MultibodyPlant<double>& plant,
      int order,
      double sampling_frequency,
      const std::vector<double>& w_c,
      std::optional<std::vector<int>> filter_idxs);

 private:
  drake::systems::AbstractStateIndex filter_state_idx_;
  std::vector<int> filter_idxs_;
  std::unordered_map<int, filtering_utils::CascadedFilter> index_to_filter_map_;
  void CopyFilterValues(const drake::systems::Context<double>& context,
                        OutputVector<double>* y) const;

  drake::systems::EventStatus UnrestrictedUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::State<double>* state) const;

};
}  // namespace dairlib::systems
