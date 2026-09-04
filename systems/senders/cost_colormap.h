#pragma once

#include <Eigen/Core>

namespace dairlib {
namespace systems {

/// Number of entries in the sampled colormap.
constexpr int kNumColormapEntries = 100;

/// Samples of matplotlib's `RdYlGn.reversed()` colormap, for coloring a scalar
/// per-sample quantity: green at the low end, yellow through the middle, red at
/// the high end.
///
/// @param t position along the colormap in [0, 1]; values outside that range
///          are clamped, so outliers saturate rather than wrapping around.
/// @return an (r, g, b) triple, each channel in [0, 255].
Eigen::Vector3i RdYlGnReversedColor(double t);

/// The raw colormap table, one row of (r, g, b) per entry.  Prefer
/// RdYlGnReversedColor(); this is exposed for callers that do their own
/// nearest-entry lookup.
const Eigen::MatrixXi& RdYlGnReversedTable();

/// The [0, 1] position of each entry in RdYlGnReversedTable(), evenly spaced.
const Eigen::VectorXf& RdYlGnReversedPositions();

}  // namespace systems
}  // namespace dairlib
