#include "grid_map_visualizer.h"

namespace dairlib {
namespace perception {

using Eigen::MatrixXd;
using Eigen::RowVectorXd;
using Eigen::VectorXd;

using grid_map::GridMap;

using drake::geometry::Meshcat;
using drake::systems::Context;
using drake::systems::State;
using drake::systems::EventStatus;

GridMapVisualizer::GridMapVisualizer(
    std::shared_ptr<Meshcat> meshcat, double update_rate,
    const std::vector<std::string>& layers) : meshcat_(meshcat), layers_(layers) {

  DeclareAbstractInputPort("grid_map", drake::Value<GridMap>());
  DeclarePeriodicUnrestrictedUpdateEvent(
      1.0 / update_rate, 0, &GridMapVisualizer::UpdateVisualization);
}

drake::systems::EventStatus GridMapVisualizer::UpdateVisualization(
    const Context<double> &context, State<double> *state) const {

  const auto& grid_map = EvalAbstractInput(context, 0)->get_value<GridMap>();

  const int nx = grid_map.getSize()(0);
  const int ny = grid_map.getSize()(1);
  const double cx = grid_map.getPosition()(0);
  const double cy = grid_map.getPosition()(1);
  const double hx = grid_map.getLength()(0) / 2.0;
  const double hy = grid_map.getLength()(1) / 2.0;

  MatrixXd X = RowVectorXd::LinSpaced(nx, cx - hx, cx + hx).replicate(ny, 1);
  MatrixXd Y = VectorXd::LinSpaced(ny, cy - hy, cy + hy).replicate(1, nx);

  for (const auto& layer : grid_map.getLayers()) {
    if (layers_.empty() or
        std::find(layers_.begin(), layers_.end(), layer) != layers_.end()) {
      MatrixXd Z = grid_map.get(layer).cast<double>();
      meshcat_->PlotSurface("grid_map_" + layer, X, Y, Z);
    }
  }
  meshcat_->Flush();
  return EventStatus::Succeeded();
}

} // dairlib
} // perception