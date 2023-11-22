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

  const auto& map = EvalAbstractInput(context, 0)->get_value<GridMap>();

  const int nx = map.getSize()(0);
  const int ny = map.getSize()(1);
  const double cx = map.getPosition()(0);
  const double cy = map.getPosition()(1);
  const double hx = map.getLength()(0) / 2.0;
  const double hy = map.getLength()(1) / 2.0;

  // TODO (@Brian-Acosta) figure out how to efficiently do this
  MatrixXd X = MatrixXd::Zero(nx, ny);
  MatrixXd Y = MatrixXd::Zero(nx, ny);
  for (int i = 0; i < nx; i++) {
    for (int j = 0; j < ny; j++) {
      Eigen::Array2i idx(i, j);
      grid_map::Position xy;
      map.getPosition(idx, xy);
      X(i,j) = xy(0);
      Y(i,j) = xy(1);
    }
  }

  for (const auto& layer : map.getLayers()) {
    if (layers_.empty() or
        std::find(layers_.begin(), layers_.end(), layer) != layers_.end()) {

      MatrixXd Z = map.get(layer).cast<double>();
      meshcat_->PlotSurface("grid_map_" + layer, X.transpose(), Y.transpose(), Z.transpose());
    }
  }
  meshcat_->Flush();
  return EventStatus::Succeeded();
}

} // dairlib
} // perception