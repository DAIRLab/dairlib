#include <iostream>
#include "grid_map_visualizer.h"

#include "drake/perception/point_cloud.h"

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
    std::shared_ptr<Meshcat> meshcat, double update_period_sec,
    std::vector<std::string> layers) : meshcat_(meshcat), layers_(layers) {

  DeclareAbstractInputPort("grid_map", drake::Value<GridMap>());
  DeclarePeriodicUnrestrictedUpdateEvent(
      update_period_sec, 0, &GridMapVisualizer::UpdateVisualization);
}

drake::systems::EventStatus GridMapVisualizer::UpdateVisualization(
    const Context<double> &context, State<double> *state) const {

  auto map = EvalAbstractInput(context, 0)->get_value<GridMap>();

  if (map.getSize()(0) <= 0 or map.getSize()(1) <= 0) {
    return EventStatus::Succeeded();
  }

  map.convertToDefaultStartIndex();
  grid_map::Position pos;
  auto length = map.getSize();

  map.getPosition(grid_map::Index(0,0), pos);
  double xmax = pos.x();
  double ymax = pos.y();

  map.getPosition(grid_map::Index(length.x() - 1, length.y() - 1), pos);
  double xmin = pos.x();
  double ymin = pos.y();

  MatrixXd X = VectorXd::LinSpaced(length.x(), xmax, xmin).replicate(1, length.y());
  MatrixXd Y = RowVectorXd::LinSpaced(length.y(), ymax, ymin).replicate(length.x(), 1);

  for (const auto& layer : map.getLayers()) {
    if (layers_.empty() or
        std::find(layers_.begin(), layers_.end(), layer) != layers_.end()) {

      const MatrixXd& Z = map.get(layer).cast<double>();

      meshcat_->PlotSurface("grid_map_" + layer, X, Y, Z);
    }
  }
//  meshcat_->Flush();
  return EventStatus::Succeeded();
}

} // dairlib
} // perception