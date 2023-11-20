#include <math.h>
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
  double resolution = map.getResolution();

  // For now visualize this as a point cloud since PlotSurface is being flaky
  for (const auto& layer : map.getLayers()) {
    if (layers_.empty() or
        std::find(layers_.begin(), layers_.end(), layer) != layers_.end()) {

      drake::perception::PointCloud map_as_cloud(nx * ny);
      auto points = map_as_cloud.mutable_xyzs();
      MatrixXd Z = map.get(layer).cast<double>();
      int n = 0;
      for (int i = 0; i < nx; i++) {
        for (int j = 0; j < ny; j++) {
          Eigen::Array2i idx(i, j);
          grid_map::Position xy;
          map.getPosition(idx, xy);
          double z = Z(i, j);
          if (not isnan(z)) {
            points.col(n) = Eigen::Vector3f(xy(0), xy(1), z);
            ++n;
          }
        }
      }
      map_as_cloud.resize(n);
      meshcat_->SetObject("grid_map_" + layer, map_as_cloud, resolution,
                          {0.1, 0.1, 0.9, 1.0});
    }
  }
  meshcat_->Flush();
  return EventStatus::Succeeded();
}

} // dairlib
} // perception