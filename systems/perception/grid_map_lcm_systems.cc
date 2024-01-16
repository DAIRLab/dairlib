#include "grid_map_lcm_systems.h"

namespace dairlib {
namespace perception {

using Eigen::Map;
using Eigen::RowVectorXf;

using grid_map::GridMap;
using grid_map::Length;
using grid_map::Position;

lcmt_grid_map GridMapToLcm(const GridMap& grid_map,
                           const std::vector<std::string>& layers) {

  lcmt_grid_map message;

  auto& info = message.info;

  info.utime = grid_map.getTimestamp() / 1e3; // nanoseconds to microseconds
  info.length_x = grid_map.getLength().x();
  info.length_y = grid_map.getLength().y();
  info.resolution = grid_map.getResolution();
  info.parent_frame = grid_map.getFrameId();

  memcpy(info.position, grid_map.getPosition().data(), 2 * sizeof(double));

  message.layers.clear();
  message.layer_names.clear();
  message.num_layers = layers.size();

  for (const auto& layer_name: layers) {
    lcmt_grid_map_layer layer;

    message.layer_names.push_back(layer_name);
    layer.name = layer_name;
    layer.rows = grid_map.getSize()(0);
    layer.cols = grid_map.getSize()(1);
    layer.data.clear();

    const auto& data = grid_map.get(layer_name);
    for (int i = 0; i < layer.rows; ++i) {
      layer.data.push_back(std::vector<float>(
              data.row(i).data(), data.row(i).data() + layer.cols
      ));
    }
    message.layers.push_back(layer);
  }

  message.outer_start_index = grid_map.getStartIndex()(0);
  message.inner_start_index = grid_map.getStartIndex()(1);

  return message;
}

GridMap LcmToGridMap(const lcmt_grid_map& message) {

  GridMap grid_map(message.layer_names);

  DRAKE_DEMAND(message.layer_names.size() == message.layers.size());

  grid_map.setTimestamp(1e3 * message.info.utime);
  grid_map.setFrameId(message.info.parent_frame);
  grid_map.setGeometry(
      Length(message.info.length_x, message.info.length_y),
      message.info.resolution,
      Position(message.info.position)
  );

  for (const auto& layer: message.layers) {
    Eigen::MatrixXf data(layer.rows, layer.cols);
    for (Eigen::Index i = 0; i < data.rows(); ++i) {
      data.row(i) = Map<const RowVectorXf>(layer.data.at(i).data(), layer.cols);
    }
    grid_map.get(layer.name) = data;
  }

  grid_map.setStartIndex(
      grid_map::Index(message.outer_start_index, message.inner_start_index));

  return grid_map;
}

}
}