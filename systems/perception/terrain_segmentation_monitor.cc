#include "terrain_segmentation_monitor.h"
#include "common/time_series_buffer.h"
#include "common/eigen_utils.h"

namespace dairlib {
namespace perception {

using drake::Value;
using drake::systems::State;
using drake::systems::Context;
using drake::systems::EventStatus;

using grid_map::GridMap;

using BufferType = std::shared_ptr<
    TimeSeriesBuffer<GridMap, TerrainSegmentationMonitor::kMaxBufferLen>>;

TerrainSegmentationMonitor::TerrainSegmentationMonitor(terrain_segmentation_reset_params params) {
  DRAKE_DEMAND(params.lookback_size <= kMaxBufferLen);
  DRAKE_DEMAND(params.lookback_size > 0);
  DRAKE_DEMAND(params.iou_threshold >= 0 and params.iou_threshold <= 1);
  DRAKE_DEMAND(params.area_threshold >= 0 and params.area_threshold <= 1);

  iou_threshold_ = params.iou_threshold;
  area_threshold = params.area_threshold;

  lookback_ = params.update_period * params.lookback_size;

  map_history_index_ = DeclareAbstractState(Value<BufferType>{nullptr});

  DeclarePeriodicUnrestrictedUpdateEvent(
      params.update_period, 0, &TerrainSegmentationMonitor::DiscreteVariableUpdate);

  input_port_grid_map_ = DeclareAbstractInputPort(
      "grid_map", Value<GridMap>()).get_index();
}

grid_map::GridMap TerrainSegmentationMonitor::GetMapForReInitialization(
    const drake::systems::Context<double> &context) const {
  double back_time = std::max(0.0, context.get_time() - lookback_);
  const auto& buf_ptr =
      context.get_abstract_state<BufferType>(map_history_index_);
  return buf_ptr->get(1e6 * back_time);
}

bool TerrainSegmentationMonitor::NeedsIoUReset(const Context<double> &context) const {
  const auto& buf_ptr =
      context.get_abstract_state<BufferType>(map_history_index_);

  if (not buf_ptr->full())  {
    return false;
  }

  double curr_time = context.get_time();
  double back_time = std::max(0.0, curr_time - lookback_);
  const auto& recent_map = buf_ptr->get(1e6 * curr_time);
  const auto& old_map = buf_ptr->get(1e6 * back_time);

  double iou = MatrixIoU(recent_map.get("segmentation"),
                         old_map.get("segmentation"));
  return iou < iou_threshold_;
}

bool TerrainSegmentationMonitor::NeedsMinValidAreaReset(
    const Context<double> &context) const {
  const auto& buf_ptr =
      context.get_abstract_state<BufferType>(map_history_index_);

  if (not buf_ptr->full())  {
    return false;
  }
  const auto& recent_map = buf_ptr->get(1e6 * context.get_time());
  const Eigen::MatrixXf& seg = recent_map.get("segmentation");
  double nan_count = seg.array().isNaN().count();
  double area = seg.cols() * seg.cols();
  return (nan_count / area) < (1.0 - area_threshold);
}

drake::systems::EventStatus TerrainSegmentationMonitor::DiscreteVariableUpdate(
    const Context<double> &context, State<double> *state) const {
  const auto& grid_map_in = EvalAbstractInput(
     context, input_port_grid_map_)->get_value<GridMap>();

  auto& buffer =
      state->get_mutable_abstract_state<BufferType>(map_history_index_);

  buffer->put(1e6 * context.get_time(), grid_map_in);

  return EventStatus::Succeeded();
}

void TerrainSegmentationMonitor::SetDefaultState(
    const Context<double>& context, State<double>* state) const {
  auto& buf_ptr =
      state->get_mutable_abstract_state<BufferType>(map_history_index_);
  buf_ptr = std::make_shared<TimeSeriesBuffer<GridMap, kMaxBufferLen>>();
  buf_ptr->reset();
}

drake::systems::EventStatus TerrainSegmentationMonitor::Initialize(
    const Context<double>&, State<double> *state) const {
  auto& buf_ptr =
      state->get_mutable_abstract_state<BufferType>(map_history_index_);
  DRAKE_DEMAND(buf_ptr != nullptr);
  buf_ptr->reset();
  return EventStatus::Succeeded();
}

}
}