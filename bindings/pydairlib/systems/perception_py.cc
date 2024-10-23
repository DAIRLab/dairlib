#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/perception/camera_utils.h"
#include "systems/perception/grid_map_visualizer.h"
#include "systems/perception/grid_map_lcm_systems.h"
#include "systems/perception/terrain_segmentation_monitor.h"
#include "systems/perception/ethz_plane_segmentation/plane_segmentation_system.h"
#include "systems/perception/ori_drs_plane_segmentation/plane_seg_system.h"

namespace py = pybind11;
using py_rvp = py::return_value_policy;


namespace dairlib::pydairlib {

using perception::GridMapVisualizer;
using perception::GridMapSender;
using perception::GridMapReceiver;
using perception::PlaneSegmentationSystem;
using perception::PlaneSegSystem;
using perception::TerrainSegmentationMonitor;
using perception::terrain_segmentation_reset_params;

PYBIND11_MODULE(perception, m) {
  m.doc() = "Binding camera utilities";
  py::module::import("pydrake.systems.framework");

  py::class_<GridMapVisualizer, drake::systems::LeafSystem<double>>(
      m, "GridMapVisualizer")
      .def(py::init<std::shared_ptr<drake::geometry::Meshcat>, double,
          const std::vector<std::string>&>(),
          py::arg("meshcat"), py::arg("update_rate"), py::arg("layers"));

  py::class_<PlaneSegmentationSystem, drake::systems::LeafSystem<double>>(
      m, "PlaneSegmentationSystem")
      .def(py::init<std::string>(), py::arg("params_yaml"));

  py::class_<PlaneSegSystem, drake::systems::LeafSystem<double>>(
      m, "PlaneSegSystem")
      .def(py::init<std::string>(), py::arg("layer"));

  py::class_<GridMapSender, drake::systems::LeafSystem<double>>(
      m, "GridMapSender")
      .def(py::init<>());

  py::class_<GridMapReceiver, drake::systems::LeafSystem<double>>(
      m, "GridMapReceiver")
      .def(py::init<>());

  py::class_<TerrainSegmentationMonitor, drake::systems::LeafSystem<double>>(
      m, "TerrainSegmentationMonitor")
      .def(py::init<terrain_segmentation_reset_params>(), py::arg("params"))
      .def("Reset", &TerrainSegmentationMonitor::Reset)
      .def("NeedsIoUReset", &TerrainSegmentationMonitor::NeedsIoUReset)
      .def("NeedsMinValidAreaReset", &TerrainSegmentationMonitor::NeedsMinValidAreaReset)
      .def("GetMapForReInitialization", &TerrainSegmentationMonitor::GetMapForReInitialization);

  py::class_<terrain_segmentation_reset_params>(
      m, "terrain_segmentation_reset_params")
      .def(py::init<double, double, double, size_t>(),
          py::arg("update_period"), py::arg("iou_threshold"),
          py::arg("area_threshold"), py::arg("lookback_size"))
      .def_readwrite("update_period", &terrain_segmentation_reset_params::update_period)
      .def_readwrite("iou_threshold", &terrain_segmentation_reset_params::iou_threshold)
      .def_readwrite("area_threshold", &terrain_segmentation_reset_params::area_threshold)
      .def_readwrite("lookback_size", &terrain_segmentation_reset_params::lookback_size);

  m.def("ReadCameraPoseFromYaml", &dairlib::camera::ReadCameraPoseFromYaml, py::arg("fname"));
}
}