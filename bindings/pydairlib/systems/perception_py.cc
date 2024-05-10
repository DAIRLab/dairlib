#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/perception/camera_utils.h"
#include "systems/perception/grid_map_visualizer.h"
#include "systems/perception/grid_map_lcm_systems.h"
#include "systems/perception/ethz_plane_segmentatation/plane_segmentation_system.h"

namespace py = pybind11;
using py_rvp = py::return_value_policy;


namespace dairlib::pydairlib {

using perception::GridMapVisualizer;
using perception::GridMapSender;
using perception::GridMapReceiver;
using perception::PlaneSegmentationSystem;

PYBIND11_MODULE(perception, m) {
  m.doc() = "Binding camera utilities";

  py::class_<GridMapVisualizer, drake::systems::LeafSystem<double>>(
      m, "GridMapVisualizer")
      .def(py::init<std::shared_ptr<drake::geometry::Meshcat>, double,
          const std::vector<std::string>&>(),
          py::arg("meshcat"), py::arg("update_rate"), py::arg("layers"));

  py::class_<PlaneSegmentationSystem, drake::systems::LeafSystem<double>>(
      m, "PlaneSegmentationSystem")
      .def(py::init<std::string>(), py::arg("params_yaml"));

  py::class_<GridMapSender, drake::systems::LeafSystem<double>>(
      m, "GridMapSender")
      .def(py::init<>());

  py::class_<GridMapReceiver, drake::systems::LeafSystem<double>>(
      m, "GridMapReceiver")
      .def(py::init<>());

  m.def("ReadCameraPoseFromYaml", &dairlib::camera::ReadCameraPoseFromYaml, py::arg("fname"));
}
}