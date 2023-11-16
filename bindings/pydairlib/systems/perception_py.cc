#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/perception/camera_utils.h"
#include "systems/perception/grid_map_visualizer.h"

namespace py = pybind11;
using py_rvp = py::return_value_policy;


namespace dairlib::pydairlib {

  using perception::GridMapVisualizer;

  PYBIND11_MODULE(perception, m) {
  m.doc() = "Binding camera utilities";

  py::class_<GridMapVisualizer, drake::systems::LeafSystem<double>>(
      m, "GridMapVisualizer")
      .def(py::init<std::shared_ptr<drake::geometry::Meshcat>, double,
          const std::vector<std::string>&>(),
          py::arg("meshcat"), py::arg("update_rate"), py::arg("layers"));

  m.def("ReadCameraPoseFromYaml", &dairlib::camera::ReadCameraPoseFromYaml, py::arg("fname"));
}
}