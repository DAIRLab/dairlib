#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/plant_visualizer.h"

#include "drake/multibody/plant/multibody_plant.h"

namespace py = pybind11;
using py_rvp = py::return_value_policy;

namespace dairlib {
namespace pydairlib {

using systems::PlantVisualizer;

PYBIND11_MODULE(plant_visualizer, m) {
  m.doc() = "Binding plant visualizer utility";

  py::class_<PlantVisualizer, drake::systems::Diagram<double>>(
      m, "PlantVisualizer")
      .def(py::init<const std::string&>(), py::arg("urdf"))
      .def("get_meshcat", &PlantVisualizer::get_meshcat, py_rvp::reference_internal)
      .def("get_plant", &PlantVisualizer::get_plant, py_rvp::reference_internal);
}

}  // namespace pydairlib
}  // namespace dairlib
