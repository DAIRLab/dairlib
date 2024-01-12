#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/perceptive_locomotion/diagrams/elevation_mapping_ros_diagram.h"

namespace py = pybind11;
using py_rvp = py::return_value_policy;


namespace dairlib {
namespace pydairlib {

using perceptive_locomotion::ElevationMappingRosDiagram;

PYBIND11_MODULE(ros_diagrams, m) {
  m.doc() = "Binding perceptive locomotion diagrams with ros dependencies";

  using py_rvp = py::return_value_policy;

  py::class_<ElevationMappingRosDiagram, drake::systems::Diagram<double>>(
      m, "ElevationMappingRosDiagram")
      .def(py::init<const std::string&, const std::string&>(),
          py::arg("params_yaml"), py::arg("points_topic"))
      .def("InitializeElevationMap", &ElevationMappingRosDiagram::InitializeElevationMap)
      .def("lcm", &ElevationMappingRosDiagram::lcm, py_rvp::reference_internal);

};

}
}
