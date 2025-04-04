#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/sampling_c3/jacktoy/diagrams/franka_visualizer_diagram.h"

namespace py = pybind11;

namespace dairlib::pydairlib{

using jacktoy::FrankaVisualizerDiagram;

PYBIND11_MODULE(diagrams, m) {
  m.doc() = "Binding visualizer utility";

  using py_rvp = py::return_value_policy;
  py::module::import("pydrake.systems.framework");

  py::class_<FrankaVisualizerDiagram, drake::systems::Diagram<double>>(
      m, "FrankaVisualizerDiagram")
      .def(py::init<>())
      .def("get_meshcat", &FrankaVisualizerDiagram::get_meshcat, py_rvp::reference_internal)
      .def("has_input_for_channel", &FrankaVisualizerDiagram::has_input_for_channel)
      .def("get_input_channels", &FrankaVisualizerDiagram::get_input_channels)
      .def("get_lcm_type", &FrankaVisualizerDiagram::get_lcm_type)
      .def("get_input_port_for_channel", &FrankaVisualizerDiagram::get_input_port_for_channel, py_rvp::reference_internal);
}
}

