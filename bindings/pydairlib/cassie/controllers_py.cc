#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/Cassie/diagrams/osc_running_controller_diagram.h"
#include "examples/Cassie/diagrams/osc_walking_controller_diagram.h"

#include "drake/multibody/plant/multibody_plant.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using examples::controllers::OSCRunningControllerDiagram;
using examples::controllers::OSCWalkingControllerDiagram;

PYBIND11_MODULE(controllers, m) {
  m.doc() = "Binding controller factories for Cassie";

  using py_rvp = py::return_value_policy;

  py::class_<OSCRunningControllerDiagram, drake::systems::Diagram<double>>(
      m, "OSCRunningControllerFactory")
      .def(py::init<drake::multibody::MultibodyPlant<double>&, const std::string&, const std::string&>(),
           py::arg("plant"), py::arg("osc_gains_filename"), py::arg("osqp_settings_filename"))
      .def("get_plant", &OSCRunningControllerDiagram::get_plant,
           py_rvp::reference_internal)
      .def("get_input_port_state",
           &OSCRunningControllerDiagram::get_input_port_state,
           py_rvp::reference_internal)
      .def("get_input_port_radio",
           &OSCRunningControllerDiagram::get_input_port_radio,
           py_rvp::reference_internal)
      .def("get_output_port_robot_input",
           &OSCRunningControllerDiagram::get_output_port_robot_input,
           py_rvp::reference_internal)
      .def("get_output_port_torque",
           &OSCRunningControllerDiagram::get_output_port_torque,
           py_rvp::reference_internal)
      .def("get_output_port_controller_failure",
           &OSCRunningControllerDiagram::get_output_port_controller_failure,
           py_rvp::reference_internal);

  py::class_<OSCWalkingControllerDiagram, drake::systems::Diagram<double>>(
      m, "OSCWalkingControllerFactory")
      .def(py::init<drake::multibody::MultibodyPlant<double>&, bool, const std::string&, const std::string&>(),
           py::arg("plant"), py::arg("has_double_stance"), py::arg("osc_gains_filename"), py::arg("osqp_settings_filename"))
      .def("get_plant", &OSCWalkingControllerDiagram::get_plant,
           py_rvp::reference_internal)
      .def("get_input_port_state",
           &OSCWalkingControllerDiagram::get_input_port_state,
           py_rvp::reference_internal)
      .def("get_input_port_radio",
           &OSCWalkingControllerDiagram::get_input_port_radio,
           py_rvp::reference_internal)
      .def("get_output_port_robot_input",
           &OSCWalkingControllerDiagram::get_output_port_robot_input,
           py_rvp::reference_internal)
      .def("get_output_port_torque",
           &OSCWalkingControllerDiagram::get_output_port_torque,
           py_rvp::reference_internal)
      .def("get_output_port_controller_failure",
           &OSCWalkingControllerDiagram::get_output_port_controller_failure,
           py_rvp::reference_internal);
}
}  // namespace pydairlib
}  // namespace dairlib