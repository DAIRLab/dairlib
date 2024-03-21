#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/franka/diagrams/franka_c3_controller_diagram.h"
#include "examples/franka/diagrams/franka_osc_controller_diagram.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using examples::controllers::FrankaC3ControllerDiagram;
using examples::controllers::FrankaOSCControllerDiagram;

PYBIND11_MODULE(controllers, m) {
  m.doc() = "Binding controller factories for plate balancing demo";

  using py_rvp = py::return_value_policy;

  py::class_<FrankaOSCControllerDiagram, drake::systems::Diagram<double>>(
      m, "FrankaOSCControllerDiagram")
      .def(py::init<const std::string&, const std::string&,
                    drake::lcm::DrakeLcm*>(),
           py::arg("controller_settings"), py::arg("lcm_channels"),
           py::arg("lcm"))
      .def("get_input_port_robot_state",
           &FrankaOSCControllerDiagram::get_input_port_robot_state,
           py_rvp::reference_internal)
      .def("get_input_port_end_effector_position",
           &FrankaOSCControllerDiagram::get_input_port_end_effector_position,
           py_rvp::reference_internal)
      .def("get_input_port_end_effector_orientation",
           &FrankaOSCControllerDiagram::get_input_port_end_effector_orientation,
           py_rvp::reference_internal)
      .def("get_input_port_end_effector_force",
           &FrankaOSCControllerDiagram::get_input_port_end_effector_force,
           py_rvp::reference_internal)
      .def("get_input_port_radio",
           &FrankaOSCControllerDiagram::get_input_port_radio,
           py_rvp::reference_internal)
      .def("get_output_port_robot_input",
           &FrankaOSCControllerDiagram::get_output_port_robot_input,
           py_rvp::reference_internal);

  py::class_<FrankaC3ControllerDiagram, drake::systems::Diagram<double>>(
      m, "FrankaC3ControllerDiagram")
      .def(py::init<const std::string&, const std::string&,
                    drake::lcm::DrakeLcm*>(),
           py::arg("controller_settings"), py::arg("lcm_channels"),
           py::arg("lcm"))
      .def("get_input_port_robot_state",
           &FrankaC3ControllerDiagram::get_input_port_robot_state,
           py_rvp::reference_internal)
      .def("get_input_port_object_state",
           &FrankaC3ControllerDiagram::get_input_port_object_state,
           py_rvp::reference_internal)
      .def("get_input_port_radio",
           &FrankaC3ControllerDiagram::get_input_port_radio,
           py_rvp::reference_internal)
      .def("get_output_port_mpc_plan",
           &FrankaC3ControllerDiagram::get_output_port_mpc_plan,
           py_rvp::reference_internal);
}
}  // namespace pydairlib
}  // namespace dairlib