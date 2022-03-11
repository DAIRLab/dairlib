#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/Cassie/input_supervisor.h"

#include "drake/multibody/plant/multibody_plant.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

PYBIND11_MODULE(cassie_utils, m) {
  m.doc() = "Binding utility functions for Cassie";

  using drake::multibody::MultibodyPlant;

  py::class_<InputSupervisor>(m, "InputSupervisor")
      .def(py::init<const drake::multibody::MultibodyPlant<double>&,
                    const std::string&, double, double, Eigen::VectorXd&>())
      .def("get_input_port_command", &InputSupervisor::get_input_port_command)
      .def("get_input_port_safety_controller",
           &InputSupervisor::get_input_port_safety_controller)
      .def("get_input_port_state", &InputSupervisor::get_input_port_state)
      .def("get_input_port_controller_switch",
           &InputSupervisor::get_input_port_controller_switch)
      .def("get_input_port_cassie", &InputSupervisor::get_input_port_cassie)
      .def("get_input_port_controller_error",
           &InputSupervisor::get_input_port_controller_error)
      .def("get_output_port_command", &InputSupervisor::get_output_port_command)
      .def("get_output_port_status", &InputSupervisor::get_output_port_status)
      .def("get_output_port_failure",
           &InputSupervisor::get_output_port_failure);
}

}  // namespace pydairlib
}  // namespace dairlib