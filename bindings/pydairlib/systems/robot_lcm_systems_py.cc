#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "systems/robot_lcm_systems.h"

#include "drake/multibody/plant/multibody_plant.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

PYBIND11_MODULE(robot_lcm_systems, m) {
  m.doc() = "Binding robot lcm systems";

  using drake::multibody::MultibodyPlant;

  py::class_<systems::RobotOutputReceiver, drake::systems::LeafSystem<double>>(
      m, "RobotOutputReceiver")
      .def(py::init<const MultibodyPlant<double>&>());
  py::class_<systems::RobotInputReceiver, drake::systems::LeafSystem<double>>(
      m, "RobotInputReceiver")
      .def(py::init<const MultibodyPlant<double>&>());
  py::class_<systems::RobotOutputSender, drake::systems::LeafSystem<double>>(
      m, "RobotOutputSender")
      .def(py::init<const MultibodyPlant<double>&, bool>())
      .def("get_input_port_state", &systems::RobotOutputSender::get_input_port_state, py::return_value_policy::reference_internal)
      .def("get_input_port_imu", &systems::RobotOutputSender::get_input_port_imu, py::return_value_policy::reference_internal)
      .def("get_input_port_effort", &systems::RobotOutputSender::get_input_port_effort, py::return_value_policy::reference_internal);
}

}  // namespace pydairlib
}  // namespace dairlib