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
  using systems::RobotOutputSender;

  py::class_<systems::RobotOutputReceiver, drake::systems::LeafSystem<double>>(
      m, "RobotOutputReceiver")
      .def(py::init<const MultibodyPlant<double>&>());
  py::class_<systems::RobotInputReceiver, drake::systems::LeafSystem<double>>(
      m, "RobotInputReceiver")
      .def(py::init<const MultibodyPlant<double>&>());
  py::class_<RobotOutputSender, drake::systems::LeafSystem<double>>(
      m, "RobotOutputSender")
      .def(py::init<const MultibodyPlant<double>&, bool>())
      .def("get_input_port_state", &RobotOutputSender::get_input_port_state)
      .def("get_input_port_effort", &RobotOutputSender::get_input_port_effort)
      .def("get_input_port_imu", &RobotOutputSender::get_input_port_imu);
  py::class_<systems::RobotCommandSender, drake::systems::LeafSystem<double>>(
      m, "RobotCommandSender")
      .def(py::init<const MultibodyPlant<double>&>());

}

}  // namespace pydairlib
}  // namespace dairlib