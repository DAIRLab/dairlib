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
      .def(py::init<const MultibodyPlant<double>&, bool>());
  py::class_<systems::RobotCommandSender, drake::systems::LeafSystem<double>>(
      m, "RobotCommandSender")
      .def(py::init<const MultibodyPlant<double>&>());
  m.def("AddActuationRecieverAndStateSenderLcm",
        &dairlib::systems::AddActuationRecieverAndStateSenderLcm,
        py::arg("builder"), py::arg("plant"), py::arg("lcm"),
        py::arg("actuator_channel"), py::arg("state_channel"),
        py::arg("publish_rate"), py::arg("publish_efforts"),
        py::arg("actuator_delay"));

}

}  // namespace pydairlib
}  // namespace dairlib
