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

  py::class_<systems::RobotOutputReceiver>(m, "RobotOutputReceiver")
      .def(py::init<const MultibodyPlant<double>&>());
  py::class_<systems::RobotInputReceiver>(m, "RobotInputReceiver")
      .def(py::init<const MultibodyPlant<double>&>());
  py::class_<systems::RobotOutputSender>(m, "RobotOutputSender")
      .def(py::init<const MultibodyPlant<double>&, bool>());
}

}  // namespace pydairlib
}  // namespace dairlib