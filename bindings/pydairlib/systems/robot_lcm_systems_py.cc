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
  using py_rvp = py::return_value_policy;

  py::class_<systems::RobotOutputReceiver, drake::systems::LeafSystem<double>>(
      m, "RobotOutputReceiver")
      .def(py::init<const MultibodyPlant<double>&>())
      .def(py::init<const MultibodyPlant<double>&,
                    drake::multibody::ModelInstanceIndex>());
  py::class_<systems::RobotInputReceiver, drake::systems::LeafSystem<double>>(
      m, "RobotInputReceiver")
      .def(py::init<const MultibodyPlant<double>&>());
  py::class_<RobotOutputSender, drake::systems::LeafSystem<double>>(
      m, "RobotOutputSender")
      .def(py::init<const MultibodyPlant<double>&, bool, bool>())
      .def(py::init<const MultibodyPlant<double>&,
                    drake::multibody::ModelInstanceIndex,
                    bool, bool>())
      .def("get_input_port_state", &RobotOutputSender::get_input_port_state,
           py_rvp::reference_internal)
      .def("get_input_port_effort", &RobotOutputSender::get_input_port_effort,
           py_rvp::reference_internal)
      .def("get_input_port_imu", &RobotOutputSender::get_input_port_imu,
           py_rvp::reference_internal);
  py::class_<systems::RobotCommandSender, drake::systems::LeafSystem<double>>(
      m, "RobotCommandSender")
      .def(py::init<const MultibodyPlant<double>&>());
  m.def("AddActuationRecieverAndStateSenderLcm",
        py::overload_cast<drake::systems::DiagramBuilder<double>*,
                          const MultibodyPlant<double>&,
                          drake::systems::lcm::LcmInterfaceSystem*, std::string,
                          std::string, double,
                          drake::multibody::ModelInstanceIndex, bool, double>(
            &dairlib::systems::AddActuationRecieverAndStateSenderLcm),
        py::arg("builder"), py::arg("plant"), py::arg("lcm"),
        py::arg("actuator_channel"), py::arg("state_channel"),
        py::arg("publish_rate"), py::arg("model_instance"),
        py::arg("publish_efforts"), py::arg("actuator_delay"));
  m.def("AddActuationRecieverAndStateSenderLcm",
        py::overload_cast<drake::systems::DiagramBuilder<double>*,
                          const MultibodyPlant<double>&,
                          drake::systems::lcm::LcmInterfaceSystem*, std::string,
                          std::string, double, bool, double>(
            &dairlib::systems::AddActuationRecieverAndStateSenderLcm),
        py::arg("builder"), py::arg("plant"), py::arg("lcm"),
        py::arg("actuator_channel"), py::arg("state_channel"),
        py::arg("publish_rate"), py::arg("publish_efforts"),
        py::arg("actuator_delay"));
}

}  // namespace pydairlib
}  // namespace dairlib
