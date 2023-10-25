#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/perceptive_locomotion/diagrams/hiking_sim_diagram.h"

namespace py = pybind11;


namespace dairlib{
namespace pydairlib {

using perceptive_locomotion::HikingSimDiagram;
using multibody::SquareSteppingStoneList;

PYBIND11_MODULE(simulators, m) {
  m.doc() = "Bindings for perceptive locomotion simulators";
  using py_rvp = py::return_value_policy;

  py::class_<HikingSimDiagram, drake::systems::Diagram<double>>(
      m, "HikingSimDiagram")
      .def(py::init<const std::variant<std::string, SquareSteppingStoneList>&,
                    const std::string&>(),
           py::arg("tarrain_yaml"), py::arg("camera_pose_yaml"))
    .def("get_input_port_actuation",
         &HikingSimDiagram::get_input_port_actuation,
         py_rvp::reference_internal)
    .def("get_input_port_radio",
         &HikingSimDiagram::get_input_port_radio,
         py_rvp::reference_internal)
    .def("get_output_port_state_lcm",
         &HikingSimDiagram::get_output_port_state_lcm,
         py_rvp::reference_internal)
    .def("get_output_port_state",
         &HikingSimDiagram::get_output_port_state,
         py_rvp::reference_internal)
    .def("get_output_port_cassie_out",
         &HikingSimDiagram::get_output_port_cassie_out,
         py_rvp::reference_internal)
    .def("get_output_port_lcm_radio",
         &HikingSimDiagram::get_output_port_lcm_radio,
         py_rvp::reference_internal)
    .def("get_plant",
         &HikingSimDiagram::get_plant,
         py_rvp::reference_internal)
    .def("AddDrakeVisualizer",
         &HikingSimDiagram::AddDrakeVisualizer,
         py_rvp::reference_internal)
    .def("SetPlantInitialConditionFromIK",
         &HikingSimDiagram::SetPlantInitialConditionFromIK)
    .def("SetPlantInitialCondition",
         &HikingSimDiagram::SetPlantInitialCondition);
}

}
}