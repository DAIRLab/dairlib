#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/perceptive_locomotion/diagrams/cassie_elevation_mapping_lcm_diagram.h"
#include "examples/perceptive_locomotion/diagrams/cassie_realsense_driver_diagram.h"
#include "examples/perceptive_locomotion/diagrams/perceptive_full_sim_diagram.h"
#include "examples/perceptive_locomotion/diagrams/full_sim_diagram.h"

namespace py = pybind11;

namespace dairlib{
namespace pydairlib{

using perceptive_locomotion::CassieRealSenseDriverDiagram;
using perceptive_locomotion::CassieElevationMappingLcmDiagram;
using perceptive_locomotion::PerceptiveFullSimDiagram;
using perceptive_locomotion::FullSimDiagram;


PYBIND11_MODULE(diagrams, m) {
  m.doc() = "Binding perceptive locomotion diagrams for developing perception "
            "modules in python.";

  using py_rvp = py::return_value_policy;
  py::module::import("pydrake.systems.framework");

  py::class_<CassieElevationMappingLcmDiagram, drake::systems::Diagram<double>>(
        m, "CassieElevationMappingLcmDiagram")
        .def(py::init<const std::string&, const std::string&>(),
            py::arg("params_yaml"), py::arg("points_channel"))
        .def("InitializeElevationMap", &CassieElevationMappingLcmDiagram::InitializeElevationMap)
        .def("lcm", &CassieElevationMappingLcmDiagram::lcm, py_rvp::reference_internal)
        .def("plant", &CassieElevationMappingLcmDiagram::plant, py_rvp::reference_internal)
        .def("get_input_port_state", &CassieElevationMappingLcmDiagram::get_input_port_state, py_rvp::reference_internal)
        .def("get_input_port_contact", &CassieElevationMappingLcmDiagram::get_input_port_contact, py_rvp::reference_internal);

  py::class_<CassieRealSenseDriverDiagram, drake::systems::Diagram<double>>(
        m, "CassieRealSenseDriverDiagram")
        .def(py::init<const std::string&>(), py::arg("points_channel"))
        .def("InitializeElevationMap", &CassieRealSenseDriverDiagram::InitializeElevationMap)
        .def("ReInitializeElevationMap", &CassieRealSenseDriverDiagram::ReInitilizeElevationMap)
        .def("lcm", &CassieRealSenseDriverDiagram::lcm, py_rvp::reference_internal)
        .def("plant", &CassieRealSenseDriverDiagram::plant, py_rvp::reference_internal)
        .def("get_input_port_state", &CassieRealSenseDriverDiagram::get_input_port_state, py_rvp::reference_internal)
        .def("get_input_port_contact", &CassieRealSenseDriverDiagram::get_input_port_contact, py_rvp::reference_internal)
        .def("get_output_port_grid_map", &CassieRealSenseDriverDiagram::get_output_port_grid_map, py_rvp::reference_internal)
        .def("get_output_port_profiling", &CassieRealSenseDriverDiagram::get_output_port_profiling, py_rvp::reference_internal)
        .def("start_rs", &CassieRealSenseDriverDiagram::start_rs)
        .def("stop_rs", &CassieRealSenseDriverDiagram::stop_rs);

  py::class_<PerceptiveFullSimDiagram, drake::systems::Diagram<double>>(
      m, "PerceptiveFullSimDiagram")
    .def(py::init<const std::string&,  const std::string&, const std::string&, const std::string&>(),
         py::arg("mpc_gains_yaml"), py::arg("solver_options_yaml"), py::arg("terrain_yaml"), py::arg("sim_options"))
      .def("SetPlantInitialConditions",
           &dairlib::perceptive_locomotion::PerceptiveFullSimDiagram::SetPlantInitialConditions,
           py::arg("diagram"),
           py::arg("context"),
           "Set initial conditions for the plant")
      .def("SaveLcmLog", &PerceptiveFullSimDiagram::SaveLcmLog, py::arg("fname"))
      .def("get_input_port_footholds", &PerceptiveFullSimDiagram::get_input_port_footholds,
           py_rvp::reference_internal)
      .def("get_input_port_grid_map", &PerceptiveFullSimDiagram::get_input_port_grid_map,
           py_rvp::reference_internal)
      .def("get_output_port_grid_map", &PerceptiveFullSimDiagram::get_output_port_grid_map,
           py_rvp::reference_internal)
      .def("GetCassiePelvisPoseInWorld", &PerceptiveFullSimDiagram::GetCassiePelvisPoseInWorld)
      .def("meshcat", &PerceptiveFullSimDiagram::meshcat);

  py::class_<FullSimDiagram, drake::systems::Diagram<double>>(
      m, "FullSimDiagram")
    .def(py::init<const std::string&,  const std::string&,
                  const std::string&, const std::string&, bool>(),
         py::arg("mpc_gains_yaml"), py::arg("solver_options_yaml"), py::arg
         ("terrain_yaml"), py::arg("sim_options"), py::arg("visualize"))
      .def("SetPlantInitialConditions",
           &dairlib::perceptive_locomotion::FullSimDiagram::SetPlantInitialConditions,
           py::arg("diagram"),
           py::arg("context"),
           "Set initial conditions for the plant")
      .def("SaveLcmLog", &FullSimDiagram::SaveLcmLog, py::arg("fname"))
      .def("GetCassiePelvisPoseInWorld", &FullSimDiagram::GetCassiePelvisPoseInWorld)
      .def("meshcat", &FullSimDiagram::meshcat);
   }


}
}