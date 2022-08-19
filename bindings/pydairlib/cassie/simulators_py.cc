#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "examples/Cassie/diagrams/cassie_sim_diagram.h"
#include "examples/Cassie/diagrams/cassie_vision_sim_diagram.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using examples::CassieSimDiagram;
using examples::CassieVisionSimDiagram;
using examples::VisionSimTerrainType;

PYBIND11_MODULE(simulators, m) {
  m.doc() = "Binding controller factories for Cassie";

  using py_rvp = py::return_value_policy;

  py::class_<dairlib::examples::CassieSimDiagram,
             drake::systems::Diagram<double>>(m, "CassieSimDiagram")
      .def(py::init<
               std::unique_ptr<drake::multibody::MultibodyPlant<double>>,
               const std::string &, bool, double, Eigen::Vector3d>(),
           py::arg("plant"),
           py::arg("urdf"),
           py::arg("visualize"),
           py::arg("mu"),
           py::arg("normal"))
      .def("get_plant", &CassieSimDiagram::get_plant,
           py_rvp::reference_internal)
      .def("get_actuation_input_port",
           &CassieSimDiagram::get_actuation_input_port,
           py_rvp::reference_internal)
      .def("get_radio_input_port", &CassieSimDiagram::get_radio_input_port,
           py_rvp::reference_internal)
      .def("get_state_output_port", &CassieSimDiagram::get_state_output_port,
           py_rvp::reference_internal)
      .def("get_cassie_out_output_port",
           &CassieSimDiagram::get_cassie_out_output_port,
           py_rvp::reference_internal);


  py::enum_<VisionSimTerrainType>(m, "VisionSimTerrainType")
      .value("kFlat", VisionSimTerrainType::kFlat)
      .value("kStairs", VisionSimTerrainType::kStairs)
      .value("kBoxy", VisionSimTerrainType::kBoxy)
      .value("kCubic", VisionSimTerrainType::kCubic)
      .value("kCubicWithVoids", VisionSimTerrainType::kCubicWithVoids);

  py::class_<dairlib::examples::CassieVisionSimDiagram,
             drake::systems::Diagram<double>>(m, "CassieVisionSimDiagram")
      .def(py::init<
               std::unique_ptr<drake::multibody::MultibodyPlant<double>>,
               const Eigen::VectorXd&, double,
               const std::string &, VisionSimTerrainType, bool, double, double,
               Eigen::Vector3d>(),
           py::arg("plant"),
           py::arg("camera_position"),
           py::arg("camera_pitch"),
           py::arg("urdf"),
           py::arg("terrain_type"),
           py::arg("visualize"),
           py::arg("mu"),
           py::arg("map_yaw"),
           py::arg("normal"))
      .def("get_plant", &CassieVisionSimDiagram::get_plant,
           py_rvp::reference_internal)
      .def("get_actuation_input_port",
           &CassieVisionSimDiagram::get_actuation_input_port,
           py_rvp::reference_internal)
      .def("get_radio_input_port",
           &CassieVisionSimDiagram::get_radio_input_port,
           py_rvp::reference_internal)
      .def("get_state_output_port",
           &CassieVisionSimDiagram::get_state_output_port,
           py_rvp::reference_internal)
      .def("get_cassie_out_output_port",
           &CassieVisionSimDiagram::get_cassie_out_output_port,
           py_rvp::reference_internal)
      .def("get_camera_out_output_port",
           &CassieVisionSimDiagram::get_camera_out_output_port,
           py_rvp::reference_internal)
      .def("get_camera_pose_output_port",
           &CassieVisionSimDiagram::get_camera_pose_output_port,
           py_rvp::reference_internal)
      .def("X_BC", &CassieVisionSimDiagram::X_BC)
      .def_static("default_camera_transform",
           &CassieVisionSimDiagram::default_camera_transform);
}
}  // namespace pydairlib
}  // namespace dairlib