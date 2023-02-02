#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "multibody/multibody_utils.h"
#include "multibody/multipose_visualizer.h"
#include "multibody/visualization_utils.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using multibody::MultiposeVisualizer;

PYBIND11_MODULE(multibody, m) {
  py::module::import("pydrake.all");

  m.doc() = "Binding utility functions for MultibodyPlant";

  py::class_<MultiposeVisualizer>(m, "MultiposeVisualizer")
      .def(py::init<std::string, int, std::string>())
      .def(py::init<std::string, int, double, std::string>())
      .def(py::init<std::string, int, Eigen::VectorXd, std::string>())
      .def("DrawPoses", &MultiposeVisualizer::DrawPoses, py::arg("poses"));

  m.def("ConnectTrajectoryVisualizer",
        &dairlib::multibody::ConnectTrajectoryVisualizer, py::arg("plant"),
        py::arg("builder"), py::arg("scene_graph"), py::arg("trajectory"));

  m.def("MakeNameToPositionsMap",
        &dairlib::multibody::MakeNameToPositionsMap<double>, py::arg("plant"))
      .def("MakeNameToVelocitiesMap",
           &dairlib::multibody::MakeNameToVelocitiesMap<double>,
           py::arg("plant"))
      .def("MakeNameToActuatorsMap",
           &dairlib::multibody::MakeNameToActuatorsMap<double>,
           py::arg("plant"))
      .def("CreateStateNameVectorFromMap",
           &dairlib::multibody::CreateStateNameVectorFromMap<double>,
           py::arg("plant"))
      .def("CreateActuatorNameVectorFromMap",
           &dairlib::multibody::CreateActuatorNameVectorFromMap<double>,
           py::arg("plant"))
      .def("CreateWithSpringsToWithoutSpringsMapPos",
           &dairlib::multibody::CreateWithSpringsToWithoutSpringsMapPos<double>,
           py::arg("plant_w_spr"), py::arg("plant_wo_spr"))
      .def("CreateWithSpringsToWithoutSpringsMapVel",
           &dairlib::multibody::CreateWithSpringsToWithoutSpringsMapVel<double>,
           py::arg("plant_w_spr"), py::arg("plant_wo_spr"))
      .def("AddFlatTerrain", &dairlib::multibody::AddFlatTerrain<double>,
           py::arg("plant"), py::arg("scene_graph"), py::arg("mu_static"),
           py::arg("mu_kinetic"),
           py::arg("normal_W") = Eigen::Vector3d(0, 0, 1),
           py::arg("stiffness") = 0, py::arg("dissipation_rate") = 0,
           py::arg("show_ground") = 1);
}

}  // namespace pydairlib
}  // namespace dairlib
