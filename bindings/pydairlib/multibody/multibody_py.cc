#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "multibody/multibody_utils.h"
#include "multibody/multipose_visualizer.h"
#include "multibody/stepping_stone_utils.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using multibody::MultiposeVisualizer;
using multibody::SquareSteppingStoneList;
using geometry::ConvexPolygon;

PYBIND11_MODULE(multibody, m) {
  m.import("pydrake.all");
  m.doc() = "Binding utility functions for MultibodyPlant";

  py::class_<MultiposeVisualizer>(m, "MultiposeVisualizer")
      .def(py::init<std::string, int, std::string>())
      .def(py::init<std::string, int, double, std::string>())
      .def(py::init<std::string, int, Eigen::VectorXd, std::string>())
      .def("DrawPoses", &MultiposeVisualizer::DrawPoses, py::arg("poses"))
      .def("AddSteppingStonesFromYaml",
           &MultiposeVisualizer::AddSteppingStonesFromYaml, py::arg("filename"))
      .def("GetMeshcat", &MultiposeVisualizer::GetMeshcat);

  py::class_<SquareSteppingStoneList>(m, "SquareSteppingStoneList")
      .def(py::init<std::vector<std::vector<std::vector<double>>>,
                    std::vector<std::pair<RigidTransformd, Eigen::Vector3d>>,
                    std::vector<ConvexPolygon>>())
      .def("GetFootholdsWithMargin", &SquareSteppingStoneList::GetFootholdsWithMargin)
      .def_readwrite("stones", &SquareSteppingStoneList::stones)
      .def_readwrite("cubes", &SquareSteppingStoneList::cubes)
      .def_readwrite("footholds", &SquareSteppingStoneList::footholds);

  m.def("LoadSteppingStonesFromYaml", &multibody::LoadSteppingStonesFromYaml,
        py::arg("filename"));

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
      .def("AddFlatTerrain", &dairlib::multibody::AddFlatTerrain<double>,
           py::arg("plant"), py::arg("scene_graph"), py::arg("mu_static"),
           py::arg("mu_kinetic"),
           py::arg("normal_W") = Eigen::Vector3d(0, 0, 1),
           py::arg("show_ground") = 1)
      .def("ReExpressWorldVector3InBodyYawFrame",
          &dairlib::multibody::ReExpressWorldVector3InBodyYawFrame<double>,
          py::arg("plant"),
          py::arg("context"),
          py::arg("body_name"),
          py::arg("vec"))
      .def("ReExpressBodyYawVector3InWorldFrame",
          &dairlib::multibody::ReExpressBodyYawVector3InWorldFrame<double>,
          py::arg("plant"),
          py::arg("context"),
          py::arg("body_name"),
          py::arg("vec"));
}

}  // namespace pydairlib
}  // namespace dairlib
