#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "geometry/convex_polygon.h"
#include "geometry/convex_polygon_set.h"
#include "geometry/convex_polygon_lcm_systems.h"

#include "drake/bindings/pydrake/common/value_pybind.h"


namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using geometry::ConvexPolygon;
using geometry::ConvexPolygonSet;
using geometry::ConvexPolygonSender;

PYBIND11_MODULE(convex_polygon, m) {
m.doc() = "Binding geometry utils";

using py_rvp = py::return_value_policy;

py::class_<ConvexPolygon>(m, "ConvexPolygon")
    .def(py::init<>())
    .def("SetPlane",
         py::overload_cast<const Eigen::Vector3d&, const Eigen::Vector3d&>(
         &ConvexPolygon::SetPlane))
    .def("SetPlane", py::overload_cast<const Eigen::Vector3d&, double>(
         &ConvexPolygon::SetPlane))
    .def("AddHalfspace", &ConvexPolygon::AddHalfspace)
    .def("AddFace", &ConvexPolygon::AddFace)
    .def("GetConstraintMatrices", &ConvexPolygon::GetConstraintMatrices)
    .def("GetEqualityConstraintMatrices", &ConvexPolygon::GetEqualityConstraintMatrices)
    .def("GetVertices", &ConvexPolygon::GetVertices)
    .def("Get2dViolation", &ConvexPolygon::Get2dViolation);

py::class_<ConvexPolygonSet>(m, "ConvexPolygonSet")
    .def(py::init<std::vector<ConvexPolygon>>(), py::arg("set"))
    .def("CalcHeightOfPoint", &ConvexPolygonSet::CalcHeightOfPoint);

py::class_<ConvexPolygonSender, drake::systems::LeafSystem<double>>(m, "ConvexPolygonSender").def(py::init<>());

  drake::pydrake::AddValueInstantiation<ConvexPolygon>(m);
  drake::pydrake::AddValueInstantiation<ConvexPolygonSet>(m);

}

}
}