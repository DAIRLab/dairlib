#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "geometry/convex_foothold.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using geometry::ConvexFoothold;

PYBIND11_MODULE(convex_foothold, m) {
m.doc() = "Binding geometry utils";

using py_rvp = py::return_value_policy;

py::class_<ConvexFoothold>(m, "ConvexFoothold")
    .def(py::init<>())
    .def("SetContactPlane", &ConvexFoothold::SetContactPlane)
    .def("AddHalfspace", &ConvexFoothold::AddHalfspace)
    .def("AddFace", &ConvexFoothold::AddFace)
    .def("GetConstraintMatrices", &ConvexFoothold::GetConstraintMatrices)
    .def("GetEqualityConstraintMatrices", &ConvexFoothold::GetEqualityConstraintMatrices);
}

}
}