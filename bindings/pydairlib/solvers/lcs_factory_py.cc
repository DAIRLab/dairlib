#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "solvers/lcs_factory.h"

#include "drake/bindings/pydrake/common/sorted_pair_pybind.h"

namespace py = pybind11;

namespace dairlib {
namespace pydairlib {

using dairlib::solvers::ContactModel;
using dairlib::solvers::LCSFactory;

PYBIND11_MODULE(lcs_factory, m) {
  py::enum_<ContactModel>(m, "ContactModel")
      .value("kStewartAndTrinkle", ContactModel::kStewartAndTrinkle)
      .value("kAnitescu", ContactModel::kAnitescu)
      .export_values();
  py::class_<LCSFactory>(m, "LCSFactory")
      .def_static("linearize_plant_to_lcs", &LCSFactory::LinearizePlantToLCS,
                  py::arg("plant"), py::arg("context"), py::arg("plant_ad"),
                  py::arg("context_ad"), py::arg("contact_geoms"),
                  py::arg("num_friction_directions"), py::arg("mu"),
                  py::arg("dt"), py::arg("N"),
                  py::arg("contact_model") = ContactModel::kStewartAndTrinkle);
}
}  // namespace pydairlib
}  // namespace dairlib
