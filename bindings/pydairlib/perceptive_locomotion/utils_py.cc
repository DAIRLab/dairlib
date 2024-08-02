#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "multibody/multibody_utils.h"
#include "geometry/convex_polygon_set.h"
#include "geometry/polygon_height_map.h"
#include <grid_map_core/GridMap.hpp>

namespace py = pybind11;

namespace dairlib{
namespace pydairlib {

using grid_map::GridMap;
using grid_map::Position;
using grid_map::Index;
using grid_map::InterpolationMethods;

using geometry::PolygonHeightMap;

PYBIND11_MODULE(utils, m) {

  m.def("CalcHeightMapInStanceFrame", [](
      const GridMap* grid_map, const std::string& layer,
      const drake::multibody::MultibodyPlant<double>* plant,
      const drake::systems::Context<double>* context,
      const std::string& body_name,
      const Eigen::Ref<Eigen::Vector3d>& stance_pos,
      const Eigen::Ref<Eigen::Vector3d>& center,
      const Eigen::Ref<Eigen::VectorXd>& xgrid_stance_frame,
      const Eigen::Ref<Eigen::VectorXd>& ygrid_stance_frame) {

    Eigen::Vector3d body_x =
      plant->GetBodyByName(body_name).EvalPoseInWorld(*context).rotation().col(0);

    double yaw  = atan2(body_x(1), body_x(0));
    double cyaw = cos(yaw);
    double syaw = sin(yaw);

    Eigen::MatrixXd hmap = Eigen::MatrixXd::Constant(
        xgrid_stance_frame.size(),
        ygrid_stance_frame.size(),
        std::nan("")
    );

    for (int col = 0; col < hmap.cols(); ++col) {
      for (int row = 0; row < hmap.rows(); ++row) {
        Eigen::Vector2d q = Eigen::Vector2d(
            cyaw * (xgrid_stance_frame(row) + center(0)) - syaw * (ygrid_stance_frame(col) + center(1)),
            syaw * (xgrid_stance_frame(row) + center(0)) + cyaw * (ygrid_stance_frame(col) + center(1))
        ) + stance_pos.head<2>();
        if (grid_map->isInside(q)) {
          hmap(row, col) = grid_map->atPosition(layer, q, InterpolationMethods::INTER_NEAREST) - stance_pos(2);
        }
      }
    }

    return hmap;
  }, py::arg("grid_map"), py::arg("layer"),
     py::arg("plant"), py::arg("plant_context"), py::arg("floating_base_body_name"),
     py::arg("stance_pos"), py::arg("center"), py::arg("xgrid_stance_frame"),
     py::arg("ygrid_stance_frame"))
  .def("CalcGTHeightMapInStanceFrame", [](
      const PolygonHeightMap* poly_hmap,
      const drake::multibody::MultibodyPlant<double>* plant,
      const drake::systems::Context<double>* context,
      const std::string& body_name,
      const Eigen::Ref<Eigen::Vector3d>& stance_pos,
      const Eigen::Ref<Eigen::Vector3d>& center,
      const Eigen::Ref<Eigen::VectorXd>& xgrid_stance_frame,
      const Eigen::Ref<Eigen::VectorXd>& ygrid_stance_frame) {

    Eigen::Vector3d body_x =
      plant->GetBodyByName(body_name).EvalPoseInWorld(*context).rotation().col(0);

    double yaw  = atan2(body_x(1), body_x(0));
    double cyaw = cos(yaw);
    double syaw = sin(yaw);

    Eigen::MatrixXd hmap = Eigen::MatrixXd::Constant(
        xgrid_stance_frame.size(),
        ygrid_stance_frame.size(),
        std::nan("")
    );

    for (int col = 0; col < hmap.cols(); ++col) {
      for (int row = 0; row < hmap.rows(); ++row) {
        double x = cyaw * (xgrid_stance_frame(row) + center(0)) - syaw *
            (ygrid_stance_frame(col) + center(1));
        double y = syaw * (xgrid_stance_frame(row) + center(0)) + cyaw *
        (ygrid_stance_frame(col) + center(1));
        hmap(row, col) = poly_hmap->CalcHeight(x, y) - stance_pos(2);
      }
    }

    return hmap;
  },
  py::arg("polygons"),
  py::arg("plant"), py::arg("plant_context"), py::arg("floating_base_body_name"),
  py::arg("stance_pos"), py::arg("center"), py::arg("xgrid_stance_frame"),
  py::arg("ygrid_stance_frame"));

}

}
}