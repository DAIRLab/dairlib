#pragma once
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "drake/geometry/meshcat.h"

namespace dairlib::geometry {

using drake::geometry::Meshcat;

/// Class of exteneded function customized for visualization
class MeshcatUtils{
 public:
    MeshcatUtils() = default;
    static void PlotColoredSurface(std::string_view path,
                            std::shared_ptr<Meshcat>,
                            const Eigen::Ref<const Eigen::MatrixXd>& X,
                            const Eigen::Ref<const Eigen::MatrixXd>& Y,
                            const Eigen::Ref<const Eigen::MatrixXd>& Z,
                            const Eigen::Ref<const Eigen::MatrixXd>& R,
                            const Eigen::Ref<const Eigen::MatrixXd>& G,
                            const Eigen::Ref<const Eigen::MatrixXd>& B,
                            bool wireframe = false, double wireframe_line_width = 1.0);
};
}
