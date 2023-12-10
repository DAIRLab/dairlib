#include "geometry/meshcat_utils.h"

namespace dairlib::geometry{

using Eigen::Vector3d;
using Eigen::Matrix3Xd;
using Eigen::Matrix3Xi;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using drake::geometry::Meshcat;

void MeshcatUtils::PlotColoredSurface(std::string_view path,
                                      std::shared_ptr<Meshcat> MeshcatObject,
                                      const Eigen::Ref<const Eigen::MatrixXd>& X,
                                      const Eigen::Ref<const Eigen::MatrixXd>& Y,
                                      const Eigen::Ref<const Eigen::MatrixXd>& Z,
                                      const Eigen::Ref<const Eigen::MatrixXd>& R,
                                      const Eigen::Ref<const Eigen::MatrixXd>& G,
                                      const Eigen::Ref<const Eigen::MatrixXd>& B,
                                      bool wireframe, double wireframe_line_width) {
  for (const auto& mat: {Y, Z, R, G, B}) {
      DRAKE_DEMAND(mat.cols() == X.cols() && mat.rows() == X.rows());
  }
  const int rows = X.rows();
  const int cols = X.cols();

  if (wireframe) {
    int count = -1;
    Eigen::Matrix3Xd vertices(3, rows * cols * 2);
    // Sweep back and forth along rows.
    for (int r = 0; r < rows; ++r) {
      const int c0 = (r & 0x1) ? cols - 1 : 0;
      const int c_delta = (r & 0x1) ? -1 : 1;
      for (int j = 0, c = c0; j < cols; ++j, c += c_delta) {
        vertices.col(++count) << X(r, c), Y(r, c), Z(r, c);
      }
    }
    // Sweep back and forth along columns.
    const int c0 = (rows & 0x1) ? cols - 1 : 0;
    const int c_delta = (rows & 0x1) ? -1 : 1;
    for (int j = 0, c = c0; j < cols; ++j, c += c_delta) {
      const int r0 = (j & 0x1) ? 0 : rows - 1;
      const int r_delta = (j & 0x1) ? 1 : -1;
      for (int i = 0, r = r0; i < rows; ++i, r += r_delta) {
        vertices.col(++count) << X(r, c), Y(r, c), Z(r, c);
      }
    }

//      meshcatobject.impl().SetLine(path, vertices, wireframe_line_width, rgba);
  } else {
    using MapRowVector = const Eigen::Map<const Eigen::RowVectorXd>;

    Eigen::Matrix3Xd vertices(3, rows * cols);
    Eigen::Matrix3Xd colors(3, rows * cols);
    vertices.row(0) = MapRowVector(X.data(), rows * cols);
    vertices.row(1) = MapRowVector(Y.data(), rows * cols);
    vertices.row(2) = MapRowVector(Z.data(), rows * cols);
    colors.row(0) = MapRowVector(R.data(), rows * cols);
    colors.row(1) = MapRowVector(G.data(), rows * cols);
    colors.row(2) = MapRowVector(B.data(), rows * cols);

    // Make a regular grid as in https://stackoverflow.com/q/44934631.
    const int num_boxes = (rows - 1) * (cols - 1);
    Eigen::Matrix3Xi faces(3, 2 * num_boxes);
    Eigen::MatrixXi ids(rows, cols);
    // Populate ids with [0, 1, ..., num vertices-1]
    std::iota(ids.data(), ids.data() + rows * cols, 0);

    int count = 0;
    for (int i = 0; i < rows - 1; ++i) {
      for (int j = 0; j < cols - 1; ++j) {
        // Upper left triangles.
        faces.col(count++) << ids(i, j), ids(i + 1, j), ids(i, j + 1);
        // Lower right triangles.
        faces.col(count++) << ids(i + 1, j), ids(i + 1, j + 1), ids(i, j + 1);
      }
    }

      MeshcatObject->SetTriangleColorMesh(path, vertices, faces, colors, wireframe,
                           wireframe_line_width,
                           Meshcat::SideOfFaceToRender::kDoubleSide);
  }
}
}