#include <gtest/gtest.h>

#include "geometry/convex_polygon_set.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"

namespace dairlib::geometry{
namespace {
using drake::CompareMatrices;

class ConvexPolygonTest : public ::testing::Test {
 protected:
  void SetUp() override {
    verts_ << -1, 0, 1, 0,
               0,-1, 0, 1,
               0, 0, 0, 0;
    diamond_.SetPlane(Eigen::Vector3d::UnitZ(), 0);
    for (int i = 0; i < 4; i++) {
      diamond_.AddVertices(verts_.col(i), verts_.col((i + 1) % 4));
    }
  }
  Eigen::Matrix<double, 3, 4> verts_;
  ConvexPolygon diamond_;
  std::vector<ConvexPolygon> polygons_;
 };
}

TEST_F(ConvexPolygonTest, SortFacesTest) {
  // check faces already in yaw ordering get sorted correctly
  const auto& [A1, b1] = diamond_.GetConstraintMatrices();
  diamond_.SortFacesByYawAngle();
  const auto& [A2, b2] = diamond_.GetConstraintMatrices();
  EXPECT_TRUE(CompareMatrices(A1, A2, 1e-12));
  EXPECT_TRUE(CompareMatrices(b1, b2, 1e-12));

  // check faces not in yaw ordering get sorted correctly
  ConvexPolygon p;
  p.SetPlane(Eigen::Vector3d::UnitZ(), 0);
  for (const auto i : {3, 1, 2, 0}) {
    p.AddHalfspace(A1.row(i).transpose(), Eigen::VectorXd::Constant(1, b1(i)));
  }
  p.SortFacesByYawAngle();
  const auto& [A3, b3] = p.GetConstraintMatrices();
  EXPECT_TRUE(CompareMatrices(A1, A3, 1e-12));
  EXPECT_TRUE(CompareMatrices(b1, b3, 1e-12));
}

TEST_F(ConvexPolygonTest, GetVertexTest) {
  const auto& verts_processed = diamond_.GetVertices();
  EXPECT_TRUE(CompareMatrices(verts_, verts_processed, 0.01));
}
}

int main(int argc, char*argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}