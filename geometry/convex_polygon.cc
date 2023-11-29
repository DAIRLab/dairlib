#include "geometry/convex_polygon.h"

namespace dairlib::geometry{

using Eigen::Vector3d;
using Eigen::RowVector3d;
using Eigen::Matrix3Xd;
using Eigen::Matrix3Xi;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

void ConvexPolygon::SetPlane(const Vector3d& normal, const Vector3d& pt) {
  Vector3d normalized = normal.normalized();
  A_eq_ = normalized.transpose();
  b_eq_ = normalized.dot(pt) * VectorXd::Ones(1);
  bounding_box_.valid = false;
}

// Rotate the coordinate frame so that the constraint A x_W <= b
// becomes A R_WF x_F <= b
void ConvexPolygon::ReExpressInNewFrame(const Eigen::Matrix3d &R_WF) {
  A_ = A_ * R_WF;
  A_eq_ = A_eq_ * R_WF;
  bounding_box_.valid = false;
}

void ConvexPolygon::AddHalfspace(Vector3d a, VectorXd b) {
  if (A_.rows() == 0) {
    A_ = a.transpose();
    b_ = b;
  } else {
    A_.conservativeResize(A_.rows() + 1, Eigen::NoChange);
    A_.bottomRows(1) = a.transpose();
    b_.conservativeResize(b_.rows() + 1);
    b_.tail(1) = b;
  }
  bounding_box_.valid = false;
}

// Add a face with the (outward facing) normal and a point on the face
void ConvexPolygon::AddFace(const Vector3d& normal, const Vector3d& pt) {
  AddHalfspace(normal, normal.dot(pt) * VectorXd::Ones(1));
  bounding_box_.valid = false;
}

void ConvexPolygon::AddVertices(const Vector3d &v1, const Vector3d &v2) {
  Vector3d face = v2 - v1;
  Vector3d normal = face.cross(A_eq_.transpose());
  AddFace(normal, v1);
  bounding_box_.valid = false;
}

std::pair<MatrixXd, VectorXd> ConvexPolygon::GetConstraintMatrices() const {
  return {A_, b_};
}

std::pair<MatrixXd, VectorXd> ConvexPolygon::GetEqualityConstraintMatrices() const {
  return {A_eq_, b_eq_};
}

double ConvexPolygon::Get2dViolation(const Eigen::Vector3d &pt) const {
  const auto& [A, b] = GetConstraintMatrices();
  const auto viol = A * pt - b;
  return viol.maxCoeff();
}

bool ConvexPolygon::PointViolatesInequalities(const Eigen::Vector3d &pt) const {
  if (bounding_box_.valid) {
    if (pt(0) < bounding_box_.xmin_ or pt(0) > bounding_box_.xmax_ or
        pt(1) < bounding_box_.ymin_ or pt(1) > bounding_box_.ymax_) {
      return true;
    }
  }
  return Get2dViolation(pt) > 0;
}

Matrix3d ConvexPolygon::R_WF() const {
  Vector3d b_z = A_eq_.transpose().normalized();
  Vector3d b_x (b_z(2), 0, -b_z(0));
  b_x.normalize();
  Vector3d b_y = b_z.cross(b_x).normalized();
  Matrix3d R_WF = Matrix3d::Zero();
  R_WF.col(0) = b_x;
  R_WF.col(1) = b_y;
  R_WF.col(2) = b_z;
  return R_WF;
}

namespace {

bool yaw_greater(const RowVector3d& a, const RowVector3d& b, const Matrix3d& R){
  const Vector3d arot = R * a.transpose();
  const Vector3d brot = R * b.transpose();
  return (atan2(arot(1), arot(0)) > atan2(brot(1), brot(0)));
}
}

void ConvexPolygon::SortFacesByYawAngle() {
  const Matrix3d R_FW = R_WF().transpose();
  // Cheeky little insertion sort since these are small matrices
  for (int i = 1; i < A_.rows(); i++) {
    int j = i;
    while (j > 0 && yaw_greater(A_.row(j-1), A_.row(j), R_FW)) {
      A_.row(j).swap(A_.row(j-1));
      b_.row(j).swap(b_.row(j-1));
      j--;
    }
  }
}

Vector3d ConvexPolygon::SolveForVertexSharedByFaces(int i, int j) {
  Matrix3d A = Matrix3d::Zero();
  Vector3d b = Vector3d::Zero();
  A.row(0) = A_eq_;
  A.row(1) = A_.row(i);
  A.row(2) = A_.row(j);
  if (A.row(1).cross(A.row(2)).normalized().dot(A_eq_) > 0 ) {
    throw std::runtime_error(
        "A convex Polygon must be closed in order to find a valid set of "
        "vertices"
    );
  }
  b(0) = b_eq_(0);
  b(1) = b_(i);
  b(2) = b_(j);
  return A.inverse() * b;
}

void ConvexPolygon::CalcBoundingBox() {
  auto verts = GetVertices();
  bounding_box_.xmin_ = verts.row(0).minCoeff();
  bounding_box_.ymin_ = verts.row(1).minCoeff();
  bounding_box_.zmin_ = verts.row(2).minCoeff();
  bounding_box_.xmax_ = verts.row(0).maxCoeff();
  bounding_box_.ymax_ = verts.row(1).maxCoeff();
  bounding_box_.zmax_ = verts.row(2).maxCoeff();
  bounding_box_.valid = true;
}


Matrix3Xd ConvexPolygon::GetVertices() {
  SortFacesByYawAngle();
  Matrix3Xd verts = Matrix3Xd::Zero(3, A_.rows());
  for (int i = 1; i < A_.rows(); i++) {
    verts.col(i) = SolveForVertexSharedByFaces(i, i-1);
  }
  verts.leftCols(1) = SolveForVertexSharedByFaces(0, A_.rows() -1);
  return verts;
}

std::pair<Matrix3Xd, Matrix3Xi> ConvexPolygon::GetSurfaceMesh() {
  int N = A_.rows();
  Matrix3Xd verts = Matrix3Xd::Zero(3, N + 1);
  verts.leftCols(N) = GetVertices();
  Vector3d centroid = Vector3d::Zero();
  for (int i = 0; i < N; i++) {
    centroid += verts.col(i);
  }
  verts.rightCols(1) = centroid / N;
  Matrix3Xi idxs = Matrix3Xi::Zero(3, N);
  for (int i = 0; i < A_.rows(); i++) {
    idxs(0, i) = N;
    idxs(1, i) = i;
    idxs(2, i) = (i + 1) % N;
  }
  return {verts, idxs};
}

}