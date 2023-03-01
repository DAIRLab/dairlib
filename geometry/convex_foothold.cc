#include "geometry/convex_foothold.h"

namespace dairlib::geometry{

using Eigen::Vector3d;
using Eigen::RowVector3d;
using Eigen::Matrix3Xd;
using Eigen::Matrix3Xi;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

void ConvexFoothold::SetContactPlane(const Vector3d& normal,
                                     const Vector3d& pt) {
  A_eq_ = normal.transpose();
  b_eq_ = normal.dot(pt) * VectorXd::Ones(1);
}

// Rotate the coordinate frame so that the constraint A x_W <= b
// becomes A R_WF x_F <= b
void ConvexFoothold::ReExpressInNewFrame(const Eigen::Matrix3d &R_WF) {
  A_ = A_ * R_WF;
  A_eq_ = A_eq_ * R_WF;
}

void ConvexFoothold::AddHalfspace(Vector3d a, VectorXd b) {
  if (A_.rows() == 0) {
    A_ = a.transpose();
    b_ = b;
  } else {
    A_.conservativeResize(A_.rows() + 1, Eigen::NoChange);
    A_.bottomRows(1) = a.transpose();
    b_.conservativeResize(b_.rows() + 1);
    b_.tail(1) = b;
  }
}

// Add a face with the (outward facing) normal and a point on the face
void ConvexFoothold::AddFace(const Vector3d& normal, const Vector3d& pt) {
  AddHalfspace(normal, normal.dot(pt) * VectorXd::Ones(1));
}

void ConvexFoothold::AddVertices(const Vector3d &v1, const Vector3d &v2) {
  Vector3d face = v2 - v1;
  Vector3d normal = face.cross(A_eq_.transpose());
  AddFace(normal, v1);
}

std::pair<MatrixXd, VectorXd> ConvexFoothold::GetConstraintMatrices() const {
  return {A_, b_};
}

std::pair<MatrixXd, VectorXd> ConvexFoothold::GetEqualityConstraintMatrices() const {
  return {A_eq_, b_eq_};
}

Matrix3d ConvexFoothold::R_WF() const {
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

void ConvexFoothold::SortFacesByYawAngle() {
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

Vector3d ConvexFoothold::SolveForVertexSharedByFaces(int i, int j) {
  Matrix3d A = Matrix3d::Zero();
  Vector3d b = Vector3d::Zero();
  A.row(0) = A_eq_;
  A.row(1) = A_.row(i);
  A.row(2) = A_.row(j);
  b(0) = b_eq_(0);
  b(1) = b_(i);
  b(2) = b_(j);
  return A.inverse() * b;
}

Matrix3Xd ConvexFoothold::GetVertices() {
  SortFacesByYawAngle();
  Matrix3Xd verts = Matrix3Xd::Zero(3, A_.rows());
  for (int i = 1; i < A_.rows(); i++) {
    verts.col(i) = SolveForVertexSharedByFaces(i, i-1);
  }
  verts.leftCols(1) = SolveForVertexSharedByFaces(0, A_.rows() -1);
  return verts;
}

std::pair<Matrix3Xd, Matrix3Xi> ConvexFoothold::GetSurfaceMesh() {
  Matrix3Xd verts = Matrix3Xd::Zero(3, A_.rows() + 1);
  verts.leftCols(A_.rows()) = GetVertices();
  Vector3d centroid = Vector3d::Zero();
  for (int i = 0; i < A_.rows(); i++) {
    centroid += verts.col(i);
  }
  verts.rightCols(1) = centroid / A_.rows();
  Matrix3Xi idxs = Matrix3Xi ::Zero(A_.rows());
}

}