#include "geometry/convex_foothold.h"

namespace dairlib::geometry{

using Eigen::Vector3d;
using Eigen::RowVector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

void ConvexFoothold::SetContactPlane(Eigen::Vector3d normal,
                                     Eigen::Vector3d pt) {
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

}