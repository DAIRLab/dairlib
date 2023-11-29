#pragma once
#include "Eigen/Dense"
#include "Eigen/Geometry"

namespace dairlib::geometry {

constexpr int kMaxFootholdFaces = 10;

/// Class representing a convex foothold consisting of a single
/// equality constraint a'x = b defining the contact plane, and up to
/// kMaxFootholdFaces inequality constraints defining the extents of the
/// foothold. No effort is made to check feasibility or reasonableness of any
/// combination of constraints. No frame information is supplied.
class ConvexPolygon {
 public:
  ConvexPolygon()= default;

  /*
   * Set the polygon plane by supplying a normal and a point on the plane
   */
  void SetPlane(const Eigen::Vector3d& normal, const Eigen::Vector3d& pt);

  /*
   * Set the polygon's plane to a'x = b
   */
  void SetPlane(const Eigen::Vector3d& a, double b) {
    double norm_a = a.norm();
    A_eq_ = a.transpose() / norm_a;
    b_eq_ = Eigen::VectorXd::Constant(1, b / norm_a);
  }

  /*
   * Make the convex polygon the intersection of itself with the halfspace
   * a'x <= b
   */
  void AddHalfspace(Eigen::Vector3d a, Eigen::VectorXd b);

  /*
   * Add a face with an outward facing normal which intersects with
   * pt
   */
  void AddFace(const Eigen::Vector3d& normal, const Eigen::Vector3d& pt);

  /*
   * Add a face by adding two vertices. v1 and v2 should be unique points
   * in the contact plane, and with the contact normal pointing toward the
   * observer, v2 should be counterclockwise from v1. These conditions are not
   * checked.
   */
  void AddVertices(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2);

  /*
   * Get the violation of the inequality constraints
   */
  double Get2dViolation(const Eigen::Vector3d& pt) const;

  /*
   * Returns true if inequality constraints are violated by the point
   */
  bool PointViolatesInequalities(const Eigen::Vector3d& pt) const;

  /*
   * Get the inequality constraints, Ax <= b, as a pair {A, b}
   */
  std::pair<Eigen::MatrixXd, Eigen::VectorXd> GetConstraintMatrices() const;

  /*
   * Get the equality constraints Aeq*x == b as a pair {Aeq, b}
   */
  std::pair<Eigen::MatrixXd, Eigen::VectorXd> GetEqualityConstraintMatrices() const;
  void ReExpressInNewFrame(const Eigen::Matrix3d& R_WF);
  Eigen::Matrix3Xd GetVertices();

  static ConvexPolygon MakeFlatGround(double half_len=100.0) {
    ConvexPolygon foothold;
    foothold.SetPlane(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero());
    foothold.AddFace(Eigen::Vector3d::UnitX(),
                     half_len * Eigen::Vector3d::UnitX());
    foothold.AddFace(-Eigen::Vector3d::UnitX(),
                     -half_len * Eigen::Vector3d::UnitX());
    foothold.AddFace(Eigen::Vector3d::UnitY(),
                     half_len * Eigen::Vector3d::UnitY());
    foothold.AddFace(-Eigen::Vector3d::UnitY(),
                     -half_len * Eigen::Vector3d::UnitY());
    return foothold;
  }

  void CalcBoundingBox();

  Eigen::Matrix3d R_WF() const;
  std::pair<Eigen::Matrix3Xd, Eigen::Matrix3Xi> GetSurfaceMesh();

  // only public for unit_testing
  void SortFacesByYawAngle();

 private:

  struct bounding_box {
    bool valid = false;
    double xmin_ = -std::numeric_limits<double>::infinity();
    double ymin_ = -std::numeric_limits<double>::infinity();
    double zmin_ = -std::numeric_limits<double>::infinity();
    double xmax_ = std::numeric_limits<double>::infinity();
    double ymax_ = std::numeric_limits<double>::infinity();
    double zmax_ = std::numeric_limits<double>::infinity();
  };

  bounding_box bounding_box_;
  Eigen::Vector3d SolveForVertexSharedByFaces(int i, int j);

  Eigen::RowVector3d A_eq_;
  Eigen::VectorXd b_eq_;
  Eigen::MatrixXd A_ = Eigen::MatrixXd::Zero(0,0);
  Eigen::VectorXd b_ = Eigen::VectorXd::Zero(0);
};
}
