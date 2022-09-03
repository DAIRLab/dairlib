#pragma once
#include "Eigen/Dense"

namespace dairlib::geometry {

 class ConvexFoothold {
  public:
   ConvexFoothold()= default;
   void SetContactPlane(Eigen::Vector3d normal, Eigen::Vector3d pt);
   void AddHalfspace(Eigen::Vector3d a, Eigen::VectorXd b);
   void AddFace(Eigen::Vector3d normal, Eigen::Vector3d pt);
   std::pair<Eigen::MatrixXd, Eigen::VectorXd>GetConstraintMatrices() const;
   std::pair<Eigen::MatrixXd, Eigen::VectorXd>GetEqualityConstraintMatrices() const;
   std::vector<Eigen::Vector3d> GetVertices();
   void ReExpressInNewFrame(const Eigen::Matrix3d& R_WF);

  private:
   Eigen::Vector3d SolveForVertexSharedByFaces(int i, int j);
   void SortFacesByYawAngle();

   Eigen::RowVector3d A_eq_;
   Eigen::VectorXd b_eq_;
   Eigen::MatrixXd A_ = Eigen::MatrixXd::Zero(0,0);
   Eigen::VectorXd b_ = Eigen::VectorXd::Zero(0);

 };

}
