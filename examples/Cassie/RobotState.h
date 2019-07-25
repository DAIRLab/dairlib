/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   RobotState.h
 *  @author Ross Hartley
 *  @brief  Header file for RobotState
 *  @date   September 25, 2018
 **/

#pragma once
#include <Eigen/Dense>
#include <iostream>

namespace inekf {

class RobotState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RobotState();
  RobotState(const Eigen::MatrixXd& X);
  RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta);
  RobotState(const Eigen::MatrixXd& X, const Eigen::VectorXd& Theta,
             const Eigen::MatrixXd& P);

  const Eigen::MatrixXd getX();
  const Eigen::VectorXd getTheta();
  const Eigen::MatrixXd getP();
  const Eigen::Matrix3d getRotation();
  const Eigen::Vector3d getVelocity();
  const Eigen::Vector3d getPosition();
  const Eigen::Vector3d getGyroscopeBias();
  const Eigen::Vector3d getAccelerometerBias();
  const int dimX();
  const int dimTheta();
  const int dimP();

  void setX(const Eigen::MatrixXd& X);
  void setP(const Eigen::MatrixXd& P);
  void setTheta(const Eigen::VectorXd& Theta);
  void setRotation(const Eigen::Matrix3d& R);
  void setVelocity(const Eigen::Vector3d& v);
  void setPosition(const Eigen::Vector3d& p);
  void setGyroscopeBias(const Eigen::Vector3d& bg);
  void setAccelerometerBias(const Eigen::Vector3d& ba);

  void copyDiagX(int n, Eigen::MatrixXd& BigX);

  friend std::ostream& operator<<(std::ostream& os, const RobotState& s);

 private:
  Eigen::MatrixXd X_;
  Eigen::VectorXd Theta_;
  Eigen::MatrixXd P_;
};

}  // namespace inekf

