/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   InEKF.h
 *  @author Ross Hartley
 *  @brief  Header file for Invariant EKF
 *  @date   September 25, 2018
 **/

#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <iostream>
#include <map>
#include <vector>
#include "LieGroup.h"
#include "NoiseParams.h"
#include "RobotState.h"

namespace inekf {

class Kinematics {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Kinematics(int id_in, Eigen::Matrix4d pose_in,
             Eigen::Matrix<double, 6, 6> covariance_in)
      : id(id_in), pose(pose_in), covariance(covariance_in) {}

  int id;
  Eigen::Matrix4d pose;
  Eigen::Matrix<double, 6, 6> covariance;
};

class Landmark {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Landmark(int id_in, Eigen::Vector3d position_in)
      : id(id_in), position(position_in) {}

  int id;
  Eigen::Vector3d position;
};

typedef std::map<
    int, Eigen::Vector3d, std::less<int>,
    Eigen::aligned_allocator<std::pair<const int, Eigen::Vector3d> > >
    mapIntVector3d;
typedef std::map<
    int, Eigen::Vector3d, std::less<int>,
    Eigen::aligned_allocator<std::pair<const int, Eigen::Vector3d> > >::iterator
    mapIntVector3dIterator;
typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark> >
    vectorLandmarks;
typedef std::vector<Landmark,
                    Eigen::aligned_allocator<Landmark> >::const_iterator
    vectorLandmarksIterator;
typedef std::vector<Kinematics, Eigen::aligned_allocator<Kinematics> >
    vectorKinematics;
typedef std::vector<Kinematics,
                    Eigen::aligned_allocator<Kinematics> >::const_iterator
    vectorKinematicsIterator;

class Observation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Observation(Eigen::VectorXd& Y, Eigen::VectorXd& b, Eigen::MatrixXd& H,
              Eigen::MatrixXd& N, Eigen::MatrixXd& PI);
  bool empty();

  Eigen::VectorXd Y;
  Eigen::VectorXd b;
  Eigen::MatrixXd H;
  Eigen::MatrixXd N;
  Eigen::MatrixXd PI;

  friend std::ostream& operator<<(std::ostream& os, const Observation& o);
};

class InEKF {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  InEKF();
  InEKF(NoiseParams params);
  InEKF(RobotState state);
  InEKF(RobotState state, NoiseParams params);

  // assignment operator (needed for drake abstract value)
  InEKF& operator = (const InEKF&);

  RobotState getState() const;
  NoiseParams getNoiseParams() const;
  mapIntVector3d getPriorLandmarks() const;
  std::map<int, int> getEstimatedLandmarks() const;
  std::map<int, bool> getContacts() const;
  std::map<int, int> getEstimatedContactPositions() const;
  void setState(RobotState state);
  void setNoiseParams(NoiseParams params);
  void setPriorLandmarks(const mapIntVector3d& prior_landmarks);
  void setContacts(std::vector<std::pair<int, bool> > contacts);

  void Propagate(const Eigen::Matrix<double, 6, 1>& m, double dt);
  void Correct(const Observation& obs);
  void CorrectLandmarks(const vectorLandmarks& measured_landmarks);
  void CorrectKinematics(const vectorKinematics& measured_kinematics);

 private:
  RobotState state_;
  NoiseParams noise_params_;
  mapIntVector3d prior_landmarks_;
  std::map<int, int> estimated_landmarks_;
  std::map<int, bool> contacts_;
  std::map<int, int> estimated_contact_positions_;
  const Eigen::Vector3d g_ = Eigen::Vector3d(0, 0, -9.81);  // Gravity
};

}  // namespace inekf
