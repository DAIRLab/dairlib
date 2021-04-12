//
// Created by brian on 4/12/21.
//

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/multibody/plant/multibody_plant.h>

#include "examples/KoopmanMPC/koopman_mpc.h"
#include "lcm/lcm_trajectory.h"


namespace  dairlib {

class KoopmanMPCTrajectory : public LcmTrajectory {
 public:
  KoopmanMPCTrajectory(const std::string& filepath ) {LoadFromFile(filepath);}

  KoopmanMPCTrajectory(const drake::multibody::MultibodyPlant<double>& plant,
                       const KoopmanMPC& koopman_mpc,
                       const drake::solvers::MathematicalProgramResult result,
                       const std::string& name, const std::string& description);

  drake::trajectories::PiecewisePolynomial<double> ReconstructComTrajectory() const;
  drake::trajectories::PiecewisePolynomial<double> ReconstructAngularTrajectory() const;
  drake::trajectories::PiecewisePolynomial<double> ReconstructFootTrajectory(koopMpcStance stance) const;



};

}