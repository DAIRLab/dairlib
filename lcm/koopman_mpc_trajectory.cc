//
// Created by brian on 4/12/21.
//

#include "koopman_mpc_trajectory.h"

#include "multibody/multibody_utils.h"

using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;

using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using std::string;
using std::vector;

namespace dairlib {

KoopmanMPCTrajectory::KoopmanMPCTrajectory(const drake::multibody::MultibodyPlant<double>& plant,
                                           const std::vector<KoopmanMpcMode>& koopman_modes,
                                           const drake::solvers::MathematicalProgramResult result,
                                           const std::string& name, const std::string& description){
  int n_modes = koopman_modes.size();

}

}