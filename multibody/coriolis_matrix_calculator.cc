//
// Created by brian on 11/7/20.
//
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/math/autodiff_gradient.h>
#include <drake/common/autodiff.h>

#include "coriolis_matrix_calculator.h"


using drake::math::autoDiffToGradientMatrix;

using drake::AutoDiffXd;
using drake::AutoDiffVecXd;
using drake::AutoDiffd;

namespace dairlib::multibody {
    CoriolisMatrixCalculator::CoriolisMatrixCalculator(
            const drake::multibody::MultibodyPlant<AutoDiffXd>& plant)
            : plant_(const_cast<drake::multibody::MultibodyPlant<AutoDiffXd> &>(plant)),
              n_q_(plant.num_positions()),
              n_v_(plant.num_velocities()){
        Cv_ = AutoDiffVecXd::Zero(n_v_, 1);
    }

    void CoriolisMatrixCalculator::CalcCoriolisAutoDiff(const std::unique_ptr<drake::systems::Context<AutoDiffXd>> context,
                                                        drake::MatrixX<AutoDiffXd>& C) {

        DRAKE_ASSERT(C.rows() == n_v_);
        DRAKE_ASSERT(C.cols() == n_v_);

        plant_.CalcBiasTerm(*context, &Cv_);
        auto jac = autoDiffToGradientMatrix(Cv_);
        C = 0.5*jac.rightCols(n_v_);
    }
}

