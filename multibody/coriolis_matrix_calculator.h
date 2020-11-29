#pragma once

//
// Created by brian on 11/7/20.
//

#include <drake/multibody/plant/multibody_plant.h>

using drake::AutoDiffXd;
using drake::AutoDiffVecXd;
using drake::MatrixX;

namespace dairlib {
namespace multibody {
    ///
    class CoriolisMatrixCalculator {
    public:
        explicit CoriolisMatrixCalculator(drake::multibody::MultibodyPlant<AutoDiffXd>& plant);

        void CalcCoriolisAutoDiff(std::unique_ptr<drake::systems::Context<AutoDiffXd>> &context,
                                        MatrixX <AutoDiffXd> &C) const;
    private:
        drake::multibody::MultibodyPlant<AutoDiffXd>& plant_;
        int n_v_;
        int n_q_;
    };

    }
}

