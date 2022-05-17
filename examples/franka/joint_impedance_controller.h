// INCLUDES
// TODO:
// 1. confirm that includes are ok
// 2. confirm parameters to constructor and other functions

#pragma once

// collection of includes I found from example files in dairlib/systems/controllers
#include "drake/systems/framework/leaf_system.h"
#include "multibody/multibody_utils.h"
#include "drake/math/autodiff_gradient.h"
#include "systems/framework/output_vector.h"
#include "multibody/multibody_utils.h"
#include "drake/math/autodiff_gradient.h"

namespace dairlib{
namespace systems{

using Eigen::VectorXd;
using Eigen::MatrixXd;

template <typename T>
class JointImpedanceController : public drake::systems::LeafSystem<T> {

    public:
    JointImpedanceController(const MatrixXd& K, const MatrixXd& B);

    // TODO: what other functions are needed?

    MatrixXd get_K() { return K_; }
    MatrixXd get_B() { return B_; }

    private:
    // parameters
    MatrixXd K_;
    MatrixXd B_;

    // input/output indices
    int input_index_state_;
    int input_index_desired_state_;
    int input_index_lambda_;
    int output_index_control_;

}; // controller class

} // systems namspace
} // dairlib namespace