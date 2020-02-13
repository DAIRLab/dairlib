#include <iostream>
#include <string>
#include "math.h"
#include <Eigen/Dense>

#include "drake/common/drake_assert.h"

#include "drake/systems/framework/system.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "drake/math/autodiff_gradient.h"
#include "multibody/multibody_utils.h"
#include "common/find_resource.h"

using std::cout;
using std::endl;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using drake::VectorX;
using drake::MatrixX;
using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::math::DiscardGradient;
using drake::math::autoDiffToValueMatrix;
using drake::math::autoDiffToGradientMatrix;
using drake::math::initializeAutoDiff;

using dairlib::FindResourceOrThrow;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;

int main() {
  bool left_stance = false;

  VectorXd x_double = VectorXd::Random(14);
  AutoDiffVecXd x = initializeAutoDiff(x_double);

  if (left_stance) {
    VectorX<AutoDiffXd> left_foot_pos_z(1);
    left_foot_pos_z <<
                    x(1) - 0.5 * cos(x(2) + x(3)) - 0.5 * cos(x(2) + x(3) + x(5));
    MatrixX<AutoDiffXd> J_left_foot_pos_z(1, 7);
    J_left_foot_pos_z << 0,
                      1,
                      0.5 * sin(x(2) + x(3)) + 0.5 * sin(x(2) + x(3) + x(5)),
                      0.5 * sin(x(2) + x(3)) + 0.5 * sin(x(2) + x(3) + x(5)),
                      0,
                      0.5 * sin(x(2) + x(3) + x(5)),
                      0;
    // VectorX<AutoDiffXd> left_foot_vel_z = J_left_foot_pos_z * x.tail(7);
    cout << "Manual J_left_foot_pos_z = " << J_left_foot_pos_z << endl;

    MatrixXd J_left_foot_pos_z_autoDiff = autoDiffToGradientMatrix(
                                            left_foot_pos_z).block(0, 0, 1, 7);
    cout << "AutoDiff J_left_foot_pos_z = " << J_left_foot_pos_z_autoDiff << endl;

  } else {
    VectorX<AutoDiffXd> right_foot_pos_z(1);
    right_foot_pos_z <<
                     x(1) - 0.5 * cos(x(2) + x(4)) - 0.5 * cos(x(2) + x(4) + x(6));
    MatrixX<AutoDiffXd> J_right_foot_pos_z(1, 7);
    J_right_foot_pos_z << 0,
                       1,
                       0.5 * sin(x(2) + x(4)) + 0.5 * sin(x(2) + x(4) + x(6)),
                       0,
                       0.5 * sin(x(2) + x(4)) + 0.5 * sin(x(2) + x(4) + x(6)),
                       0,
                       0.5 * sin(x(2) + x(4) + x(6));
    // VectorX<AutoDiffXd> right_foot_vel_z = J_right_foot_pos_z * x.tail(7);
    cout << "Manual J_right_foot_pos_z = " << J_right_foot_pos_z << endl;

    MatrixXd J_right_foot_pos_z_autoDiff = autoDiffToGradientMatrix(
        right_foot_pos_z).block(0, 0, 1, 7);
    cout << "AutoDiff J_right_foot_pos_z = " << J_right_foot_pos_z_autoDiff << endl;
  }

  return 0;
}
