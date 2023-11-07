#pragma once

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include "dairlib/lcmt_c3_output.hpp"

namespace dairlib {

/// Used for outputting c3 solutions and intermediate variables for debugging purposes

class C3Output {
 public:
  struct C3Solution {
    C3Solution() = default;

    // Shape is (variable_vector_size, knot_points)
    Eigen::VectorXf time_vector_;
    Eigen::MatrixXf x_sol_;
    Eigen::MatrixXf lambda_sol_;
    Eigen::MatrixXf u_sol_;
  };

  struct C3Intermediates {
    C3Intermediates() = default;
//    C3Intermediates(Eigen::VectorXd time_vector_, Eigen::MatrixXd delta_,
//                    Eigen::MatrixXd w_);

    // Shape is (variable_vector_size, knot_points)
    Eigen::VectorXf time_vector_;
    Eigen::MatrixXf delta_;
    Eigen::MatrixXf w_;
  };

  C3Output() = default;
  C3Output(C3Solution c3_sol, C3Intermediates c3_intermediates);

  explicit C3Output(const lcmt_c3_output& traj);

  virtual ~C3Output() = default;

  lcmt_c3_output GenerateLcmObject(double time) const;

 private:
  C3Solution c3_solution_;
  C3Intermediates c3_intermediates_;
};

}  // namespace dairlib
