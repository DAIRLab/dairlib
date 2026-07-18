#include <vector>

#include <Eigen/Dense>

#include "drake/common/drake_assert.h"

/// CopyVectorXdToStdVector returns an std::vector<double> which is converted
/// from an Eigen::VectorXd.
std::vector<double> CopyVectorXdToStdVector(const Eigen::VectorXd& eigen_vec);

Eigen::VectorXd StdVectorToVectorXd(std::vector<double>);

Eigen::VectorXd eigen_clamp(const Eigen::VectorXd& value,
                            const Eigen::VectorXd& lb,
                            const Eigen::VectorXd& ub);
