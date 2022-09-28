#include <vector>
#include <Eigen/Dense>

/// CopyVectorXdToStdVector returns an std::vector<double> which is converted
/// from an Eigen::VectorXd.
std::vector<double> CopyVectorXdToStdVector(
    const Eigen::VectorXd& eigen_vec);

std::vector<float> CopyVectorXdToStdFloatVector(
    const Eigen::VectorXd& eigen_vec);