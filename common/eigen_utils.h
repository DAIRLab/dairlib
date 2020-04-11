#include <vector>
#include <Eigen/Dense>

/// ConvertVectorXdToStdVector returns an std::vector<double> which is converted
/// from an Eigen::VectorXd.
std::vector<double> ConvertVectorXdToStdVector(
    const Eigen::VectorXd& eigen_vec);
