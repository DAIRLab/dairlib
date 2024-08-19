#include <cmath>
#include <Eigen/Dense>

namespace dairlib::polynomials {

Eigen::MatrixXd MakeChangeOfBasisOperatorFromLegendreToMonomials(int order);
Eigen::MatrixXi LegendreBasisDerivativeOperator(int order);

/*!
 * Compute the inner product of two polynomials, expressed as coefficients over
 * the basis of legendre polynomials.
 *
 * Needed b/c the legendre polynomials are orthogonal but not orthonormal
 *
 * @param u
 * @param v
 * @return <u, v>
 */
double LegendreBasisInner(const Eigen::VectorXd& u, const Eigen::VectorXd& v);

Eigen::VectorXd EvalLegendreBasis(int order, double t);

}


