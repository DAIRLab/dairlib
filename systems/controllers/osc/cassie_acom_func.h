#pragma once

#include <Eigen/Dense>
#include "drake/math/autodiff_gradient.h"
#include "drake/common/eigen_types.h"

using std::string;
using std::vector;

using Eigen::MatrixXd;
using Eigen::VectorXd;

using drake::AutoDiffXd;

namespace dairlib::systems::controllers {

template <typename T>
T getQx(const drake::VectorX<T>& q);
template <typename T>
T getQy(const drake::VectorX<T>& q);
template <typename T>
T getQz(const drake::VectorX<T>& q);

////////////
template <typename T>
void getJQx0(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQx1(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQx2(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQx3(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQx4(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQx5(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQx8(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQx9(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQx10(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQx11(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQx12(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQx13(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);

template <typename T>
void getJQx(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);

///////////////////////////////
template <typename T>
void getJQy0(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQy1(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQy2(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQy3(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQy4(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQy5(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQy8(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQy9(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQy10(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQy11(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQy12(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQy13(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);

template <typename T>
void getJQy(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);

/////////////////////////////
template <typename T>
void getJQz0(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQz1(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQz2(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQz3(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQz4(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQz5(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQz8(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQz9(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQz10(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQz11(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQz12(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);
template <typename T>
void getJQz13(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);

template <typename T>
void getJQz(const drake::VectorX<T>& q, drake::MatrixX<T>& JQ);

}  // namespace dairlib::systems::controllers
