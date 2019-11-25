#include "examples/PlanarWalker/LIPM_swing_leg.h"
#include "drake/common/polynomial.h"

template <typename T>
LIPMSwingLeg<T>::LIPMSwingLeg(double g, double z_nom, double step_time,
                                   double cop_max)
    : HybridSystem<T>(3, 1 + (cop_max > 0), 0),
      g_(g),
      z_nom_(z_nom),
      step_time_(step_time),
      cop_max_(cop_max) {
  num_states_ = 3;
  num_inputs_ = (cop_max > 0) ? 2 : 1;
}

template <typename T>
int LIPMSwingLeg<T>::get_num_states() const {
  return num_states_;
}

template <typename T>
int LIPMSwingLeg<T>::get_num_inputs() const {
  return num_inputs_;
}

template <typename T>
double LIPMSwingLeg<T>::get_desired_com_height() const {
  return z_nom_;
}

template <typename T>
double LIPMSwingLeg<T>::get_omega() const {
  return sqrt(g_ / z_nom_);
}

template <typename T>
void LIPMSwingLeg<T>::controlAffineDynamics(
    double time, Eigen::Matrix<T, Eigen::Dynamic, 1> x,
    Eigen::Matrix<T, Eigen::Dynamic, 1> &f,
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &g) const {
  f.resize(this->num_states_, 1);
  f(0) = x(1);
  f(1) = x(0) * g_ / z_nom_;
  f(2) = 0;

  g.resize(this->num_states_, this->num_inputs_);
  if (cop_max_ > 0) {
    g(0, 0) = 0;
    g(0, 1) = 0;
    g(1, 0) = -cop_max_ * g_ / z_nom_;
    g(1, 1) = 0;
    g(2, 0) = 0;
    g(2, 1) = 1;
  } else {
    g(0) = 0;
    g(1) = 0;
    g(2) = 1;
  }
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> LIPMSwingLeg<T>::dynamics(
    double time, Eigen::Matrix<T, Eigen::Dynamic, 1> x,
    Eigen::Matrix<T, Eigen::Dynamic, 1> u) const {
  Eigen::Matrix<T, Eigen::Dynamic, 1> f;
  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> g;
  controlAffineDynamics(time, x, f, g);
  return f + g * u;
}

template <typename T>
Eigen::Matrix<T, Eigen::Dynamic, 1> LIPMSwingLeg<T>::reset(
    double time, Eigen::Matrix<T, Eigen::Dynamic, 1> x,
    Eigen::Matrix<T, Eigen::Dynamic, 1> u) const {
  Eigen::Matrix<T, Eigen::Dynamic, 1> x_p;
  x_p.resize(3, 1);
  x_p(0) = x(0) - x(2);
  x_p(1) = x(1);
  x_p(2) = -x(2);
  return x_p;
}

template <typename T>
T LIPMSwingLeg<T>::inputLimits(Eigen::Matrix<T, Eigen::Dynamic, 1> x,
                 Eigen::Matrix<T, Eigen::Dynamic, 1> u) const {
  return 1 - u.transpose() * u;
}

template <typename T>
T LIPMSwingLeg<T>::resetInputLimits(Eigen::Matrix<T, Eigen::Dynamic, 1> x,
                      Eigen::Matrix<T, Eigen::Dynamic, 1> s) const {
  return 1;
}

template class LIPMSwingLeg<double>;
template class LIPMSwingLeg<Polynomiald>;
