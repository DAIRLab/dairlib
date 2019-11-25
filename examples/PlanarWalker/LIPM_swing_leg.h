#include <iostream>
#include "examples/PlanarWalker/hybrid_system.h"

template <typename T>
class LIPMSwingLeg : public HybridSystem<T> {
 private:
  double g_;
  double z_nom_;
  double step_time_;
  double cop_max_;
  int num_states_;
  int num_inputs_;

 public:
  LIPMSwingLeg(double g, double z_nom, double step_time, double cop_max);

  int get_num_inputs() const;

  int get_num_states() const;

  double get_desired_com_height() const;

  double get_omega() const;

  void controlAffineDynamics(
      double time, Eigen::Matrix<T, Eigen::Dynamic, 1> x,
      Eigen::Matrix<T, Eigen::Dynamic, 1> &f,
      Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &g) const;

  Eigen::Matrix<T, Eigen::Dynamic, 1> dynamics(
      double time, Eigen::Matrix<T, Eigen::Dynamic, 1> x,
      Eigen::Matrix<T, Eigen::Dynamic, 1> u) const override;


  Eigen::Matrix<T, Eigen::Dynamic, 1> reset(
      double time, Eigen::Matrix<T, Eigen::Dynamic, 1> x,
      Eigen::Matrix<T, Eigen::Dynamic, 1> u) const override;

  T inputLimits(Eigen::Matrix<T, Eigen::Dynamic, 1> x,
                   Eigen::Matrix<T, Eigen::Dynamic, 1> u) const override;

  T resetInputLimits(Eigen::Matrix<T, Eigen::Dynamic, 1> x,
                        Eigen::Matrix<T, Eigen::Dynamic, 1> s) const override;
};
