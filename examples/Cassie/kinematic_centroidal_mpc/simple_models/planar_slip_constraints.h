#pragma  once

#include <drake/multibody/plant/multibody_plant.h>
#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/constraint.h"
#include "solvers/nonlinear_constraint.h"
#include "solvers/nonlinear_cost.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "examples/Cassie/kinematic_centroidal_mpc/simple_models/planar_slip_lifter.h"

template <typename T>
class PlanarSlipReductionConstraint : public dairlib::solvers::NonlinearConstraint<T> {

 public:
  PlanarSlipReductionConstraint(const drake::multibody::MultibodyPlant<T>& plant,
                                drake::systems::Context<T>* context,
                                const std::vector<dairlib::multibody::WorldPointEvaluator<T>>& slip_feet,
                                int complex_state_size, int knot_index);

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

  const T m_;
  drake::systems::Context<T>* context_;
  const drake::multibody::MultibodyPlant<T>& plant_;
  int nx_;
  const std::vector<dairlib::multibody::WorldPointEvaluator<T>> slip_feet_;
  const int kSLIP_DIM = 2;
  const Eigen::Vector2i slip_index_{0,2};
};

class PlanarSlipLiftingConstraint : public dairlib::solvers::NonlinearConstraint<double> {
 public:
  PlanarSlipLiftingConstraint(const drake::multibody::MultibodyPlant<double>& plant,
                              std::shared_ptr<PlanarSlipLifter> lifting_function,
                              int n_slip_feet, int n_complex_feet,
                              int knot_index);

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override;

  std::shared_ptr<PlanarSlipLifter> lifting_function_;
  const int slip_dim_;
  const int complex_dim_;
};

template <typename T>
class PlanarSlipDynamicsConstraint : public dairlib::solvers::NonlinearConstraint<T> {

 public:
  PlanarSlipDynamicsConstraint(double r0, double k, T m, int n_feet, std::vector<bool>  contact_mask, double dt, int knot_index);

 private:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

  drake::VectorX<T> CalcTimeDerivativesWithForce(
      const drake::VectorX<T>& com_position,
      const drake::VectorX<T>& com_vel,
      const drake::VectorX<T>& contact_loc) const;

  const double r0_;
  const double k_;
  const T m_;
  const int n_feet_;
  const std::vector<bool> contact_mask_;
  const double dt_;
};

///     complex_com
///     complex_ang_momentum
///     complex_lin_momentum
///     complex_contact_pos
///     complex_contact_vel
///     complex_contact_force
///     complex_gen_pos
///     complex_gen_vel
class QuadraticLiftedCost : public dairlib::solvers::NonlinearCost<double> {

  struct cost_element{
    const Eigen::MatrixXd& Q;
    Eigen::MatrixXd ref;
  };

 public:
  QuadraticLiftedCost(std::shared_ptr<PlanarSlipLifter> lifting_function,
                      cost_element  com_cost,
                      cost_element  momentum_cost,
                      cost_element  contact_cost,
                      cost_element  grf_cost,
                      cost_element  q_cost,
                      cost_element  v_cost,
                      double terminal_gain,
                      int n_slip_feet,
                      int knot_point);

 private:
  void EvaluateCost(const Eigen::Ref<const drake::VectorX<double>>& x,
                          drake::VectorX<double>* y) const override;

  double CalcCost(const cost_element& cost, const Eigen::Ref<const drake::VectorX<double>>& x);


  std::shared_ptr<PlanarSlipLifter> lifting_function_;
  const cost_element com_cost_;
  const cost_element momentum_cost_;
  const cost_element contact_cost_;
  const cost_element grf_cost_;
  const cost_element q_cost_;
  const cost_element v_cost_;
  const double terminal_gain_;
};


