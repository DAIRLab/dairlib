#include <iostream>
#include "planar_slip.h"
#include "multibody/multibody_utils.h"

template<typename T>
PlanarSlipReductionConstraint<T>::PlanarSlipReductionConstraint(const drake::multibody::MultibodyPlant<T>& plant,
                                                                drake::systems::Context<T> *context,
                                                                const std::vector<dairlib::multibody::WorldPointEvaluator<
                                                                    T>> &slip_feet,
                                                                int complex_state_size,
                                                                int knot_index):dairlib::solvers::NonlinearConstraint<T>(
                                                                    3+3+2*2*slip_feet.size(), 3+3+2*2*slip_feet.size() + complex_state_size,
                                                                    Eigen::VectorXd::Zero(3+3+2*2*slip_feet.size()),
                                                                    Eigen::VectorXd::Zero(3+3+2*2*slip_feet.size()),
                                                                    "PlanarSlipReductionConstraint[" +
                                                                        std::to_string(knot_index) + "]"),
                                                                        m_(plant.CalcTotalMass(*context)),
                                                                        context_(context),
                                                                        plant_(plant),
                                                                        nx_(plant.num_positions()+plant.num_velocities()),
                                                                        slip_feet_(slip_feet)
                                                                    {}


/// Input is of the form:
///     slip_com
///     slip_velocity
///     slip_contact_pos
///     slip_contact_vel
///     complex_com
///     complex_ang_momentum
///     complex_lin_momentum
///     complex_contact_pos
///     complex_contact_vel
///     complex_contact_force
///     complex_gen_pos
///     complex_gen_vel
template<typename T>
void PlanarSlipReductionConstraint<T>::EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>> &x,
                                                          drake::VectorX<T> *y) const {
  const auto& slip_com = x.head(kSLIP_DIM);
  const auto& slip_vel = x.segment(kSLIP_DIM, kSLIP_DIM);
  const auto& slip_contact_pos = x.segment(kSLIP_DIM + kSLIP_DIM, kSLIP_DIM * slip_feet_.size());
  const auto& slip_contact_vel = x.segment(kSLIP_DIM + kSLIP_DIM + kSLIP_DIM * slip_feet_.size(), kSLIP_DIM * slip_feet_.size());
  const auto& complex_com = x.segment(kSLIP_DIM + kSLIP_DIM + kSLIP_DIM * slip_feet_.size() + kSLIP_DIM * slip_feet_.size(), 3);
  const auto& complex_ang_momentum = x.segment(kSLIP_DIM + kSLIP_DIM + kSLIP_DIM * slip_feet_.size() + kSLIP_DIM * slip_feet_.size() + 3, 3);
  const auto& complex_lin_momentum = x.segment(kSLIP_DIM + kSLIP_DIM + kSLIP_DIM * slip_feet_.size() + kSLIP_DIM * slip_feet_.size() + 3 + 3, 3);
  const auto& complex_gen_state = x.tail(nx_);

  dairlib::multibody::SetPositionsAndVelocitiesIfNew<T>(plant_, complex_gen_state,  context_);

  *y = drake::VectorX<T>::Zero(2+2+2*kSLIP_DIM*slip_feet_.size());
  y->head(kSLIP_DIM) = slip_com - slip_index_.unaryExpr(complex_com);

  y->segment(kSLIP_DIM, kSLIP_DIM) = slip_vel * m_ - slip_index_.unaryExpr(complex_lin_momentum);
  for(int i = 0; i<slip_feet_.size(); i++){
    y->segment(kSLIP_DIM+kSLIP_DIM+kSLIP_DIM * i, kSLIP_DIM) = slip_contact_pos.segment(kSLIP_DIM * i, kSLIP_DIM) - slip_index_.unaryExpr(slip_feet_[i].EvalFull(*context_));
    y->segment(kSLIP_DIM+kSLIP_DIM+kSLIP_DIM * slip_feet_.size()+kSLIP_DIM * i, kSLIP_DIM) = slip_contact_vel.segment(kSLIP_DIM * i, kSLIP_DIM) - slip_index_.unaryExpr(slip_feet_[i].EvalFullTimeDerivative(*context_));
  }
}

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS (class PlanarSlipReductionConstraint);
