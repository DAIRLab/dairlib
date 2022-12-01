#include <iostream>
#include "planar_slip_reducer.h"
#include "multibody/multibody_utils.h"

PlanarSlipReducer::PlanarSlipReducer(const drake::multibody::MultibodyPlant<double> &plant,
                                     drake::systems::Context<double> *context,
                                     const std::vector<dairlib::multibody::WorldPointEvaluator<double>> &slip_contact_points,
                                     const std::vector<dairlib::multibody::WorldPointEvaluator<double>> &complex_contact_points,
                                     const std::map<int, std::vector<int>> &simple_foot_index_to_complex_foot_index,
                                     double k,
                                     double r0):plant_(plant),
                                                context_(context),
                                                k_(k),
                                                r0_(r0),
                                                m_(plant.CalcTotalMass(*context)),
                                                slip_contact_points_(slip_contact_points),
                                                complex_contact_points_(complex_contact_points),
                                                simple_foot_index_to_complex_foot_index_(simple_foot_index_to_complex_foot_index),
                                                n_q_(plant.num_positions()),
                                                n_v_(plant.num_velocities()) {}
/// Input is of the form:
///     complex_com
///     complex_ang_momentum
///     complex_lin_momentum
///     complex_contact_pos
///     complex_contact_vel
///     complex_contact_force
///     complex_gen_pos
///     complex_gen_vel
/// Output is of the form:
///     slip_com
///     slip_velocity
///     slip_contact_pos
///     slip_contact_vel
///     slip_force
void PlanarSlipReducer::Reduce(const Eigen::Ref<const drake::VectorX<double>> &complex_state,
                               drake::VectorX<double> *slip_state) const {
  const auto& complex_com = complex_state.segment(0, 3);
  const auto& complex_ang_momentum = complex_state.segment(3, 3);
  const auto& complex_lin_momentum = complex_state.segment( 3 + 3, 3);
  const auto& complex_grf = complex_state.segment( 3 + 3 + 3 + 2 * 3 * complex_contact_points_.size(), 3 * complex_contact_points_.size());
  const auto& complex_gen_state = complex_state.tail(n_q_+n_v_);

  dairlib::multibody::SetPositionsAndVelocitiesIfNew<double>(plant_, complex_gen_state, context_);
  slip_state->head(kSLIP_DIM) = complex_com;
  slip_state->segment(kSLIP_DIM, kSLIP_DIM) = complex_lin_momentum/m_;
  for(int i = 0; i<slip_contact_points_.size(); i++){
    slip_state->segment(kSLIP_DIM+kSLIP_DIM+kSLIP_DIM * i, kSLIP_DIM) = slip_contact_points_[i].EvalFull(*context_);
    slip_state->segment(kSLIP_DIM+kSLIP_DIM+kSLIP_DIM * slip_contact_points_.size()+kSLIP_DIM * i, kSLIP_DIM) = slip_contact_points_[i].EvalFullTimeDerivative(*context_);
  }
  slip_state->tail(slip_contact_points_.size()) = ReduceGrf(complex_com, slip_state->segment(kSLIP_DIM+kSLIP_DIM, slip_contact_points_.size()+kSLIP_DIM), complex_grf);
}

drake::VectorX<double> PlanarSlipReducer::Reduce(const Eigen::Ref<const drake::VectorX<double>> &complex_state) const {
  drake::VectorX<double> slip_state(kSLIP_DIM + kSLIP_DIM + 2 * kSLIP_DIM * slip_contact_points_.size() + slip_contact_points_.size());
  Reduce(complex_state, &slip_state);
  return slip_state;
}

drake::VectorX<double> PlanarSlipReducer::ReduceGrf(const Eigen::Ref<const drake::VectorX<double>> &complex_com,
                                                    const Eigen::Ref<const drake::VectorX<double>> &slip_contact_pos,
                                                    const Eigen::Ref<const drake::VectorX<double>> &complex_grf)const {
  Eigen::VectorXd slip_force(slip_contact_points_.size());
  for(int i = 0; i<slip_contact_points_.size(); i++){
    const double r = (complex_com - slip_contact_pos.segment(3 * i, 3)).norm();
    const double spring_force = k_ * (r0_ - r);
    auto complex_feet_it = simple_foot_index_to_complex_foot_index_.find(i);
    Eigen::Vector3d complex_force = Eigen::Vector3d::Zero(3);
    for(const auto complex_index : complex_feet_it->second){
      complex_force = complex_force + complex_grf.segment(3 * complex_index, 3);
    }
    slip_force.coeffRef(i) = complex_force.norm() - spring_force;
  }
  return slip_force;
}
