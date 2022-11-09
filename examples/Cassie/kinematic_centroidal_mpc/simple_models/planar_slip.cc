#include <drake/solvers/solve.h>
#include "planar_slip.h"
#include "multibody/multibody_utils.h"
#include "multibody/kinematic/kinematic_constraints.h"

template<typename T>
PlanarSlipReductionConstraint<T>::PlanarSlipReductionConstraint(const drake::multibody::MultibodyPlant<T>& plant,
                                                                drake::systems::Context<T> *context,
                                                                const std::vector<dairlib::multibody::WorldPointEvaluator<
                                                                    T>> &slip_feet,
                                                                int complex_state_size,
                                                                T m,
                                                                int knot_index):dairlib::solvers::NonlinearConstraint<T>(
                                                                    3+3+2*2*slip_feet.size(), 3+3+2*2*slip_feet.size() + complex_state_size,
                                                                    Eigen::VectorXd::Zero(3+3+2*2*slip_feet.size()),
                                                                    Eigen::VectorXd::Zero(3+3+2*2*slip_feet.size()),
                                                                    "PlanarSlipReductionConstraint[" +
                                                                        std::to_string(knot_index) + "]"),
                                                                        context_(context),
                                                                        plant_(plant),
                                                                        m_(m),
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

  *y = drake::VectorX<T>::Zero(3+3+2*kSLIP_DIM*slip_feet_.size());

  y->head(kSLIP_DIM) = slip_com - complex_com({0,2});
  y->segment(kSLIP_DIM, kSLIP_DIM) = slip_vel * m_ - complex_lin_momentum({0,2});
  for(int i = 0; i<slip_feet_.size(); i++){
    y->segment(kSLIP_DIM+kSLIP_DIM+kSLIP_DIM * i, kSLIP_DIM) = slip_contact_pos - slip_feet_[i].EvalFull(context_)({0,2});
    y->segment(kSLIP_DIM+kSLIP_DIM+kSLIP_DIM * slip_feet_.size()+kSLIP_DIM * i, kSLIP_DIM) = slip_contact_vel - slip_feet_[i].EvalFullTimeDerivative(context_)({0,2});
  }
}

template<typename T>
PlanarSlipLifter<T>::PlanarSlipLifter(const drake::multibody::MultibodyPlant<T> &plant,
                                      drake::systems::Context<T> *context,
                                      const std::vector<dairlib::multibody::WorldPointEvaluator<T>> &slip_contact_points,
                                      const std::vector<dairlib::multibody::WorldPointEvaluator<T>> &complex_contact_points,
                                      const std::map<int, std::vector<int>>& simple_foot_index_to_complex_foot_index,
                                      const drake::VectorX<T> &nominal_stand,
                                      T m,
                                      double k,
                                      double r0,
                                      const drake::VectorX<T> &stance_widths):
    plant_(plant),
    context_(context),
    ik_(plant, context),
    slip_contact_points_(slip_contact_points),
    complex_contact_points_(complex_contact_points),
    simple_foot_index_to_complex_foot_index_(simple_foot_index_to_complex_foot_index),
    m_(m),
    k_(k),
    r0_(r0),
    stance_widths_(stance_widths),
    n_q_(plant.num_positions()),
    n_v_(plant.num_velocities()){
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q(), nominal_stand);
  com_vars_ = ik_.get_mutable_prog()->NewContinuousVariables(3);
  auto constraint = std::make_shared<drake::multibody::ComPositionConstraint>(&plant, std::nullopt, plant.world_frame(), context);
  ik_.get_mutable_prog()->AddConstraint(constraint, {ik_.q(),com_vars_});

  const auto& world_frame = plant.world_frame();
  const auto& pelvis_frame = plant.GetFrameByName("pelvis");

  ik_.AddOrientationConstraint(pelvis_frame, drake::math::RotationMatrix<double>(),
                               world_frame, drake::math::RotationMatrix<double>(), 1e-4);

  std::map<std::string, int> positions_map = dairlib::multibody::MakeNameToPositionsMap(plant);
  ik_.get_mutable_prog()->AddLinearConstraint(
      (ik_.q())(positions_map.at("hip_yaw_left")) == 0);
  ik_.get_mutable_prog()->AddLinearConstraint(
      (ik_.q())(positions_map.at("hip_yaw_right")) == 0);
}

template<typename T>
drake::VectorX<T> PlanarSlipLifter<T>::LiftGeneralizedPosition(const drake::Vector3<T> &com_position,
                                                               const drake::VectorX<T> &slip_feet_positions) {
  DRAKE_DEMAND(slip_feet_positions.size() == 2*slip_contact_points_.size());
  //Add com position constraint
  const auto com_constraint = ik_.get_mutable_prog()->AddBoundingBoxConstraint(com_position, com_position, com_vars_);
  //Add feet position constraint
  std::vector<drake::solvers::Binding<drake::solvers::Constraint>> foot_constraints;
  for(int i = 0; i < slip_contact_points_.size(); i++){
    foot_constraints.push_back(ik_.AddPositionConstraint(slip_contact_points_[i].get_frame(), drake::VectorX<T>::Zero(3), plant_.world_frame(),
                                                         {slip_feet_positions[2*i], stance_widths_[i], slip_feet_positions[2*i+1]},
                                                         {slip_feet_positions[2*i], stance_widths_[i], slip_feet_positions[2*i+1]}));
  }
  //Set initial guess for com
  ik_.get_mutable_prog()->SetInitialGuess(com_vars_,com_position);
  //Solve
  const auto result = drake::solvers::Solve(ik_.prog());
  const auto q_sol = result.GetSolution(ik_.q());
  //Normalize quaternion
  drake::VectorX<T> q_sol_normd(n_q_);
  q_sol_normd << q_sol.head(4).normalized(), q_sol.tail(n_q_ - 4);
  //Set initial guess for next time
  ik_.get_mutable_prog()->SetInitialGuess(ik_.q(), q_sol_normd);
  //Remove added constraints
  ik_.get_mutable_prog()->RemoveConstraint(com_constraint);
  for(const auto& constraint : foot_constraints){
    ik_.get_mutable_prog()->RemoveConstraint(constraint);
  }
  // TODO figure out what to do about toe
  return q_sol_normd;
}
template<typename T>
drake::VectorX<T> PlanarSlipLifter<T>::LiftGeneralizedVelocity(const drake::VectorX<T>& generalized_pos,
                                                               const drake::Vector3<T>& linear_momentum,
                                                               const drake::Vector3<T>& com_pos,
                                                               const drake::VectorX<T>& slip_feet_velocities) {
  DRAKE_DEMAND(slip_feet_velocities.size() == 2*slip_contact_points_.size());
  // Preallocate linear constraint
  drake::MatrixX<T> A(6 + 3 * slip_contact_points_.size() ,n_v_); // 6 rows for momentum, 3 rows for each slip foot
  drake::VectorX<T> b(6 + 3 * slip_contact_points_.size());

  b.segment(3 * slip_contact_points_.size(), 3) = drake::VectorX<T>::Zero(3);
  b.tail(3) = linear_momentum;

  dairlib::multibody::SetPositionsIfNew(plant_, generalized_pos, context_);


  for(int i = 0; i < slip_contact_points_.size(); i++){
    b.segment(3 * i, 3) = {slip_feet_velocities[2 * i] , 0 , slip_feet_velocities[2 * i + 1]};
    slip_contact_points_[i].EvalFullJacobian(context_ , &A.middleRows(3 * i, 3));
  }

  // Finite difference to calculate momentum jacobian
  drake::VectorX<T> x_val = drake::VectorX<T>::Zero(n_v_);
  drake::VectorX<T> yi(6);
  dairlib::multibody::SetVelocitiesIfNew(plant_,x_val,context_);
  auto y0 = plant_.CalcSpatialMomentumInWorldAboutPoint(context_, com_pos).get_coeffs();
  const T eps = 1e-7;
  for (int i = 0; i < n_v_; i++) {
    x_val(i) += eps;
    dairlib::multibody::SetVelocitiesIfNew(plant_,x_val,context_);
    x_val(i) -= eps;
    A.col(i).tail(6) = (plant_.CalcSpatialMomentumInWorldAboutPoint(context_, com_pos).get_coeffs() - y0) / eps;
  }

  // TODO figure out what to do about toe
  return A.template bdcSvd<Eigen::ComputeThinU | Eigen::ComputeThinV>().solve(b);
}

template<typename T>
drake::VectorX<T> PlanarSlipLifter<T>::LiftContactPos(const drake::VectorX<T> &generalized_position) {
  dairlib::multibody::SetPositionsIfNew(plant_, generalized_position, context_);
  drake::VectorX<T> rv(complex_contact_points_.size() * 3);
  for(int i = 0; i < complex_contact_points_.size(); i++){
    rv.segment(3 * i, 3) = complex_contact_points_[i].EvalFull(context_);
  }
  return rv;
}

template<typename T>
drake::VectorX<T> PlanarSlipLifter<T>::LiftContactVel(const drake::VectorX<T> &generalized_state) {
  dairlib::multibody::SetPositionsAndVelocitiesIfNew(plant_, generalized_state, context_);
  drake::VectorX<T> rv(complex_contact_points_.size() * 3);
  for(int i = 0; i < complex_contact_points_.size(); i++){
    rv.segment(3 * i, 3) = complex_contact_points_[i].EvalFullTimeDerivative(context_);
  }
  return rv;
}

template<typename T>
drake::VectorX<T> PlanarSlipLifter<T>::LiftGrf(const drake::VectorX<T> &com_pos,
                                               const drake::VectorX<T> &slip_feet_pos,
                                               const drake::VectorX<T> &complex_contact_point_pos) {
  drake::VectorX<T> rv(complex_contact_points_.size() * 3);

  // Loop through the slip feet
  for(int simple_index = 0; simple_index < slip_contact_points_.size(); simple_index ++){
    // Calculate the slip grf
    double r = (slip_feet_pos - com_pos({0, 2})).norm();
    double slip_grf_mag = k_ * (r - r0_);

    // Find the average location for all of the complex contact points that make up the SLIP foot
    drake::Vector3<T> average_pos = drake::VectorX<T>::Zero(3);
    const auto& complex_feet_list = simple_foot_index_to_complex_foot_index_.at(simple_index);
    for(const auto& complex_index : complex_feet_list){
      average_pos = average_pos + complex_contact_point_pos.segment(3 * complex_index, complex_index);
    }
    average_pos = average_pos/complex_feet_list.size();

    // Direction of all the grf for this slip foot must be parallel to not create internal forces
    // direction is the vector from average contact point to com, so no moment from sum of grf
    const auto dir = (com_pos - average_pos).normalized();
    // Distribute grf magnitude across all of the complex contact points
    for(const auto& complex_index : complex_feet_list){
      rv.segment(3 * complex_index, complex_index) = dir * slip_grf_mag/complex_feet_list.size();
    }
  }
  return rv;
}
