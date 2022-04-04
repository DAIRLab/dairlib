#include "solvers/lcs_factory.h"

#include "multibody/geom_geom_collider.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"

#include "drake/math/autodiff_gradient.h"

namespace dairlib {
namespace solvers {

using std::vector;

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::SortedPair;
using drake::geometry::GeometryId;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

using Eigen::MatrixXd;
using Eigen::VectorXd;

LCS LCSFactory::LinearizePlantToLCS(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    const MultibodyPlant<AutoDiffXd>& plant_ad,
    const Context<AutoDiffXd>& context_ad,
    const vector<SortedPair<GeometryId>>& contact_geoms,
    int num_friction_directions, double mu) {

  ///
  /// First, calculate vdot and derivatives from non-contact dynamcs
  ///



  AutoDiffVecXd C(plant.num_velocities());

  //0std::cout << ExtractGradient(C).cols() << std::endl;

  plant_ad.CalcBiasTerm(context_ad, &C);

  //std::cout << ExtractGradient(C).cols() << std::endl;

  AutoDiffVecXd Bu = plant_ad.MakeActuationMatrix() *
                     plant_ad.get_actuation_input_port().Eval(context_ad);

  /*
  auto u = plant.get_actuation_input_port().Eval(context);
  auto u_ad = drake::math::InitializeAutoDiff(u);
  AutoDiffVecXd Bu = plant_ad.MakeActuationMatrix() * u_ad;
  */

  AutoDiffVecXd tau_g = plant_ad.CalcGravityGeneralizedForces(context_ad);

  drake::multibody::MultibodyForces<AutoDiffXd> f_app(plant_ad);
  plant_ad.CalcForceElementsContribution(context_ad, &f_app);

  MatrixX<AutoDiffXd> M(plant.num_velocities(), plant.num_velocities());
  plant_ad.CalcMassMatrix(context_ad, &M);

  // If this ldlt is slow, there are alternate formulations which avoid it
  AutoDiffVecXd vdot_no_contact =
      M.ldlt().solve(tau_g + f_app.generalized_forces() + Bu - C);

  // Constant term in dynamics, d
  VectorXd d_v = ExtractValue(vdot_no_contact);

  // Derivatives w.r.t. x and u, AB
  MatrixXd AB_v = ExtractGradient(vdot_no_contact);

  int n_state = plant_ad.num_positions();
  int n_vel = plant_ad.num_velocities();
  int n_total = plant_ad.num_positions() + plant_ad.num_velocities();
  int n_input = plant_ad.num_actuators();

  //std::cout << n_state << std::endl;
  //std::cout << n_vel << std::endl;

  ///////////
  AutoDiffVecXd qdot_no_contact(plant.num_positions());
  AutoDiffVecXd state = plant_ad.get_state_output_port().Eval(context_ad);
  AutoDiffVecXd vel = state.tail(n_vel);
  //std::cout << std::size(state) << std::endl;
  plant_ad.MapVelocityToQDot(context_ad, vel, &qdot_no_contact);
  MatrixXd AB_q = ExtractGradient(qdot_no_contact);
  MatrixXd d_q = ExtractValue(qdot_no_contact);

  //std::cout << AB_q.rows() << "x" << AB_q.cols() << std::endl;

  //std::cout << AB_q.block(0,n_state, n_state, n_vel) << std::endl;

  MatrixXd Nq = AB_q.block(0,n_state, n_state, n_vel);

  ///////////


  ///
  /// Contact-related terms
  ///
  VectorXd phi(contact_geoms.size());
  MatrixXd J_n(contact_geoms.size(), plant.num_velocities());
  MatrixXd J_t(2 * contact_geoms.size() * num_friction_directions,
               plant.num_velocities());
  for (int i = 0; i < contact_geoms.size(); i++) {
    multibody::GeomGeomCollider collider(plant, contact_geoms[i],
                                         num_friction_directions);
    auto [phi_i, J_i] = collider.Eval(context);
    phi(i) = phi_i;
    J_n.row(i) = J_i.row(0);
    J_t.block(2 * i * num_friction_directions, 0, 2 * num_friction_directions,
              plant.num_velocities()) =
        J_i.block(1, 0, 2 * num_friction_directions, plant.num_velocities());
  }

  auto M_ldlt = ExtractValue(M).ldlt();
  MatrixXd MinvJ_n_T = M_ldlt.solve(J_n.transpose());
  MatrixXd MinvJ_t_T = M_ldlt.solve(J_t.transpose());

  ///std::cout << MinvJ_t_T.cols() << std::endl;

  float dt = 0.1;
  auto n_contact = 2 * contact_geoms.size() + 2 * contact_geoms.size() * num_friction_directions;

  //std::cout << "nc:" << n_contact  << std::endl;

  MatrixXd A(n_total,n_total);
  MatrixXd B(n_total, n_input);
  MatrixXd D(n_total, n_contact);
  VectorXd d(n_total);
  MatrixXd E(n_contact, n_total);
  MatrixXd F(n_contact, n_contact);
  MatrixXd H(n_contact, n_input);
  VectorXd c(n_contact);

  MatrixXd AB_v_q = AB_v.block(0,0,n_vel,n_state);
  MatrixXd AB_v_v = AB_v.block(0,n_state, n_vel, n_vel);
  MatrixXd AB_v_u = AB_v.block(0,n_total, n_vel, n_input);

  A.block(0,0,n_state,n_state) = MatrixXd::Identity(n_state,n_state) + dt * dt * Nq * AB_v_q;
  A.block(0,n_state,n_state,n_vel) = dt * Nq + dt * dt * Nq * AB_v_v;
  A.block(n_state,0,n_vel,n_state) = dt * AB_v_q;
  A.block(n_state,n_state, n_vel, n_vel) = dt * AB_v_v + MatrixXd::Identity(n_vel,n_vel);

  //std::cout << "A:" << A.rows() << 'x' << A.cols() << std::endl;

  B.block(0,0,n_state,n_input) = dt * dt * Nq * AB_v_u;
  B.block(n_state,0,n_vel,n_input) = dt * AB_v_u;

  //std::cout << "B:" <<  B.rows() << 'x' << B.cols() << std::endl;

  D = MatrixXd::Zero(n_total,n_contact);
  D.block(0, 2 * contact_geoms.size(), n_state, 2 * contact_geoms.size() * num_friction_directions) = dt * dt * Nq *MinvJ_t_T;
  D.block(n_state,2 * contact_geoms.size(), n_vel, 2 * contact_geoms.size() * num_friction_directions) = dt * MinvJ_t_T;

  //std::cout << "D:" <<  D.rows() << 'x' << D.cols() << std::endl;

  //std::cout << "D:" <<  D << std::endl;

  d.head(n_state) = dt * dt * Nq * d_v;
  d.tail(n_vel) = dt * d_v;

  //std::cout << "d:" <<  d.rows() << 'x' << d.cols() << std::endl;

  E = MatrixXd::Zero(n_contact,n_total);
  E.block(contact_geoms.size(), 0, contact_geoms.size(), n_state ) = dt * dt * J_n * AB_v_q;
  E.block(2 * contact_geoms.size(), 0, 2 * contact_geoms.size() * num_friction_directions, n_state ) = dt * J_t * AB_v_q;
  E.block(contact_geoms.size(), n_state, contact_geoms.size(), n_vel) =  dt * J_n + dt * dt * J_n * AB_v_v;
  E.block(2 * contact_geoms.size(), n_state, 2 * contact_geoms.size() * num_friction_directions, n_vel) = J_t + dt * J_t * AB_v_v;

  //std::cout << "E:" <<  E.rows() << 'x' << E.cols() << std::endl;
  //std::cout << "E:" <<  E << std::endl;

  MatrixXd E_t = MatrixXd::Zero(contact_geoms.size(), 2 * contact_geoms.size() * num_friction_directions);
  for (int i = 0; i < contact_geoms.size(); i++) {
      E_t.block(i,i*(2*num_friction_directions),1,2*num_friction_directions) = MatrixXd::Ones(1,2*num_friction_directions);
  };

  //std::cout <<  E_t << std::endl;

  F = MatrixXd::Zero(n_contact,n_contact);
  F.block(0,contact_geoms.size(), contact_geoms.size(), contact_geoms.size()) = mu * MatrixXd::Identity(contact_geoms.size(),contact_geoms.size());
  F.block(0, 2 * contact_geoms.size(), contact_geoms.size(), 2 * contact_geoms.size() * num_friction_directions) = -E_t;
  F.block(contact_geoms.size(), 2*contact_geoms.size(), contact_geoms.size(),2 * contact_geoms.size() * num_friction_directions ) = dt * dt * J_n * MinvJ_t_T;
  F.block(2*contact_geoms.size(), 0, 2 * contact_geoms.size() * num_friction_directions, contact_geoms.size()) = E_t.transpose();
  F.block(2*contact_geoms.size(), 2*contact_geoms.size(), 2 * contact_geoms.size() * num_friction_directions, 2 * contact_geoms.size() * num_friction_directions) = dt * J_t * MinvJ_t_T;

  //std::cout <<  F << std::endl;

  H = MatrixXd::Zero(n_contact,n_input);
  H.block(contact_geoms.size(),0, contact_geoms.size(), n_input) = dt * dt * J_n * AB_v_u;
  H.block(2*contact_geoms.size(),0, 2 * contact_geoms.size() * num_friction_directions, n_input) = dt * J_t * AB_v_u;

  //std::cout << H << std::endl;

  c = VectorXd::Zero(n_contact);
  c.segment(contact_geoms.size(), contact_geoms.size()) = phi + dt * dt * J_n * d_v;
  c.segment(2*contact_geoms.size(), 2 * contact_geoms.size() * num_friction_directions ) = J_t * dt * d_v;

  //std::cout << c << std::endl;

  /// TODO: finish by assembling the terms computed above into the LCS structure
  ///       and returning an LCS object
  ///  v_{k+1} = v_k + dt * (AB_V * [x; u] + Minv*J_T*lambda)
  ///  q_{k+1} = q_k + dt * Nq * v_{k+1}
  ///  phi_{k+1} = phi + J_n * dt * v_{k+1}
  ///  tangential velocity: J_t * v_{k+1}

  int N = 10;

  LCS system(A, B, D, d, E, F, H, c, N);

  return system;

}

}  // namespace solvers
}  // namespace dairlib
