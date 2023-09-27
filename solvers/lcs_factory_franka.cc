#include "solvers/lcs_factory_franka.h"

#include "multibody/geom_geom_collider.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "drake/solvers/moby_lcp_solver.h"

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

std::pair<LCS,double> LCSFactoryFranka::LinearizePlantToLCS(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    const MultibodyPlant<AutoDiffXd>& plant_ad,
    const Context<AutoDiffXd>& context_ad,
    const vector<SortedPair<GeometryId>>& contact_geoms,
    int num_friction_directions, double mu, float dt) {


    // std::cout<<"contact pairs size  "<<contact_geoms.size()<<std::endl;
  ///
  /// First, calculate vdot and derivatives from non-contact dynamics
  ///

  AutoDiffVecXd C(plant.num_velocities());

  plant_ad.CalcBiasTerm(context_ad, &C);

  AutoDiffVecXd Bu = plant_ad.MakeActuationMatrix() *
                     plant_ad.get_actuation_input_port().Eval(context_ad);



  AutoDiffVecXd tau_g = plant_ad.CalcGravityGeneralizedForces(context_ad);

  drake::multibody::MultibodyForces<AutoDiffXd> f_app(plant_ad);
  plant_ad.CalcForceElementsContribution(context_ad, &f_app);

  MatrixX<AutoDiffXd> M(plant.num_velocities(), plant.num_velocities());
  plant_ad.CalcMassMatrix(context_ad, &M);



  // If this ldlt is slow, there are alternate formulations which avoid it
  AutoDiffVecXd vdot_no_contact =
      M.ldlt().solve(tau_g + f_app.generalized_forces() + Bu - C);

  // Derivatives w.r.t. x and u, AB
  MatrixXd AB_v = ExtractGradient(vdot_no_contact);

  // Constant term in dynamics
  VectorXd d_vv = ExtractValue(vdot_no_contact);

  VectorXd inp_dvv = plant.get_actuation_input_port().Eval(context);
  VectorXd x_dvv(plant.num_positions() + plant.num_velocities() + plant.num_actuators());
  x_dvv << plant.GetPositions(context), plant.GetVelocities(context), inp_dvv;
  VectorXd x_dvvcomp = AB_v * x_dvv;

  VectorXd d_v = d_vv - x_dvvcomp;


  int n_state = plant_ad.num_positions();
  int n_vel = plant_ad.num_velocities();
  int n_total = plant_ad.num_positions() + plant_ad.num_velocities();
  int n_input = plant_ad.num_actuators();

  AutoDiffVecXd qdot_no_contact(plant.num_positions());

  AutoDiffVecXd state = plant_ad.get_state_output_port().Eval(context_ad);

  AutoDiffVecXd vel = state.tail(n_vel);

  plant_ad.MapVelocityToQDot(context_ad, vel, &qdot_no_contact);

  MatrixXd AB_q = ExtractGradient(qdot_no_contact);

  MatrixXd d_q = ExtractValue(qdot_no_contact);

  MatrixXd Nq = AB_q.block(0, n_state, n_state, n_vel);

  ///(FIXING BUG - ALP (let me know if there are issues))
  Eigen::MatrixXd NqInverse = Nq.completeOrthogonalDecomposition().pseudoInverse();

  ///
  /// Contact-related terms
  ///
//   std::cout<<"contact geoms size : "<<contact_geoms.size()<<std::endl;

  VectorXd phi(contact_geoms.size());
  MatrixXd J_n(contact_geoms.size(), plant.num_velocities());
  MatrixXd J_t(2 * contact_geoms.size() * num_friction_directions,
               plant.num_velocities());

  for (int i = 0; i < contact_geoms.size(); i++) {
    multibody::GeomGeomCollider collider(
        plant, contact_geoms[i]);  // deleted num_fricton_directions (check with
                                   // Michael about changes in geomgeom)
    auto [phi_i, J_i] = collider.EvalPolytope(context, num_friction_directions);

    phi(i) = phi_i; //distance between contact pair

    J_n.row(i) = J_i.row(0);
    J_t.block(2 * i * num_friction_directions, 0, 2 * num_friction_directions,
              plant.num_velocities()) =
        J_i.block(1, 0, 2 * num_friction_directions, plant.num_velocities());
  }


  auto M_ldlt = ExtractValue(M).ldlt();
  MatrixXd MinvJ_n_T = M_ldlt.solve(J_n.transpose());
  MatrixXd MinvJ_t_T = M_ldlt.solve(J_t.transpose());

  auto n_contact = 2 * contact_geoms.size() +
                   2 * contact_geoms.size() * num_friction_directions;


  /// Dynamics equations
  /// q_{k+1} = [ q_k ] + [ dt * Nq * v_{k+1} ] = [ q_k ] + [ dt * Nq * v_k ]+ [ dt * dt * Nq * AB_v * [q_k; v_k; u_k] ] + [ dt * dt * Nq * Minv * Jt^T * lam_t ] + [ dt * dt * Nq * Minv * Jn^T lam_n] + [ dt * dt * Nq * d_v]
  /// v_{k+1} = [ v_k ] + [ dt * AB_v * [q_k; v_k; u_k] ] + [ dt * Minv * Jt^T * lam_t ] + [ dt * Minv * Jn^T * lam_n ] + [ dt * d_v ]
  ///

  /// Matrix format
  /// [ q_{k+1}; v_{k+1}] = [ I + dt * dt * Nq * AB_v_q,  dt * Nq +  dt * dt * Nq * AB_v_v ] [q_k;v_k] +   [ dt * dt * Nq * AB_v_u ] [u_k] + [ 0, dt * dt * Nq * Minv * Jn^T, dt * dt * Nq * Minv * Jt^T ] [gamma lam_n lam_t] + [ dt * dt * Nq * dv ]
  ///                       [ dt * AB_v_q              ,  I + dt * AB_v_v                  ]           +   [ dt * AB_v_u           ]       + [ 0, dt * Minv * Jn^T          , dt * Minv * Jt^T           ]                     + [ dt * d_v          ]
  ///

  MatrixXd A(n_total, n_total);
  MatrixXd B(n_total, n_input);
  MatrixXd D(n_total, n_contact);
  VectorXd d(n_total);
  MatrixXd E(n_contact, n_total);
  MatrixXd F(n_contact, n_contact);
  MatrixXd H(n_contact, n_input);
  VectorXd c(n_contact);

  MatrixXd AB_v_q = AB_v.block(0, 0, n_vel, n_state);
  MatrixXd AB_v_v = AB_v.block(0, n_state, n_vel, n_vel);
  MatrixXd AB_v_u = AB_v.block(0, n_total, n_vel, n_input);

  A.block(0, 0, n_state, n_state) =
      MatrixXd::Identity(n_state, n_state) + dt * dt * Nq * AB_v_q;
  A.block(0, n_state, n_state, n_vel) = dt * Nq + dt * dt * Nq * AB_v_v;
  A.block(n_state, 0, n_vel, n_state) = dt * AB_v_q;
  A.block(n_state, n_state, n_vel, n_vel) =
      dt * AB_v_v + MatrixXd::Identity(n_vel, n_vel);

  B.block(0, 0, n_state, n_input) = dt * dt * Nq * AB_v_u;
  B.block(n_state, 0, n_vel, n_input) = dt * AB_v_u;

  D = MatrixXd::Zero(n_total, n_contact);
  D.block(0, 2 * contact_geoms.size(), n_state,
          2 * contact_geoms.size() * num_friction_directions) =
      dt * dt * Nq * MinvJ_t_T;
  D.block(n_state, 2 * contact_geoms.size(), n_vel,
          2 * contact_geoms.size() * num_friction_directions) = dt * MinvJ_t_T;

  D.block(0, contact_geoms.size(), n_state, contact_geoms.size() )  =  dt * dt * Nq * MinvJ_n_T;

  D.block(n_state, contact_geoms.size(), n_vel, contact_geoms.size() ) = dt * MinvJ_n_T;

  d.head(n_state) = dt * dt * Nq * d_v;
  d.tail(n_vel) = dt * d_v;


  /// Complementarity equations
  /// [ 0 ] <= [ gamma (PERP) mu * lam_n - E lam_t ] >= 0
  /// [ 0 ] <= [ lam_n (PERP) phi + J_n * v_{k+1} * dt ] >= 0
  /// [ 0 ] <= [ lam_t (PERP) E^T * gamma + J_t v_{k+1} ] >= 0
  ///

  /// Matrix format
  ///  [ 0 ] <= gamma (PERP) [ 0                     ,  0                                ] [q_k; v_k] + [ 0  , mu                          , -E                         ] [gamma; lambda_n; lambda_t] + [ 0                      ] [u_k] + [ 0                         ]
  ///  [ 0 ] <= lam_n (PERP) [ dt * dt * J_n * AB_v_q, dt * J_n + dt * dt * J_n * AB_v_v ]            + [ 0  , dt * dt * J_n * Minv * J_n^T, dt * dt * J_n Minv * J_t^T ]                             + [ dt * dt * J_n * AB_v_u ]       + [ phi + dt * dt * J_n * d_v ]
  ///  [ 0 ] <= lam_t (PERP) [ dt * J_t * AB_v_q     , J_t + dt * Jt * AB_v_v            ]            + [ E^T, dt * J_t * Minv * J_n^T     , dt * J_t * Minv * J_t^T    ]                             + [ dt * J_t * AB_v_u      ]       + [ J_t * dt * d_v            ]
  ///


  E = MatrixXd::Zero(n_contact, n_total);
  E.block(contact_geoms.size(), 0, contact_geoms.size(), n_state) =
      dt * dt * J_n * AB_v_q + J_n * NqInverse;
  E.block(2 * contact_geoms.size(), 0,
          2 * contact_geoms.size() * num_friction_directions, n_state) =
      dt * J_t * AB_v_q;
  E.block(contact_geoms.size(), n_state, contact_geoms.size(), n_vel) =
      dt * J_n + dt * dt * J_n * AB_v_v;
  E.block(2 * contact_geoms.size(), n_state,
          2 * contact_geoms.size() * num_friction_directions, n_vel) =
      J_t + dt * J_t * AB_v_v;


  ///E tangential
  MatrixXd E_t = MatrixXd::Zero(
      contact_geoms.size(), 2 * contact_geoms.size() * num_friction_directions);
  for (int i = 0; i < contact_geoms.size(); i++) {
    E_t.block(i, i * (2 * num_friction_directions), 1,
              2 * num_friction_directions) =
        MatrixXd::Ones(1, 2 * num_friction_directions);
  };

  F = MatrixXd::Zero(n_contact, n_contact);
  F.block(0, contact_geoms.size(), contact_geoms.size(), contact_geoms.size()) =
      mu * MatrixXd::Identity(contact_geoms.size(), contact_geoms.size());
  F.block(0, 2 * contact_geoms.size(), contact_geoms.size(),
          2 * contact_geoms.size() * num_friction_directions) = -E_t;

  F.block(contact_geoms.size(), contact_geoms.size(), contact_geoms.size(), contact_geoms.size() ) = dt * dt * J_n * MinvJ_n_T;

  F.block(contact_geoms.size(), 2 * contact_geoms.size(), contact_geoms.size(),
          2 * contact_geoms.size() * num_friction_directions) =
      dt * dt * J_n * MinvJ_t_T;

  F.block(2 * contact_geoms.size(), 0,
          2 * contact_geoms.size() * num_friction_directions,
          contact_geoms.size()) = E_t.transpose();

  F.block(2 * contact_geoms.size(), contact_geoms.size(),  2 * contact_geoms.size() * num_friction_directions, contact_geoms.size() ) = dt * J_t * MinvJ_n_T;

  F.block(2 * contact_geoms.size(), 2 * contact_geoms.size(),
          2 * contact_geoms.size() * num_friction_directions,
          2 * contact_geoms.size() * num_friction_directions) =
      dt * J_t * MinvJ_t_T;

  H = MatrixXd::Zero(n_contact, n_input);
  H.block(contact_geoms.size(), 0, contact_geoms.size(), n_input) =
      dt * dt * J_n * AB_v_u;
  H.block(2 * contact_geoms.size(), 0,
          2 * contact_geoms.size() * num_friction_directions, n_input) =
      dt * J_t * AB_v_u;


  c = VectorXd::Zero(n_contact);
  c.segment(contact_geoms.size(), contact_geoms.size()) =
      phi + dt * dt * J_n * d_v - J_n * NqInverse * plant.GetPositions(context);
  c.segment(2 * contact_geoms.size(),
            2 * contact_geoms.size() * num_friction_directions) =
      J_t * dt * d_v;

  int N = 5;

  auto Dn = D.squaredNorm();
  auto An = A.squaredNorm();
  auto AnDn = An / Dn;

//  std::cout << "quick check on AnDn" << std::endl;
//  std::cout << D << std::endl;



  std::vector<MatrixXd> A_lcs(N, A);
  std::vector<MatrixXd> B_lcs(N, B);
  std::vector<MatrixXd> D_lcs(N, D * AnDn);
  std::vector<VectorXd> d_lcs(N, d );
  std::vector<MatrixXd> E_lcs(N, E / AnDn);
  std::vector<MatrixXd> F_lcs(N, F);
  std::vector<VectorXd> c_lcs(N, c / AnDn);
  std::vector<MatrixXd> H_lcs(N, H / AnDn);

  LCS system(A_lcs, B_lcs, D_lcs, d_lcs, E_lcs, F_lcs, H_lcs, c_lcs);

  std::pair <LCS, double> ret (system, AnDn);

  return ret;

}

}  // namespace solvers
}  // namespace dairlib
