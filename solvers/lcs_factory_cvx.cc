#include "solvers/lcs_factory_cvx.h"

#include "multibody/geom_geom_collider.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "drake/solvers/moby_lcp_solver.h"

#include "drake/math/autodiff_gradient.h"



namespace dairlib {
namespace solvers {

using std::vector;
using std::set;

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

std::pair<LCS,double> LCSFactoryConvex::LinearizePlantToLCS(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    const MultibodyPlant<AutoDiffXd>& plant_ad,
    const Context<AutoDiffXd>& context_ad,
    const vector<SortedPair<GeometryId>>& contact_geoms,
    int num_friction_directions, double mu, double dt, int N) {


  /// Use Anitescu's Convex Relaxation on contact model, the complementarity
  /// constraints are imposed by the velocity cone

  int n_pos = plant_ad.num_positions();
  int n_vel = plant_ad.num_velocities();
  int n_total = plant_ad.num_positions() + plant_ad.num_velocities();
  int n_input = plant_ad.num_actuators();

  // ------------------------------------------------------------------------ //
  /// First, calculate vdot from non-contact dynamics using AutoDiff Plant
  /// manipulator equation: M * vdot + C + G = Bu + J.T * F_ext (no J_c.T * F_contact)
  /// in Drake's notation convention: M * vdot + C = tau_g + tau_app

  AutoDiffVecXd C(n_vel);
  plant_ad.CalcBiasTerm(context_ad, &C);

  AutoDiffVecXd Bu = plant_ad.MakeActuationMatrix() *
                     plant_ad.get_actuation_input_port().Eval(context_ad);

  // tau_g = -G, see the above comments on drake notation
  AutoDiffVecXd tau_g = plant_ad.CalcGravityGeneralizedForces(context_ad);

  // f_app is a drake MultibodyForces object, not an actual sptial or generalized force
  // f_app.generalized_forces() = tau_app, tau_app = J.T * F_ext
  drake::multibody::MultibodyForces<AutoDiffXd> f_app(plant_ad);
  plant_ad.CalcForceElementsContribution(context_ad, &f_app);

  MatrixX<AutoDiffXd> M(n_vel, n_vel);
  plant_ad.CalcMassMatrix(context_ad, &M);

  // solve vdot_no_contact
  // If ldlt is slow, there are alternate formulations which avoid it
  AutoDiffVecXd vdot_no_contact =
      M.ldlt().solve(tau_g + f_app.generalized_forces() + Bu - C);

  // ------------------------------------------------------------------------ //
  /// Next, calculate qdot from non-contact dynamics and its derivatives (Jacobians) AB_q
  /// AB_q can be used to derive the mapping Nq
  /// Nq is the mapping from velocity to quaternion derivative qdot = Nq v

  // solve qdot_no_contact, can directly get from current plant_ad context
  AutoDiffVecXd qdot_no_contact(n_pos);
  AutoDiffVecXd state = plant_ad.get_state_output_port().Eval(context_ad);
  AutoDiffVecXd vel = state.tail(n_vel);
  plant_ad.MapVelocityToQDot(context_ad, vel, &qdot_no_contact);
  MatrixXd AB_q = ExtractGradient(qdot_no_contact);
  MatrixXd Nq = AB_q.block(0, n_pos, n_pos, n_vel);

  // ------------------------------------------------------------------------ //
  /// Then, from vdot_no_contact get the derivatives (Jacobians) AB_v
  /// Jacobians are named AB_v_q,AB_v_v,AB_v_u for q,v,u, respectively later
  /// Also calculate the dynamics constant term d_v that would be used later

  // Jacobian of vdot_no_contact
  MatrixXd AB_v = ExtractGradient(vdot_no_contact);

  // Constant term in dynamics, d_v = vdot_no_contact - AB_v * [q, v, u]
  VectorXd d_vv = ExtractValue(vdot_no_contact);
  VectorXd inp_dvv = plant.get_actuation_input_port().Eval(context);
  VectorXd x_dvv(n_pos + n_vel + n_input);
  x_dvv << plant.GetPositions(context), plant.GetVelocities(context), inp_dvv;
  VectorXd x_dvvcomp = AB_v * x_dvv;
  VectorXd d_v = d_vv - x_dvvcomp;

  // ------------------------------------------------------------------------ //
  /// Now, calculate the contact related terms, J_n and J_t means the contact
  /// Jacobians in normal and tangential directions, respectively
  /// Note that in Anitescu's convex formulation, the contact Jacobian is not
  /// decoupled in tangential and normal, but use a combination of the two
  /// i.e. J_c = E.T * J_n + mu * J_t

  VectorXd phi(contact_geoms.size());
  MatrixXd J_n(contact_geoms.size(), n_vel);
  MatrixXd J_t(2 * contact_geoms.size() * num_friction_directions, n_vel);

  // from GeomGeomCollider (collision dectection) get contact information
  for (int i = 0; i < contact_geoms.size(); i++) {
    multibody::GeomGeomCollider collider(
        plant, contact_geoms[i]);  // deleted num_fricton_directions (check with
                                   // Michael about changes in geomgeom)
    auto [phi_i, J_i] = collider.EvalPolytope(context, num_friction_directions);

    phi(i) = phi_i;

    J_n.row(i) = J_i.row(0);
    J_t.block(2 * i * num_friction_directions, 0, 2 * num_friction_directions, n_vel)
        = J_i.block(1, 0, 2 * num_friction_directions, n_vel);
  }

  // Define block diagonal E_t containing ones to combine the contact Jacobians
  MatrixXd E_t = MatrixXd::Zero(
      contact_geoms.size(), 2 * contact_geoms.size() * num_friction_directions);
  for (int i = 0; i < contact_geoms.size(); i++) {
    E_t.block(i, i * (2 * num_friction_directions), 1,
              2 * num_friction_directions) =
        MatrixXd::Ones(1, 2 * num_friction_directions);
  };

  // Contact Jacobian for Anitescu Model
  MatrixXd J_c = E_t.transpose() * J_n + mu * J_t;

  // Also calculate M^(-1)J_c.T that would be used in the future
  auto M_ldlt = ExtractValue(M).ldlt();
  MatrixXd MinvJ_c_T = M_ldlt.solve(J_c.transpose());

  // also note that now the n_contact should be smaller since no slack variable
  // and the complementarity variable lambda now means impulse component along
  // the extreme ray of the friction cone, so each contact is 2 * firction directions
  auto n_contact = 2 * contact_geoms.size() * num_friction_directions;

  // ------------------------------------------------------------------------ //
  /// Now, formulate the LCS matrices
  /// Dynamics equations
  /// q_{k+1} = [ q_k ] + [ dt * Nq * v_{k+1} ] = [ q_k ] + [ dt * Nq * v_k ]+ [ dt * dt * Nq * AB_v * [q_k; v_k; u_k] ] + [dt * Nq * Minv * J_c.T * lam] + [ dt * dt * Nq * d_v]
  /// v_{k+1} = [ v_k ] + [ dt * AB_v * [q_k; v_k; u_k] ] + [Minv * J_c.T * lam] + [ dt * d_v ]

  /// Matrix format
  /// [ q_{k+1}; v_{k+1}] = [ I + dt * dt * Nq * AB_v_q,  dt * Nq +  dt * dt * Nq * AB_v_v ] [q_k;v_k] +   [ dt * dt * Nq * AB_v_u ] [u_k] + [ dt * Nq * Minv * J_c.T ] [lam] + [ dt * dt * Nq * dv ]
  ///                       [ dt * AB_v_q              ,  I + dt * AB_v_v                  ]           +   [ dt * AB_v_u           ]       + [ Minv * J_c.T           ]       + [ dt * d_v          ]

  MatrixXd A(n_total, n_total);
  MatrixXd B(n_total, n_input);
  MatrixXd D(n_total, n_contact);
  VectorXd d(n_total);

  MatrixXd AB_v_q = AB_v.block(0, 0, n_vel, n_pos);
  MatrixXd AB_v_v = AB_v.block(0, n_pos, n_vel, n_vel);
  MatrixXd AB_v_u = AB_v.block(0, n_total, n_vel, n_input);

  A.block(0, 0, n_pos, n_pos) =
      MatrixXd::Identity(n_pos, n_pos) + dt * dt * Nq * AB_v_q;
  A.block(0, n_pos, n_pos, n_vel) = dt * Nq + dt * dt * Nq * AB_v_v;
  A.block(n_pos, 0, n_vel, n_pos) = dt * AB_v_q;
  A.block(n_pos, n_pos, n_vel, n_vel) =
       MatrixXd::Identity(n_vel, n_vel) + dt * AB_v_v;

  B.block(0, 0, n_pos, n_input) = dt * dt * Nq * AB_v_u;
  B.block(n_pos, 0, n_vel, n_input) = dt * AB_v_u;

  D = MatrixXd::Zero(n_total, n_contact);
  D.block(0, 0, n_pos, n_contact) = dt * Nq * MinvJ_c_T;
  D.block(n_pos, 0, n_vel, n_contact) = MinvJ_c_T;

  d.head(n_pos) = dt * dt * Nq * d_v;
  d.tail(n_vel) = dt * d_v;

//    std::cout<< "D" << std::endl;
//    std::cout<< D << std::endl;


  /// Complementarity equations
  /// [ 0 ] <= [ lambda ] (PERP) [ E_t.T * phi / dt + J_c * v_{k+1} ] >= 0
  /// [ 0 ] <= [ lambda ] (PERP) [ E_t.T * phi / dt + J_c * ([ v_k ] + [ dt * AB_v * [q_k; v_k; u_k] ] + [Minv * J_c.T * lam] + [ dt * d_v ]) ]

  /// Matrix format (notice the Linearization (taylor expasion term about N_inv))
  ///  [ 0 ] <= [lambda] (PERP) [ dt * J_c * AB_v_q + E_t.T * J_n * N_inv / dt,  J_c + dt * J_c * AB_v_v ] [q_k; v_k]
  ///                             + [ dt *  J_c * AB_v_u ] * [u_k]
  ///                             + [ J_c * Minv * J_c.T] * [lam_k]
  ///                             + [E_t.T * phi / dt + dt * J_c * d_v - E_t.T * J_n * N_inv * q{*} / dt ] >= 0

  MatrixXd Nqinv = Nq.completeOrthogonalDecomposition().pseudoInverse();

  MatrixXd E(n_contact, n_total);
  MatrixXd F(n_contact, n_contact);
  MatrixXd H(n_contact, n_input);
  VectorXd c(n_contact);

//  E.block(0, 0, n_contact, n_pos) = dt * J_c * AB_v_q;
  E.block(0, 0, n_contact, n_pos) = dt * J_c * AB_v_q + E_t.transpose() * J_n * Nqinv / dt;
  E.block(0, n_pos, n_contact, n_vel) = J_c + dt * J_c * AB_v_v;

  F = J_c * MinvJ_c_T;

  H = dt * J_c * AB_v_u;

  c = E_t.transpose() * phi / dt + dt * J_c * d_v - E_t.transpose() * J_n * Nqinv * plant.GetPositions(context) / dt;


  // Scaling fact
  auto Dn = D.squaredNorm();
  auto An = A.squaredNorm();
  auto AnDn = An / Dn;

  D *= AnDn;
  E /= AnDn;
  c /= AnDn;
  H /= AnDn;


  LCS system(A, B, D, d, E, F, H, c, N);
  std::pair <LCS, double> ret (system, AnDn);
  return ret;

}


LCS LCSFactoryConvex::FixSomeModes(const LCS& other, set<int> active_lambda_inds,
                                 set<int> inactive_lambda_inds) {

    vector<int> remaining_inds;

    // Assumes constant number of contacts per index
    int n_lambda = other.F_[0].rows();

    // Need to solve for lambda_active in terms of remaining elements
    // Build temporary [F1, F2] by eliminating rows for inactive
    for (int i = 0; i < n_lambda; i++) {
        // active/inactive must be exclusive
        DRAKE_ASSERT(!active_lambda_inds.count(i) || !inactive_lambda_inds.count(i));

        // In C++20, could use contains instead of count
        if (!active_lambda_inds.count(i) && !inactive_lambda_inds.count(i)) {
                remaining_inds.push_back(i);
        }
    }

    int n_remaining = remaining_inds.size();
    int n_active = active_lambda_inds.size();

    vector<MatrixXd> A, B, D, E, F, H;
    vector<VectorXd> d, c;

    // Build selection matrices:
    // S_a selects active indices
    // S_r selects remaining indices

    MatrixXd S_a = MatrixXd::Zero(n_active, n_lambda);
    MatrixXd S_r = MatrixXd::Zero(n_remaining, n_lambda);

    for (int i = 0; i < n_remaining; i++) {
        S_r(i, remaining_inds[i]) = 1;
    }
    {
        int i = 0;
        for (auto ind_j : active_lambda_inds) {
            S_a(i, ind_j) = 1;
            i++;
        }
    }


    for (int k = 0; k < other.N_; k++) {
        Eigen::BDCSVD<MatrixXd> svd;
        svd.setThreshold(1e-5);
        svd.compute(S_a * other.F_[k] * S_a.transpose(),
                    Eigen::ComputeFullU | Eigen::ComputeFullV);

        // F_active likely to be low-rank due to friction, but that should be OK
        // MatrixXd res = svd.solve(F_ar);

        // Build new complementarity constraints
        // F_a_inv = pinv(S_a * F * S_a^T)
        // 0 <= \lambda_k \perp E_k x_k + F_k \lambda_k + H_k u_k + c_k
        // 0 = S_a *(E x + F S_a^T \lambda_a + F S_r^T \lambda_r + H_k u_k + c_k)
        // \lambda_a = -F_a_inv * (S_a F S_r^T * lambda_r + S_a E x + S_a H u + S_a c)
        //
        // 0 <= \lambda_r \perp S_r (I - F S_a^T F_a_inv S_a) E x + ...
        //                      S_r (I - F S_a^T F_a_inv S_a) F S_r^T \lambda_r + ...
        //                      S_r (I - F S_a^T F_a_inv S_a) H u + ...
        //                      S_r (I - F S_a^T F_a_inv S_a) c
        //
        // Calling L = S_r (I - F S_a^T F_a_inv S_a)S_r * other.D_[k]
        //  E_k = L E
        //  F_k = L F S_r^t
        //  H_k = L H
        //  c_k = L c
        // std::cout << S_r << std::endl << std::endl;
        // std::cout << other.F_[k] << std::endl << std::endl;
        // std::cout << other.F_[k] << std::endl << std::endl;
        // auto tmp = S_r * (MatrixXd::Identity(n_lambda, n_lambda) -
        //              other.F_[k] *  S_a.transpose());
        MatrixXd L = S_r * (MatrixXd::Identity(n_lambda, n_lambda) -
                                other.F_[k] *  S_a.transpose() * svd.solve(S_a));
        MatrixXd E_k = L * other.E_[k];
        MatrixXd F_k = L * other.F_[k] * S_r.transpose();
        MatrixXd H_k = L * other.H_[k];
        MatrixXd c_k = L * other.c_[k];

        // Similarly,
        //  A_k = A - D * S_a^T * F_a_inv * S_a * E
        //  B_k = B - D * S_a^T * F_a_inv * S_a * H
        //  D_k = D * S_r^T - D * S_a^T  * F_a_inv * S_a F S_r^T
        //  d_k = d - D * S_a^T F_a_inv * S_a * c
        //
        //  Calling P = D * S_a^T * F_a_inv * S_a
        //
        //  A_k = A - P E
        //  B_k = B - P H
        //  D_k = S_r D - P S_r^T
        //  d_k = d - P c
        MatrixXd P = other.D_[k] * S_a.transpose() * svd.solve(S_a);
        MatrixXd A_k = other.A_[k] - P * other.E_[k];
        MatrixXd B_k = other.B_[k] - P * other.H_[k];
        MatrixXd D_k = other.D_[k] * S_r.transpose() - P * S_r.transpose();
        MatrixXd d_k = other.d_[k] - P * other.c_[k];
        E.push_back(E_k);
        F.push_back(F_k);
        H.push_back(H_k);
        c.push_back(c_k);
        A.push_back(A_k);
        B.push_back(B_k);
        D.push_back(D_k);
        d.push_back(d_k);
    }
    return LCS(A, B, D, d, E, F, H, c);
}

}  // namespace solvers
}  // namespace dairlib
