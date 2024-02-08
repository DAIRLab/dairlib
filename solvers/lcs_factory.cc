#include "solvers/lcs_factory.h"

#include <iostream>

#include "multibody/geom_geom_collider.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"

#include "drake/math/autodiff_gradient.h"
#include "drake/solvers/moby_lcp_solver.h"

namespace dairlib {
namespace solvers {

using std::set;
using std::vector;

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::SortedPair;
using drake::VectorX;
using drake::geometry::GeometryId;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;

using Eigen::MatrixXd;
using Eigen::VectorXd;

std::pair<LCS, double> LCSFactory::LinearizePlantToLCS(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    const MultibodyPlant<AutoDiffXd>& plant_ad,
    const Context<AutoDiffXd>& context_ad,
    const vector<SortedPair<GeometryId>>& contact_geoms,
    int num_friction_directions, const std::vector<double>& mu, double dt,
    int N, ContactModel contact_model) {
  int n_x = plant_ad.num_positions() + plant_ad.num_velocities();
  int n_u = plant_ad.num_actuators();

  int n_contacts = contact_geoms.size();

  DRAKE_DEMAND(plant_ad.num_velocities() == plant.num_velocities());
  DRAKE_DEMAND(plant_ad.num_positions() == plant.num_positions());
  int n_v = plant.num_velocities();
  int n_q = plant.num_positions();

  AutoDiffVecXd C(n_v);
  plant_ad.CalcBiasTerm(context_ad, &C);
  VectorXd u_dyn = plant.get_actuation_input_port().Eval(context);

  auto B_dyn_ad = plant_ad.MakeActuationMatrix();
  AutoDiffVecXd Bu =
      B_dyn_ad * plant_ad.get_actuation_input_port().Eval(context_ad);

  AutoDiffVecXd tau_g = plant_ad.CalcGravityGeneralizedForces(context_ad);

  drake::multibody::MultibodyForces<AutoDiffXd> f_app(plant_ad);
  plant_ad.CalcForceElementsContribution(context_ad, &f_app);

  MatrixX<AutoDiffXd> M(n_v, n_v);
  plant_ad.CalcMassMatrix(context_ad, &M);

  // If this ldlt is slow, there are alternate formulations which avoid it
  AutoDiffVecXd vdot_no_contact =
      M.ldlt().solve(tau_g + Bu + f_app.generalized_forces() - C);
  // Constant term in dynamics, d_vv = d + A x_0 + B u_0
  VectorXd d_vv = ExtractValue(vdot_no_contact);
  // Derivatives w.r.t. x and u, AB
  MatrixXd AB_v = ExtractGradient(vdot_no_contact);
  VectorXd x_dvv(n_q + n_v + n_u);
  x_dvv << plant.GetPositions(context), plant.GetVelocities(context), u_dyn;
  VectorXd x_dvvcomp = AB_v * x_dvv;
  VectorXd d_v = d_vv - x_dvvcomp;

  ///////////
  AutoDiffVecXd x_ad = plant_ad.GetPositionsAndVelocities(context_ad);
  AutoDiffVecXd qdot_no_contact(n_q);
  AutoDiffVecXd vel_ad = x_ad.tail(n_v);

  plant_ad.MapVelocityToQDot(context_ad, vel_ad, &qdot_no_contact);
  MatrixXd AB_q = ExtractGradient(qdot_no_contact);
  MatrixXd d_q = ExtractValue(qdot_no_contact);
  Eigen::SparseMatrix<double> Nqt;
  Nqt = plant.MakeVelocityToQDotMap(context);
  MatrixXd qdotNv = MatrixXd(Nqt);

  Eigen::SparseMatrix<double> NqI;
  NqI = plant.MakeQDotToVelocityMap(context);
  MatrixXd vNqdot = MatrixXd(NqI);

  VectorXd phi(n_contacts);
  MatrixXd J_n(n_contacts, n_v);
  MatrixXd J_t(2 * n_contacts * num_friction_directions, n_v);

  for (int i = 0; i < n_contacts; i++) {
    multibody::GeomGeomCollider collider(
        plant,
        contact_geoms[i]);  // deleted num_friction_directions (check with
    // Michael about changes in geomgeom)
    auto [phi_i, J_i] = collider.EvalPolytope(context, num_friction_directions);

    phi(i) = phi_i;
    J_n.row(i) = J_i.row(0);
    J_t.block(2 * i * num_friction_directions, 0, 2 * num_friction_directions,
              n_v) = J_i.block(1, 0, 2 * num_friction_directions, n_v);
  }

  auto M_ldlt = ExtractValue(M).ldlt();
  MatrixXd MinvJ_n_T = M_ldlt.solve(J_n.transpose());
  MatrixXd MinvJ_t_T = M_ldlt.solve(J_t.transpose());

  MatrixXd A(n_x, n_x);
  MatrixXd B(n_x, n_u);
  VectorXd d(n_x);

  MatrixXd AB_v_q = AB_v.block(0, 0, n_v, n_q);
  MatrixXd AB_v_v = AB_v.block(0, n_q, n_v, n_v);
  MatrixXd AB_v_u = AB_v.block(0, n_x, n_v, n_u);
  MatrixXd M_double = MatrixXd::Zero(n_v, n_v);
  plant.CalcMassMatrix(context, &M_double);

  A.block(0, 0, n_q, n_q) =
      MatrixXd::Identity(n_q, n_q) + dt * dt * qdotNv * AB_v_q;
  A.block(0, n_q, n_q, n_v) = dt * qdotNv + dt * dt * qdotNv * AB_v_v;
  A.block(n_q, 0, n_v, n_q) = dt * AB_v_q;
  A.block(n_q, n_q, n_v, n_v) = dt * AB_v_v + MatrixXd::Identity(n_v, n_v);

  B.block(0, 0, n_q, n_u) = dt * dt * qdotNv * AB_v_u;
  B.block(n_q, 0, n_v, n_u) = dt * AB_v_u;

  d.head(n_q) = dt * dt * qdotNv * d_v;
  d.tail(n_v) = dt * d_v;

  MatrixXd E_t =
      MatrixXd::Zero(n_contacts, 2 * n_contacts * num_friction_directions);
  for (int i = 0; i < n_contacts; i++) {
    E_t.block(i, i * (2 * num_friction_directions), 1,
              2 * num_friction_directions) =
        MatrixXd::Ones(1, 2 * num_friction_directions);
  }

  int n_lambda = 0;
  if (contact_model == ContactModel::kStewartAndTrinkle) {
    n_lambda = 2 * n_contacts + 2 * n_contacts * num_friction_directions;
  } else {
    n_lambda = 2 * n_contacts * num_friction_directions;
  }

  // Matrices with contact variables
  MatrixXd D = MatrixXd::Zero(n_x, n_lambda);
  MatrixXd E = MatrixXd::Zero(n_lambda, n_x);
  MatrixXd F = MatrixXd::Zero(n_lambda, n_lambda);
  MatrixXd H = MatrixXd::Zero(n_lambda, n_u);
  VectorXd c = VectorXd::Zero(n_lambda);

  if (contact_model == ContactModel::kStewartAndTrinkle) {
    D.block(0, 2 * n_contacts, n_q, 2 * n_contacts * num_friction_directions) =
        dt * dt * qdotNv * MinvJ_t_T;
    D.block(n_q, 2 * n_contacts, n_v,
            2 * n_contacts * num_friction_directions) = dt * MinvJ_t_T;

    D.block(0, n_contacts, n_q, n_contacts) = dt * dt * qdotNv * MinvJ_n_T;

    D.block(n_q, n_contacts, n_v, n_contacts) = dt * MinvJ_n_T;
    E.block(n_contacts, 0, n_contacts, n_q) =
        dt * dt * J_n * AB_v_q + J_n * vNqdot;
    E.block(2 * n_contacts, 0, 2 * n_contacts * num_friction_directions, n_q) =
        dt * J_t * AB_v_q;
    E.block(n_contacts, n_q, n_contacts, n_v) =
        dt * J_n + dt * dt * J_n * AB_v_v;
    E.block(2 * n_contacts, n_q, 2 * n_contacts * num_friction_directions,
            n_v) = J_t + dt * J_t * AB_v_v;

    F.block(0, n_contacts, n_contacts, n_contacts) =
        Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(mu.data(),
                                                            mu.size())
            .asDiagonal();

    F.block(0, 2 * n_contacts, n_contacts,
            2 * n_contacts * num_friction_directions) = -E_t;

    F.block(n_contacts, n_contacts, n_contacts, n_contacts) =
        dt * dt * J_n * MinvJ_n_T;
    F.block(n_contacts, 2 * n_contacts, n_contacts,
            2 * n_contacts * num_friction_directions) =
        dt * dt * J_n * MinvJ_t_T;

    F.block(2 * n_contacts, 0, 2 * n_contacts * num_friction_directions,
            n_contacts) = E_t.transpose();

    F.block(2 * n_contacts, n_contacts,
            2 * n_contacts * num_friction_directions, n_contacts) =
        dt * J_t * MinvJ_n_T;
    F.block(2 * n_contacts, 2 * n_contacts,
            2 * n_contacts * num_friction_directions,
            2 * n_contacts * num_friction_directions) = dt * J_t * MinvJ_t_T;

    H.block(n_contacts, 0, n_contacts, n_u) = dt * dt * J_n * AB_v_u;
    H.block(2 * n_contacts, 0, 2 * n_contacts * num_friction_directions, n_u) =
        dt * J_t * AB_v_u;

    // TODO(yangwill): check gap function to make sure it makes sense. Potential
    // source of uncertainty
    //  std::cout << "phi: " << phi << std::endl;
    c.segment(n_contacts, n_contacts) = phi + dt * dt * J_n * d_v;
    c.segment(2 * n_contacts, 2 * n_contacts * num_friction_directions) =
        J_t * dt * d_v - J_n * vNqdot * plant.GetPositions(context);
  } else if (contact_model == ContactModel::kAnitescu) {
    VectorXd mu_vec = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
        mu.data(), mu.size());
    VectorXd anitescu_mu_vec = VectorXd::Zero(n_lambda);
    for (int i = 0; i < mu_vec.rows(); i++) {
      double cur = mu_vec(i);
      anitescu_mu_vec(4 * i) = cur;
      anitescu_mu_vec(4 * i + 1) = cur;
      anitescu_mu_vec(4 * i + 2) = cur;
      anitescu_mu_vec(4 * i + 3) = cur;
    }
    MatrixXd anitescu_mu_matrix = anitescu_mu_vec.asDiagonal();
    MatrixXd J_c = E_t.transpose() * J_n + anitescu_mu_matrix * J_t;

    MatrixXd MinvJ_c_T = M_ldlt.solve(J_c.transpose());

    D.block(0, 0, n_q, n_lambda) = dt * dt * qdotNv * MinvJ_c_T;
    D.block(n_q, 0, n_v, n_lambda) = dt * MinvJ_c_T;

    E.block(0, 0, n_lambda, n_q) =
        dt * J_c * AB_v_q + E_t.transpose() * J_n * vNqdot / dt;
    E.block(0, n_q, n_lambda, n_v) = J_c + dt * J_c * AB_v_v;

    F = dt * J_c * MinvJ_c_T;

    H = dt * J_c * AB_v_u;
    c = E_t.transpose() * phi / dt + dt * J_c * d_v -
        E_t.transpose() * J_n * vNqdot * plant.GetPositions(context) / dt;
  }

  auto Dn = D.squaredNorm();
  auto An = A.squaredNorm();
  auto AnDn = An / Dn;

  D *= AnDn;
  E /= AnDn;
  c /= AnDn;
  H /= AnDn;

  LCS system(A, B, D, d, E, F, H, c, N, dt);

  std::pair<LCS, double> ret(system, AnDn);
  return ret;
}

std::pair<Eigen::MatrixXd, std::vector<VectorXd>>
LCSFactory::ComputeContactJacobian(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::systems::Context<double>& context,
    const drake::multibody::MultibodyPlant<drake::AutoDiffXd>& plant_ad,
    const drake::systems::Context<drake::AutoDiffXd>& context_ad,
    const std::vector<drake::SortedPair<drake::geometry::GeometryId>>&
        contact_geoms,
    int num_friction_directions, const std::vector<double>& mu, double dt,
    int N, dairlib::solvers::ContactModel contact_model) {
  int n_contacts = contact_geoms.size();

  int n_v = plant.num_velocities();

  VectorXd phi(n_contacts);
  MatrixXd J_n(n_contacts, n_v);
  MatrixXd J_t(2 * n_contacts * num_friction_directions, n_v);
  std::vector<VectorXd> contact_points;
  for (int i = 0; i < n_contacts; i++) {
    multibody::GeomGeomCollider collider(
        plant,
        contact_geoms[i]);  // deleted num_friction_directions (check with
    // Michael about changes in geomgeom)
    auto [phi_i, J_i] = collider.EvalPolytope(context, num_friction_directions);
    auto [p_WCa, p_WCb] = collider.CalcWitnessPoints(context);
    // TODO(yangwill): think about if we want to push back both witness points
    contact_points.push_back(p_WCa);
    phi(i) = phi_i;
    J_n.row(i) = J_i.row(0);
    J_t.block(2 * i * num_friction_directions, 0, 2 * num_friction_directions,
              n_v) = J_i.block(1, 0, 2 * num_friction_directions, n_v);
  }

  if (contact_model == ContactModel::kStewartAndTrinkle) {
    MatrixXd J_c = MatrixXd::Zero(
        n_contacts + 2 * n_contacts * num_friction_directions, n_v);
    J_c << J_n, J_t;
    return std::make_pair(J_c, contact_points);
  } else if (contact_model == ContactModel::kAnitescu) {
    MatrixXd E_t =
        MatrixXd::Zero(n_contacts, 2 * n_contacts * num_friction_directions);
    for (int i = 0; i < n_contacts; i++) {
      E_t.block(i, i * (2 * num_friction_directions), 1,
                2 * num_friction_directions) =
          MatrixXd::Ones(1, 2 * num_friction_directions);
    }
    int n_contact_vars = 2 * n_contacts * num_friction_directions;

    VectorXd mu_vec = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
        mu.data(), mu.size());
    VectorXd anitescu_mu_vec = VectorXd::Zero(n_contact_vars);
    for (int i = 0; i < mu_vec.rows(); i++) {
      double cur = mu_vec(i);
      anitescu_mu_vec(4 * i) = cur;
      anitescu_mu_vec(4 * i + 1) = cur;
      anitescu_mu_vec(4 * i + 2) = cur;
      anitescu_mu_vec(4 * i + 3) = cur;
    }
    MatrixXd anitescu_mu_matrix = anitescu_mu_vec.asDiagonal();
    MatrixXd J_c = E_t.transpose() * J_n + anitescu_mu_matrix * J_t;
    return std::make_pair(J_c, contact_points);
  } else {
    std::cerr << ("Unknown projection type") << std::endl;
    DRAKE_THROW_UNLESS(false);
  }
}

LCS LCSFactory::FixSomeModes(const LCS& other, set<int> active_lambda_inds,
                             set<int> inactive_lambda_inds) {
  vector<int> remaining_inds;

  // Assumes constant number of contacts per index
  int n_lambda = other.F_[0].rows();

  // Need to solve for lambda_active in terms of remaining elements
  // Build temporary [F1, F2] by eliminating rows for inactive
  for (int i = 0; i < n_lambda; i++) {
    // active/inactive must be exclusive
    DRAKE_ASSERT(!active_lambda_inds.count(i) ||
                 !inactive_lambda_inds.count(i));

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
    // \lambda_a = -F_a_inv * (S_a F S_r^T * lambda_r + S_a E x + S_a H u + S_a
    // c)
    //
    // 0 <= \lambda_r \perp S_r (I - F S_a^T F_a_inv S_a) E x + ...
    //                      S_r (I - F S_a^T F_a_inv S_a) F S_r^T \lambda_r +
    //                      ... S_r (I - F S_a^T F_a_inv S_a) H u + ... S_r (I -
    //                      F S_a^T F_a_inv S_a) c
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
                        other.F_[k] * S_a.transpose() * svd.solve(S_a));
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
  return LCS(A, B, D, d, E, F, H, c, other.dt_);
}

}  // namespace solvers
}  // namespace dairlib
