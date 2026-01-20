#include "solvers/elastoplastic_lcs_factory.h"

namespace dairlib {
namespace solvers {

using drake::AutoDiffVecXd;
using drake::AutoDiffXd;
using drake::MatrixX;
using drake::math::ExtractGradient;
using drake::math::ExtractValue;
using Eigen::Matrix2Xi;
using Eigen::Matrix3Xd;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

LCS PlasticNetworkLCSFactory::ToLCS(
    const MultibodyPlant<double>& plant, const Context<double>& context,
    const MultibodyPlant<AutoDiffXd>& plant_ad,
    const Context<AutoDiffXd>& context_ad,
    const vector<SortedPair<GeometryId>>& external_contact_geoms,
    const vector<SortedPair<GeometryId>>& internal_contact_geoms,
    const VectorXd& yield_forces, const vector<double>& mu, const double& dt,
    const int& N, int n_lambda_with_tangential,
    const vector<int>& num_friction_directions_per_contact,
    const vector<int>& starting_index_per_contact_in_lambda_t_vector,
    ContactModel contact_model) {
  // First, build the LCS without considering internal plasticity forces.
  LCS lcs_without_plasticity = LCSFactory::LinearizePlantToLCS(
      plant, context, plant_ad, context_ad, external_contact_geoms, mu, dt, N,
      n_lambda_with_tangential, num_friction_directions_per_contact,
      starting_index_per_contact_in_lambda_t_vector, contact_model);

  DRAKE_DEMAND(yield_forces.size() == internal_contact_geoms.size());

  // Dimensions.
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  int n_x = n_q + n_v;
  int n_u = plant.num_actuators();
  int n_lambda = lcs_without_plasticity.E_[0].rows();
  int n_external_contacts = external_contact_geoms.size();
  int n_internal_contacts = internal_contact_geoms.size();
  int n_sigma = n_internal_contacts * 2;  // Number of plasticity variables
  int n_lambda_internal = n_internal_contacts * 3;

  // Compute some relevant quantities.
  // TODO (@bibit): This has a lot of repeated computations; consider not
  // reusing LCSFactory:LinearizeToLCS and instead replicating the functionality
  // here.  This would duplicate the code but not the run-time computation.
  Eigen::SparseMatrix<double> Nqt;
  Nqt = plant.MakeVelocityToQDotMap(context);
  MatrixXd qdotNv = MatrixXd(Nqt);
  AutoDiffVecXd C(n_v);
  plant_ad.CalcBiasTerm(context_ad, &C);
  VectorXd u_dyn = plant.get_actuation_input_port().Eval(context);
  auto B_dyn_ad = plant_ad.MakeActuationMatrix();
  AutoDiffVecXd Bu =
      B_dyn_ad * plant_ad.get_actuation_input_port().Eval(context_ad);
  drake::multibody::MultibodyForces<AutoDiffXd> f_app(plant_ad);
  plant_ad.CalcForceElementsContribution(context_ad, &f_app);
  AutoDiffVecXd generalized_forces_ad;
  plant_ad.CalcGeneralizedForces(context_ad, f_app, &generalized_forces_ad);
  MatrixX<AutoDiffXd> M(n_v, n_v);
  plant_ad.CalcMassMatrix(context_ad, &M);
  AutoDiffVecXd vdot_no_contact =
      M.ldlt().solve(Bu + generalized_forces_ad - C);
  VectorXd d_vv = ExtractValue(vdot_no_contact);
  MatrixXd AB_v = ExtractGradient(vdot_no_contact);
  MatrixXd AB_v_q = AB_v.block(0, 0, n_v, n_q);
  MatrixXd AB_v_v = AB_v.block(0, n_q, n_v, n_v);
  MatrixXd AB_v_u = AB_v.block(0, n_x, n_v, n_u);
  VectorXd x_dvv(n_q + n_v + n_u);
  x_dvv << plant.GetPositions(context), plant.GetVelocities(context), u_dyn;
  VectorXd x_dvvcomp = AB_v * x_dvv;
  VectorXd d_v = d_vv - x_dvvcomp;  // = M_inv @ k
  auto M_ldlt = ExtractValue(M).ldlt();
  MatrixXd Ep_t = MatrixXd::Zero(n_internal_contacts, n_sigma);  // = Ep.T
  for (int i = 0; i < n_internal_contacts; i++) {
    Ep_t.block(i, 2 * i, 1, 2) = MatrixXd::Ones(1, 2);
  }
  MatrixXd J_n(n_external_contacts, n_v);       // = N.T
  MatrixXd J_t(n_lambda_with_tangential, n_v);  // = D.T
  Eigen::Vector3d planar_normal(0, 0, 1);
  for (int i = 0; i < n_external_contacts; i++) {
    multibody::GeomGeomCollider collider(plant, external_contact_geoms[i]);
    auto [phi_i, J_i] =
        (num_friction_directions_per_contact[i] == 1)
            ? collider.EvalPlanar(context, planar_normal)
            : collider.EvalPolytope(context,
                                    num_friction_directions_per_contact[i]);
    J_n.row(i) = J_i.row(0);
    J_t.block(starting_index_per_contact_in_lambda_t_vector[i], 0,
              2 * num_friction_directions_per_contact[i], n_v) =
        J_i.block(1, 0, 2 * num_friction_directions_per_contact[i], n_v);
  }
  MatrixXd Jp_t(n_sigma, n_v);  // = Dp.T
  for (int i = 0; i < n_internal_contacts; i++) {
    multibody::GeomGeomCollider collider(plant, internal_contact_geoms[i]);
    // Can collide the two node spheres together and use their normal contact
    // direction as the plasticity force direction.  The '2' is the minimum
    // number of friction directions to call EvalPolytope, but we throw out the
    // tangential directions and just consider the normal direction.
    auto [phi_i, J_i] = collider.EvalPolytope(context, 2);
    Jp_t.row(2 * i) = J_i.row(0);
    Jp_t.row(2 * i + 1) = -J_i.row(0);
  }
  MatrixXd MinvJp_t_T = M_ldlt.solve(Jp_t.transpose());  // = M_inv @ Dp
  ///////////////////////////////////////////////////////////////////////////

  /// Build them.
  /// NOTE: using complementarity variable ordering lambda_int = [slack; sigma]
  /// to match the code's S&T implementation lambda_ext = [slack; normal;
  /// tangential].
  // D
  MatrixXd D_sig = MatrixXd::Zero(n_x, n_lambda_internal);
  D_sig.block(0, n_internal_contacts, n_q, n_sigma) =
      dt * dt * qdotNv * MinvJp_t_T;
  D_sig.block(n_q, n_internal_contacts, n_v, n_sigma) = dt * MinvJp_t_T;
  // E
  MatrixXd E_sig = MatrixXd::Zero(n_lambda_internal, n_x);
  E_sig.block(n_internal_contacts, 0, n_sigma, n_q) = dt * Jp_t * AB_v_q;
  E_sig.block(n_internal_contacts, n_q, n_sigma, n_v) =
      Jp_t + dt * Jp_t * AB_v_v;
  // H
  MatrixXd H_sig = MatrixXd::Zero(n_lambda_internal, n_u);
  H_sig.block(n_internal_contacts, 0, n_sigma, n_u) = dt * Jp_t * AB_v_u;
  // F
  MatrixXd F_sig = MatrixXd::Zero(n_lambda_internal, n_lambda_internal);
  F_sig.block(0, n_internal_contacts, n_internal_contacts, n_sigma) = -Ep_t;
  F_sig.block(n_internal_contacts, 0, n_sigma, n_internal_contacts) =
      Ep_t.transpose();
  F_sig.block(n_internal_contacts, n_internal_contacts, n_sigma, n_sigma) =
      dt * Jp_t * MinvJp_t_T;
  // There are coupling terms whose structure depends on the external contact
  // model.
  MatrixXd F_sig_bl = MatrixXd::Zero(n_lambda_internal, n_lambda);
  MatrixXd F_sig_ur = MatrixXd::Zero(n_lambda, n_lambda_internal);
  if (contact_model == ContactModel::kStewartAndTrinkle) {
    // Compute a few more quantities used only by the Stewart and Trinkle
    // external contact model.
    MatrixXd MinvJ_n_T = M_ldlt.solve(J_n.transpose());  // = M_inv @ N
    MatrixXd MinvJ_t_T = M_ldlt.solve(J_t.transpose());  // = M_inv @ D

    // Now build the coupled portions of the F matrix.
    F_sig_bl.block(n_internal_contacts, n_external_contacts,
                   n_internal_contacts, n_external_contacts) =
        dt * Jp_t * MinvJ_n_T;
    F_sig_bl.block(n_internal_contacts, 2 * n_external_contacts,
                   n_internal_contacts, n_lambda_with_tangential) =
        dt * Jp_t * MinvJ_t_T;
    F_sig_ur.block(n_external_contacts, n_internal_contacts,
                   n_external_contacts, n_internal_contacts) =
        dt * dt * J_n * MinvJp_t_T;
    F_sig_ur.block(2 * n_external_contacts, n_internal_contacts,
                   n_lambda_with_tangential, n_internal_contacts) =
        dt * J_t * MinvJp_t_T;
  } else if (contact_model == ContactModel::kAnitescu) {
    // Compute a few more quantities used only by the Anitescu external contact
    // model.
    MatrixXd E_t =
        MatrixXd::Zero(n_external_contacts, n_lambda_with_tangential);
    for (int i = 0; i < n_external_contacts; i++) {
      E_t.block(i, starting_index_per_contact_in_lambda_t_vector[i], 1,
                2 * num_friction_directions_per_contact[i]) =
          MatrixXd::Ones(1, 2 * num_friction_directions_per_contact[i]);
    }
    VectorXd mu_vec = Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(
        mu.data(), mu.size());
    VectorXd anitescu_mu_vec = VectorXd::Zero(n_lambda);
    for (int i = 0; i < mu_vec.rows(); i++) {
      anitescu_mu_vec
          .segment(starting_index_per_contact_in_lambda_t_vector[i],
                   2 * num_friction_directions_per_contact[i])
          .setConstant(mu[i]);
    }
    MatrixXd anitescu_mu_matrix = anitescu_mu_vec.asDiagonal();
    MatrixXd J_c = E_t.transpose() * J_n + anitescu_mu_matrix * J_t;
    MatrixXd MinvJ_c_T = M_ldlt.solve(J_c.transpose());

    // Now build the coupled portions of the F matrix.
    F_sig_bl.block(n_internal_contacts, 0, n_sigma, n_lambda) =
        dt * Jp_t * MinvJ_c_T;
    F_sig_ur.block(0, n_internal_contacts, n_lambda, n_sigma) =
        dt * J_c * MinvJp_t_T;
  }
  // c
  VectorXd c_sig = VectorXd::Zero(n_lambda_internal);
  c_sig.segment(0, n_internal_contacts) = yield_forces;
  c_sig.segment(n_internal_contacts, n_sigma) = dt * Jp_t * d_v;

  /// Piece together the matrices from original and plasticity blocks.
  // A, B, and d are unchanged when adding internal forces.
  MatrixXd A = lcs_without_plasticity.A_[0];
  MatrixXd B = lcs_without_plasticity.B_[0];
  VectorXd d = lcs_without_plasticity.d_[0];
  // D
  MatrixXd D = MatrixXd::Zero(n_x, n_lambda + n_lambda_internal);
  D.block(0, 0, n_x, n_lambda) = lcs_without_plasticity.D_[0];
  D.block(0, n_lambda, n_x, n_lambda_internal) = D_sig;
  // E
  MatrixXd E = MatrixXd::Zero(n_lambda + n_lambda_internal, n_x);
  E.block(0, 0, n_lambda, n_x) = lcs_without_plasticity.E_[0];
  E.block(n_lambda, 0, n_lambda_internal, n_x) = E_sig;
  // F
  MatrixXd F = MatrixXd::Zero(n_lambda + n_lambda_internal,
                              n_lambda + n_lambda_internal);
  F.block(0, 0, n_lambda, n_lambda) = lcs_without_plasticity.F_[0];
  F.block(n_lambda, n_lambda, n_lambda_internal, n_lambda_internal) = F_sig;
  F.block(0, n_lambda, n_lambda, n_lambda_internal) = F_sig_ur;
  F.block(n_lambda, 0, n_lambda_internal, n_lambda) = F_sig_bl;
  // H
  MatrixXd H = MatrixXd::Zero(n_lambda + n_lambda_internal, n_u);
  H.block(0, 0, n_lambda, n_u) = lcs_without_plasticity.H_[0];
  H.block(n_lambda, 0, n_lambda_internal, n_u) = H_sig;
  // c
  VectorXd c = VectorXd::Zero(n_lambda + n_lambda_internal);
  c.segment(0, n_lambda) = lcs_without_plasticity.c_[0];
  c.segment(n_lambda, n_lambda_internal) = c_sig;
  // NOTE: (@bibit) As far as I can tell, these are only defined for Anitescu
  // contact model, but they are not used anywhere.
  // W_x, W_l, W_u, w grow in size, but the additional elements are zero.
  // W_x
  MatrixXd W_x = MatrixXd::Zero(n_lambda + n_lambda_internal, n_x);
  W_x.block(0, 0, n_lambda, n_x) = lcs_without_plasticity.W_x_;
  // W_l
  MatrixXd W_l = MatrixXd::Zero(n_lambda + n_lambda_internal,
                                n_lambda + n_lambda_internal);
  W_l.block(0, 0, n_lambda, n_lambda) = lcs_without_plasticity.W_l_;
  // W_u
  MatrixXd W_u = MatrixXd::Zero(n_lambda + n_lambda_internal, n_u);
  W_u.block(0, 0, n_lambda, n_u) = lcs_without_plasticity.W_u_;
  // w
  VectorXd w = VectorXd::Zero(n_lambda + n_lambda_internal);
  w.segment(0, n_lambda) = lcs_without_plasticity.w_;

  LCS system(A, B, D, d, E, F, H, c, N, dt);
  system.SetTangentGapLinearization(W_x, W_l, W_u, w);
  return system;
}

}  // namespace solvers
}  // namespace dairlib
