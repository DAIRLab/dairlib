#include "examples/goldilocks_models/reduced_order_models.h"

#include <algorithm>

#include "multibody/multibody_utils.h"

using drake::MatrixX;
using drake::VectorX;
using drake::multibody::BodyIndex;
using drake::multibody::Frame;
using drake::multibody::JacobianWrtVariable;
using drake::multibody::ModelInstanceIndex;
using drake::multibody::MultibodyPlant;
using drake::systems::Context;
using drake::trajectories::PiecewisePolynomial;
using Eigen::Matrix3Xd;
using Eigen::Vector3d;
using std::cout;
using std::endl;
using std::map;
using std::multiset;
using std::pair;
using std::set;
using std::shared_ptr;
using std::string;
using std::to_string;
using std::vector;

namespace dairlib {
namespace goldilocks_models {

// Testing flag
bool use_pelvis = false;
// torso for five-link robot, and pelvis for Cassie
std::string pelvis_body_name = "pelvis";  // "torso";

using multibody::isQuaternion;
using multibody::JwrtqdotToJwrtv;
using multibody::WToQuatDotMap;

MonomialFeatures::MonomialFeatures() : MonomialFeatures(0, 0, {}) {}

MonomialFeatures::MonomialFeatures(int n_order, int n_q, vector<int> skip_inds,
                                   const std::string& name)
    : n_q_(n_q), name_(name), n_order_(n_order), skip_inds_(skip_inds) {
  for (const auto& idx : skip_inds) {
    DRAKE_DEMAND(idx >= 0);
    DRAKE_DEMAND(idx < n_q);
  }

  // Construct active indices (complement of skip_inds)
  std::vector<int> active_inds;
  for (int i = 0; i < n_q; i++) {
    if (std::find(skip_inds.begin(), skip_inds.end(), i) == skip_inds.end()) {
      active_inds.push_back(i);
    }
  }

  // Construct features_
  set<multiset<int>> previous_subfeatures = {};
  for (int order = 0; order <= n_order; order++) {
    previous_subfeatures =
        ConstructSubfeaturesWithOneMoreOrder(active_inds, previous_subfeatures);
    features_.insert(previous_subfeatures.begin(), previous_subfeatures.end());
  }

  // Construct first time partial derivatives for each term
  int feature_idx = 0;
  for (const auto& feat : features_) {
    for (auto i : active_inds) {
      multiset<int> monomial_copy = feat;
      // Count how many i in the term
      int count = monomial_copy.count(i);
      if (count != 0) {
        // erase the element i from the term
        auto itr = monomial_copy.find(i);
        if (itr != monomial_copy.end()) {
          monomial_copy.erase(itr);
        }
        // Add the resulting term into first_ord_partial_diff_
        first_ord_partial_diff_[{feature_idx, {i}}] = {count, monomial_copy};
      }
    }
    feature_idx++;
  }

  // Construct second time partial derivatives for each term
  for (const auto& ele : first_ord_partial_diff_) {
    for (auto i : active_inds) {
      pair<int, multiset<int>> key_copy = ele.first;
      pair<int, multiset<int>> term_copy = ele.second;
      auto monomial_copy = term_copy.second;
      // Count how many i in the term
      int count = monomial_copy.count(i);
      if (count != 0) {
        // erase the element i from the term
        auto itr = monomial_copy.find(i);
        if (itr != monomial_copy.end()) {
          monomial_copy.erase(itr);
        }
        // Update the "wrt" index set and the coefficient
        key_copy.second.insert(i);
        auto new_coeff = term_copy.first * count;
        // Insert the resulting term into first_ord_partial_diff_ if it doesn't
        // exist. Otherwise, add the new coefficient to the existing coefficient
        auto it = second_ord_partial_diff_.find(key_copy);
        if (it == second_ord_partial_diff_.end()) {
          second_ord_partial_diff_[key_copy] = {new_coeff, monomial_copy};
        } else {
          it->second.first += new_coeff;
        }
      }
    }
  }

  // Sanity Check
  for (const auto& ele : first_ord_partial_diff_) {
    DRAKE_DEMAND(ele.first.second.size() == 1);
  }
  for (const auto& ele : second_ord_partial_diff_) {
    DRAKE_DEMAND(ele.first.second.size() == 2);
  }
};

void MonomialFeatures::PrintInfo() const {
  if (!name_.empty()) cout << name_ << " ";
  cout << "uses monominal features with\n";
  cout << "  n_order = " << n_order_ << endl;
  cout << "  n_q = " << n_q_ << endl;
  cout << "  skip_inds = {";
  for (const auto& idx : skip_inds_) {
    cout << idx << ", ";
  }
  cout << "}\n";
}

set<multiset<int>> MonomialFeatures::ConstructSubfeaturesWithOneMoreOrder(
    const vector<int>& active_inds,
    const set<multiset<int>>& terms_of_same_order) {
  set<multiset<int>> ret;
  if (terms_of_same_order.empty()) {
    // if terms_of_same_order is empty, then add {}, i.e. zero order term, to
    // the set
    ret.insert(multiset<int>());
  } else {
    for (const auto& term : terms_of_same_order) {
      for (auto i : active_inds) {
        multiset<int> new_term = term;
        new_term.insert(i);
        ret.insert(new_term);
      }
    }
  }
  return ret;
}

void MonomialFeatures::PrintMultiset(const multiset<int>& set) {
  bool past_first_element = false;
  cout << "(";
  for (const auto& ele : set) {
    if (!past_first_element) {
      past_first_element = true;
    } else {
      cout << ", ";
    }
    cout << ele;
  }
  cout << ")";
}
void MonomialFeatures::PrintSymbolicFeatures() const {
  cout << "Features = \n";
  cout << "  row index : symbolic term\n";
  int row_idx = 0;
  for (const auto& feat_i : features_) {
    cout << "  " << row_idx << ": ";
    PrintMultiset(feat_i);
    cout << "\n";
    row_idx++;
  }
}
void MonomialFeatures::PrintSymbolicPartialDerivatives(int order) const {
  DRAKE_DEMAND((order == 1) || (order == 2));
  (order == 1) ? cout << "First" : cout << "Second";
  cout << " order partial derivatives = \n";
  cout << "  Key ==> Term\n";
  const auto& map =
      (order == 1) ? first_ord_partial_diff_ : second_ord_partial_diff_;
  for (const auto& ele : map) {
    cout << "  " << ele.first.first << ", ";
    PrintMultiset(ele.first.second);
    cout << " ==> " << ele.second.first << ", ";
    PrintMultiset(ele.second.second);
    cout << endl;
  }
}

VectorX<double> MonomialFeatures::Eval(const VectorX<double>& q) const {
  DRAKE_DEMAND(q.size() == n_q_);

  VectorX<double> ret(features_.size());
  int idx = 0;
  for (const auto& term : features_) {
    double value = 1;
    for (const auto& ele : term) {
      value *= q(ele);
    }
    ret(idx) = value;
    idx++;
  }
  return ret;
}
VectorX<double> MonomialFeatures::EvalJV(const VectorX<double>& q,
                                         const VectorX<double>& qdot) const {
  return EvalFeatureTimeDerivatives(q, qdot, first_ord_partial_diff_);
}
VectorX<double> MonomialFeatures::EvalJdotV(const VectorX<double>& q,
                                            const VectorX<double>& qdot) const {
  return EvalFeatureTimeDerivatives(q, qdot, second_ord_partial_diff_);
}

VectorX<double> MonomialFeatures::EvalFeatureTimeDerivatives(
    const VectorX<double>& q, const VectorX<double>& qdot,
    const map<pair<int, multiset<int>>, pair<int, multiset<int>>>&
        partial_diff_map) const {
  DRAKE_DEMAND(q.size() == n_q_);
  DRAKE_DEMAND(qdot.size() == n_q_);

  VectorX<double> ret = VectorX<double>::Zero(features_.size());
  for (const auto& ele : partial_diff_map) {
    double value = ele.second.first;
    for (const auto& q_idx : ele.second.second) {
      value *= q(q_idx);
    }
    for (const auto& qdot_idx : ele.first.second) {
      value *= qdot(qdot_idx);
    }

    ret(ele.first.first) += value;
  }
  return ret;
}

MatrixX<double> MonomialFeatures::EvalJwrtqdot(const VectorX<double>& q) const {
  DRAKE_DEMAND(q.size() == n_q_);

  MatrixX<double> ret = MatrixX<double>::Zero(features_.size(), n_q_);
  for (const auto& ele : first_ord_partial_diff_) {
    const auto& key = ele.first;
    const auto& term = ele.second;

    double value = term.first;
    for (const auto& q_idx : term.second) {
      value *= q(q_idx);
    }

    ret(key.first, *key.second.begin()) = value;
  }

  return ret;
}

/// Constructors of ReducedOrderModel
ReducedOrderModel::ReducedOrderModel(int n_y, int n_tau,
                                     const MatrixX<double>& B, int n_feature_y,
                                     int n_feature_yddot,
                                     const MonomialFeatures& mapping_basis,
                                     const MonomialFeatures& dynamic_basis,
                                     const std::set<int>& invariant_elements,
                                     const std::string& name)
    : name_(name),
      n_y_(n_y),
      n_yddot_(n_y),
      n_tau_(n_tau),
      B_(B),
      n_feature_y_(n_feature_y),
      n_feature_yddot_(n_feature_yddot),
      mapping_basis_(mapping_basis),
      dynamic_basis_(dynamic_basis),
      theta_y_(VectorX<double>::Zero((n_y - invariant_elements.size()) *
                                     n_feature_y)),
      theta_yddot_(VectorX<double>::Zero(n_y * n_feature_yddot)),
      invariant_elements_(invariant_elements) {
  for (const auto& element : invariant_elements) {
    DRAKE_DEMAND(0 <= element);
    DRAKE_DEMAND(element < n_y);
  }
  for (int i = 0; i < n_y; i++) {
    if (std::find(invariant_elements.begin(), invariant_elements.end(), i) ==
        invariant_elements.end()) {
      varying_elements_.insert(i);
    }
  }
};

/// Methods of ReducedOrderModel
void ReducedOrderModel::CheckModelConsistency() const {
  DRAKE_DEMAND(B_.rows() == n_yddot_);
  DRAKE_DEMAND(B_.cols() == n_tau_);
  DRAKE_DEMAND(theta_y_.size() == varying_elements_.size() * n_feature_y_);
  DRAKE_DEMAND(theta_yddot_.size() == n_yddot_ * n_feature_yddot_);
};
void ReducedOrderModel::PrintInfo() const {
  cout << "Reduced-order model (" << name() << ") with parameters\n";
  cout << "n_y = " << n_y_ << ", n_tau = " << n_tau_ << endl;
  if (n_tau_ != 0) {
    cout << "B = \n" << B_ << "\n";
  }
  cout << "n_feature_y = " << n_feature_y_ << endl;
  cout << "n_feature_yddot = " << n_feature_yddot_ << endl;
  cout << "varying elements of the model = ";
  for (const auto& element : varying_elements_) {
    cout << element << ", ";
  }
  cout << endl;
}

VectorX<double> ReducedOrderModel::theta() const {
  VectorX<double> ret(theta_y_.size() + theta_yddot_.size());
  ret << theta_y_, theta_yddot_;
  return ret;
};
void ReducedOrderModel::SetThetaY(const VectorX<double>& theta_y) {
  DRAKE_DEMAND(theta_y_.size() == theta_y.size());
  theta_y_ = theta_y;
};
void ReducedOrderModel::SetThetaYddot(const VectorX<double>& theta_yddot) {
  DRAKE_DEMAND(theta_yddot_.size() == theta_yddot.size());
  theta_yddot_ = theta_yddot;
};
void ReducedOrderModel::SetTheta(const VectorX<double>& theta) {
  DRAKE_DEMAND(theta.size() == theta_y_.size() + theta_yddot_.size());
  theta_y_ = theta.head(theta_y_.size());
  theta_yddot_ = theta.tail(theta_yddot_.size());
};

VectorX<double> ReducedOrderModel::EvalMappingFunc(
    const VectorX<double>& q, const Context<double>& context) const {
  VectorX<double> phi = EvalMappingFeat(q, context);

  VectorX<double> expression = VectorX<double>::Zero(n_y_);
  // Varying parts
  int row_idx = 0;
  for (const auto& element : varying_elements_) {
    expression(element) =
        theta_y_.segment(row_idx * n_feature_y_, n_feature_y_).dot(phi);
    row_idx++;
  }
  // Invariant parts
  for (const auto& element : invariant_elements_) {
    expression(element) = phi(element);
  }
  return expression;
}
VectorX<double> ReducedOrderModel::EvalDynamicFunc(
    const VectorX<double>& y, const VectorX<double>& ydot,
    const VectorX<double>& tau) const {
  VectorX<double> phi = EvalDynamicFeat(y, ydot, tau);

  VectorX<double> expression(n_yddot_);
  for (int i = 0; i < n_yddot_; i++) {
    expression(i) =
        theta_yddot_.segment(i * n_feature_yddot_, n_feature_yddot_).dot(phi);
  }
  expression += B_ * tau;
  return expression;
}
VectorX<double> ReducedOrderModel::EvalMappingFuncJV(
    const VectorX<double>& q, const VectorX<double>& v,
    const Context<double>& context) const {
  VectorX<double> JV_feat = EvalMappingFeatJV(q, v, context);

  VectorX<double> JV = VectorX<double>::Zero(n_y_);
  // Varying parts
  int row_idx = 0;
  for (const auto& element : varying_elements_) {
    JV(element) =
        theta_y_.segment(row_idx * n_feature_y_, n_feature_y_).dot(JV_feat);
    row_idx++;
  }
  // Invariant parts
  for (const auto& element : invariant_elements_) {
    JV(element) = JV_feat(element);
  }
  return JV;
}
VectorX<double> ReducedOrderModel::EvalMappingFuncJdotV(
    const VectorX<double>& q, const VectorX<double>& v,
    const Context<double>& context) const {
  VectorX<double> JdotV_feat = EvalMappingFeatJdotV(q, v, context);

  VectorX<double> JdotV = VectorX<double>::Zero(n_y_);
  // Varying parts
  int row_idx = 0;
  for (const auto& element : varying_elements_) {
    JdotV(element) =
        theta_y_.segment(row_idx * n_feature_y_, n_feature_y_).dot(JdotV_feat);
    row_idx++;
  }
  // Invariant parts
  for (const auto& element : invariant_elements_) {
    JdotV(element) = JdotV_feat(element);
  }
  return JdotV;
}
MatrixX<double> ReducedOrderModel::EvalMappingFuncJ(
    const VectorX<double>& q, const Context<double>& context) const {
  MatrixX<double> J_feat = EvalMappingFeatJ(q, context);

  MatrixX<double> J = MatrixX<double>::Zero(n_y_, J_feat.cols());
  // Varying parts
  int row_idx = 0;
  for (const auto& element : varying_elements_) {
    J.row(element) =
        theta_y_.segment(row_idx * n_feature_y_, n_feature_y_).transpose() *
        J_feat;
    row_idx++;
  }
  // Invariant parts
  for (const auto& element : invariant_elements_) {
    J.row(element) = J_feat.row(element);
  }
  return J;
}

/// LIPM
Lipm::Lipm(const MultibodyPlant<double>& plant,
           const BodyPoint& stance_contact_point,
           const MonomialFeatures& mapping_basis,
           const MonomialFeatures& dynamic_basis, int world_dim,
           const std::set<int>& invariant_elements,
           bool use_pelvis)
    : ReducedOrderModel(world_dim, 0, MatrixX<double>::Zero(world_dim, 0),
                        world_dim + mapping_basis.length(),
                        (world_dim - 1) + dynamic_basis.length(), mapping_basis,
                        dynamic_basis, invariant_elements,
                        to_string(world_dim) + "D lipm"),
      plant_(plant),
      world_(plant_.world_frame()),
      stance_contact_point_(stance_contact_point),
      world_dim_(world_dim),
      pelvis_(std::pair<const Vector3d, const Frame<double>&>(
          Vector3d::Zero(), plant_.GetFrameByName(pelvis_body_name))),
      use_pelvis_(use_pelvis){
  DRAKE_DEMAND((world_dim == 2) || (world_dim == 3));

  // Initialize model parameters (dependant on the feature vectors)
  VectorX<double> theta_y =
      VectorX<double>::Zero(varying_elements().size() * n_feature_y());
  int row_index = 0;
  for (auto element : varying_elements()) {
    theta_y(element + row_index * n_feature_y()) = 1;
    row_index++;
  }
  SetThetaY(theta_y);

  VectorX<double> theta_yddot =
      VectorX<double>::Zero(n_yddot() * n_feature_yddot());
  theta_yddot(0) = 1;
  if (world_dim == 3) {
    theta_yddot(1 + n_feature_yddot()) = 1;
  }
  SetThetaYddot(theta_yddot);

  is_quaternion_ = isQuaternion(plant);

  // Always check dimension after model construction
  CheckModelConsistency();
};
// Copy constructor
Lipm::Lipm(const Lipm& old_obj)
    : ReducedOrderModel(old_obj),
      plant_(old_obj.plant()),
      world_(old_obj.world()),
      stance_contact_point_(old_obj.stance_foot()),
      is_quaternion_(isQuaternion(old_obj.plant())),
      world_dim_(old_obj.world_dim()),
      pelvis_(std::pair<const Vector3d, const Frame<double>&>(
          Vector3d::Zero(), plant_.GetFrameByName(pelvis_body_name))) {}

VectorX<double> Lipm::EvalMappingFeat(const VectorX<double>& q,
                                      const Context<double>& context) const {
  // Get CoM position
  VectorX<double> CoM(3);
  if (use_pelvis_) {
    // testing using pelvis
    plant_.CalcPointsPositions(context, pelvis_.second, pelvis_.first,
                               plant_.world_frame(), &CoM);
  } else {
    CoM = plant_.CalcCenterOfMassPositionInWorld(context);
  }

  // Stance foot position
  VectorX<double> stance_foot_pos(3);
  plant_.CalcPointsPositions(context, stance_contact_point_.second,
                             stance_contact_point_.first, plant_.world_frame(),
                             &stance_foot_pos);
  VectorX<double> st_to_CoM = CoM - stance_foot_pos;

  VectorX<double> feature(n_feature_y());
  if (world_dim_ == 2) {
    feature << st_to_CoM(0), st_to_CoM(2), mapping_basis().Eval(q);
  } else {
    feature << st_to_CoM, mapping_basis().Eval(q);
  }
  return feature;
}
drake::VectorX<double> Lipm::EvalDynamicFeat(
    const drake::VectorX<double>& y, const drake::VectorX<double>& ydot,
    const drake::VectorX<double>& tau) const {
  VectorX<double> feature_extension = y.head(world_dim_ - 1);
  double z = y(world_dim_ - 1);
  if (z == 0) {
    cout << "avoid singularity in dynamics_expression\n";
    feature_extension *= 9.80665 / (1e-8);
  } else {
    feature_extension *= 9.80665 / z;
  }

  VectorX<double> y_and_ydot(2 * n_y());
  y_and_ydot << y, ydot;

  VectorX<double> feature(n_feature_yddot());
  feature << feature_extension, dynamic_basis().Eval(y_and_ydot);
  return feature;
}
VectorX<double> Lipm::EvalMappingFeatJV(const VectorX<double>& q,
                                        const VectorX<double>& v,
                                        const Context<double>& context) const {
  // Get CoM velocity
  VectorX<double> JV_CoM(3);
  if (use_pelvis_) {
    // testing using pelvis
    MatrixX<double> J_com(3, plant_.num_velocities());
    plant_.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable::kV,
                                             pelvis_.second, pelvis_.first,
                                             world_, world_, &J_com);
    JV_CoM = J_com * v;
  } else {
    JV_CoM = plant_.CalcCenterOfMassTranslationalVelocityInWorld(context);
  }
  // Stance foot velocity
  MatrixX<double> J_sf(3, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV, stance_contact_point_.second,
      stance_contact_point_.first, world_, world_, &J_sf);
  VectorX<double> JV_st_to_CoM = JV_CoM - J_sf * v;

  // Convert v to qdot
  VectorX<double> qdot(plant_.num_positions());
  plant_.MapVelocityToQDot(context, v, &qdot);

  VectorX<double> ret(n_feature_y());
  if (world_dim_ == 2) {
    ret << JV_st_to_CoM(0), JV_st_to_CoM(2), mapping_basis().EvalJV(q, qdot);
  } else {
    ret << JV_st_to_CoM, mapping_basis().EvalJV(q, qdot);
  }
  return ret;
}
MatrixX<double> Lipm::EvalMappingFeatJ(const VectorX<double>& q,
                                       const Context<double>& context) const {
  // Get CoM velocity
  MatrixX<double> J_com(3, plant_.num_velocities());
  if (use_pelvis_) {
    // testing using pelvis
    plant_.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable::kV,
                                             pelvis_.second, pelvis_.first,
                                             world_, world_, &J_com);
  } else {
    plant_.CalcJacobianCenterOfMassTranslationalVelocity(
        context, JacobianWrtVariable::kV, world_, world_, &J_com);
  }
  // Stance foot velocity
  MatrixX<double> J_sf(3, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV, stance_contact_point_.second,
      stance_contact_point_.first, world_, world_, &J_sf);
  MatrixX<double> J_st_to_CoM = J_com - J_sf;

  MatrixX<double> ret(n_feature_y(), plant_.num_velocities());
  if (world_dim_ == 2) {
    ret << J_st_to_CoM.row(0), J_st_to_CoM.row(2),
        is_quaternion_ ? JwrtqdotToJwrtv(q, mapping_basis().EvalJwrtqdot(q))
                       : mapping_basis().EvalJwrtqdot(q);
  } else {
    ret << J_st_to_CoM,
        is_quaternion_ ? JwrtqdotToJwrtv(q, mapping_basis().EvalJwrtqdot(q))
                       : mapping_basis().EvalJwrtqdot(q);
  }
  return ret;
}
VectorX<double> Lipm::EvalMappingFeatJdotV(
    const VectorX<double>& q, const VectorX<double>& v,
    const Context<double>& context) const {
  // Get CoM JdotV
  VectorX<double> JdotV_com(3);
  if (use_pelvis_) {
    // Testing: use pelvis origin
    JdotV_com = plant_.CalcBiasTranslationalAcceleration(
        context, JacobianWrtVariable::kV, pelvis_.second, pelvis_.first, world_,
        world_);
  } else {
    JdotV_com = plant_.CalcBiasCenterOfMassTranslationalAcceleration(
        context, JacobianWrtVariable::kV, world_, world_);
  }
  // Stance foot JdotV
  VectorX<double> JdotV_st = plant_.CalcBiasTranslationalAcceleration(
      context, JacobianWrtVariable::kV, stance_contact_point_.second,
      stance_contact_point_.first, world_, world_);
  VectorX<double> JdotV_st_to_com = JdotV_com - JdotV_st;

  // Convert v to qdot
  VectorX<double> qdot(plant_.num_positions());
  plant_.MapVelocityToQDot(context, v, &qdot);

  VectorX<double> ret(n_feature_y());
  if (world_dim_ == 2) {
    ret << JdotV_st_to_com(0), JdotV_st_to_com(2),
        mapping_basis().EvalJdotV(q, qdot);
  } else {
    ret << JdotV_st_to_com, mapping_basis().EvalJdotV(q, qdot);
  }
  return ret;
}

/// LIPM with a point-mass swing foot
LipmWithSwingFoot::LipmWithSwingFoot(const MultibodyPlant<double>& plant,
                                     const BodyPoint& stance_contact_point,
                                     const BodyPoint& swing_contact_point,
                                     const MonomialFeatures& mapping_basis,
                                     const MonomialFeatures& dynamic_basis,
                                     int world_dim,
                                     const std::set<int>& invariant_elements)
    : ReducedOrderModel(2 * world_dim, world_dim,
                        (MatrixX<double>(2 * world_dim, world_dim)
                             << MatrixX<double>::Zero(world_dim, world_dim),
                         MatrixX<double>::Identity(world_dim, world_dim))
                            .finished(),
                        2 * world_dim + mapping_basis.length(),
                        (world_dim - 1) + dynamic_basis.length(), mapping_basis,
                        dynamic_basis, invariant_elements,
                        to_string(world_dim) + "D lipm with swing foot"),
      plant_(plant),
      world_(plant_.world_frame()),
      stance_contact_point_(stance_contact_point),
      swing_contact_point_(swing_contact_point),
      world_dim_(world_dim) {
  DRAKE_DEMAND((world_dim == 2) || (world_dim == 3));

  // Initialize model parameters (dependant on the feature vectors)
  VectorX<double> theta_y =
      VectorX<double>::Zero(varying_elements().size() * n_feature_y());
  VectorX<double> theta_yddot =
      VectorX<double>::Zero(n_yddot() * n_feature_yddot());

  int row_index = 0;
  for (auto element : varying_elements()) {
    theta_y(element + row_index * n_feature_y()) = 1;
    row_index++;
  }
  SetThetaY(theta_y);

  if (world_dim == 2) {
    theta_yddot(0) = 1;
  } else if (world_dim == 3) {
    theta_yddot(0) = 1;
    theta_yddot(1 + n_feature_yddot()) = 1;
  }
  SetThetaYddot(theta_yddot);

  is_quaternion_ = isQuaternion(plant);

  // Always check dimension after model construction
  CheckModelConsistency();
};
// Copy constructor
LipmWithSwingFoot::LipmWithSwingFoot(const LipmWithSwingFoot& old_obj)
    : ReducedOrderModel(old_obj),
      plant_(old_obj.plant()),
      world_(old_obj.world()),
      stance_contact_point_(old_obj.stance_foot()),
      swing_contact_point_(old_obj.swing_foot()),
      is_quaternion_(isQuaternion(old_obj.plant())),
      world_dim_(old_obj.world_dim()) {}

VectorX<double> LipmWithSwingFoot::EvalMappingFeat(
    const VectorX<double>& q, const Context<double>& context) const {
  // Get CoM position
  VectorX<double> CoM = plant_.CalcCenterOfMassPositionInWorld(context);
  // Stance foot position
  VectorX<double> left_foot_pos(3);
  plant_.CalcPointsPositions(context, stance_contact_point_.second,
                             stance_contact_point_.first, plant_.world_frame(),
                             &left_foot_pos);
  VectorX<double> st_to_CoM = CoM - left_foot_pos;
  // Swing foot position
  VectorX<double> right_foot_pos(3);
  plant_.CalcPointsPositions(context, swing_contact_point_.second,
                             swing_contact_point_.first, plant_.world_frame(),
                             &right_foot_pos);
  VectorX<double> CoM_to_sw = right_foot_pos - CoM;

  VectorX<double> feature(n_feature_y());
  if (world_dim_ == 2) {
    feature << st_to_CoM(0), st_to_CoM(2), CoM_to_sw(0), CoM_to_sw(2),
        mapping_basis().Eval(q);
  } else {
    feature << st_to_CoM, CoM_to_sw, mapping_basis().Eval(q);
  }
  return feature;
}
drake::VectorX<double> LipmWithSwingFoot::EvalDynamicFeat(
    const drake::VectorX<double>& y, const drake::VectorX<double>& ydot,
    const drake::VectorX<double>& tau) const {
  VectorX<double> feature_extension = y.head(world_dim_ - 1);
  double z = y(world_dim_ - 1);
  if (z == 0) {
    cout << "avoid singularity in dynamics_expression\n";
    feature_extension *= 9.80665 / (1e-8);
  } else {
    feature_extension *= 9.80665 / z;
  }

  VectorX<double> y_and_ydot(2 * n_y());
  y_and_ydot << y, ydot;

  VectorX<double> feature(n_feature_yddot());
  feature << feature_extension, dynamic_basis().Eval(y_and_ydot);
  return feature;
}
VectorX<double> LipmWithSwingFoot::EvalMappingFeatJV(
    const VectorX<double>& q, const VectorX<double>& v,
    const Context<double>& context) const {
  // Get CoM velocity
  VectorX<double> JV_com(3);
  JV_com = plant_.CalcCenterOfMassTranslationalVelocityInWorld(context);
  // Stance foot velocity
  MatrixX<double> J_sf(3, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV, stance_contact_point_.second,
      stance_contact_point_.first, world_, world_, &J_sf);
  VectorX<double> JV_st_to_CoM = JV_com - J_sf * v;
  // Swing foot velocity
  MatrixX<double> J_sw(3, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV, swing_contact_point_.second,
      swing_contact_point_.first, world_, world_, &J_sw);
  VectorX<double> JV_CoM_to_sw = J_sw * v - JV_com;

  // Convert v to qdot
  VectorX<double> qdot(plant_.num_positions());
  plant_.MapVelocityToQDot(context, v, &qdot);

  VectorX<double> ret(n_feature_y());
  if (world_dim_ == 2) {
    ret << JV_st_to_CoM(0), JV_st_to_CoM(2), JV_CoM_to_sw(0), JV_CoM_to_sw(2),
        mapping_basis().EvalJV(q, qdot);
  } else {
    ret << JV_st_to_CoM(0), JV_CoM_to_sw(0), mapping_basis().EvalJV(q, qdot);
  }
  return ret;
}
MatrixX<double> LipmWithSwingFoot::EvalMappingFeatJ(
    const VectorX<double>& q, const Context<double>& context) const {
  // Get CoM velocity
  MatrixX<double> J_com(3, plant_.num_velocities());
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      context, JacobianWrtVariable::kV, world_, world_, &J_com);
  // Stance foot velocity
  MatrixX<double> J_sf(3, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV, stance_contact_point_.second,
      stance_contact_point_.first, world_, world_, &J_sf);
  MatrixX<double> J_st_to_CoM = J_com - J_sf;
  // Swing foot velocity
  MatrixX<double> J_sw(3, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV, swing_contact_point_.second,
      swing_contact_point_.first, world_, world_, &J_sw);
  MatrixX<double> J_CoM_to_sw = J_sw - J_com;

  MatrixX<double> ret(n_feature_y(), plant_.num_velocities());
  if (world_dim_ == 2) {
    ret << J_st_to_CoM.row(0), J_st_to_CoM.row(2), J_CoM_to_sw.row(0),
        J_CoM_to_sw.row(2),
        is_quaternion_ ? JwrtqdotToJwrtv(q, mapping_basis().EvalJwrtqdot(q))
                       : mapping_basis().EvalJwrtqdot(q);
  } else {
    ret << J_st_to_CoM, J_CoM_to_sw,
        is_quaternion_ ? JwrtqdotToJwrtv(q, mapping_basis().EvalJwrtqdot(q))
                       : mapping_basis().EvalJwrtqdot(q);
  }
  return ret;
}
VectorX<double> LipmWithSwingFoot::EvalMappingFeatJdotV(
    const VectorX<double>& q, const VectorX<double>& v,
    const Context<double>& context) const {
  // Get CoM JdotV
  VectorX<double> JdotV_com =
      plant_.CalcBiasCenterOfMassTranslationalAcceleration(
          context, JacobianWrtVariable::kV, world_, world_);
  // Stance foot JdotV
  VectorX<double> JdotV_st = plant_.CalcBiasTranslationalAcceleration(
      context, JacobianWrtVariable::kV, stance_contact_point_.second,
      stance_contact_point_.first, world_, world_);
  VectorX<double> JdotV_st_to_com = JdotV_com - JdotV_st;
  // Swing foot JdotV
  VectorX<double> JdotV_sw = plant_.CalcBiasTranslationalAcceleration(
      context, JacobianWrtVariable::kV, swing_contact_point_.second,
      swing_contact_point_.first, world_, world_);
  VectorX<double> JdotV_com_to_sw = JdotV_sw - JdotV_com;

  // Convert v to qdot
  VectorX<double> qdot(plant_.num_positions());
  plant_.MapVelocityToQDot(context, v, &qdot);

  VectorX<double> ret(n_feature_y());
  if (world_dim_ == 2) {
    ret << JdotV_st_to_com(0), JdotV_st_to_com(2), JdotV_com_to_sw(0),
        JdotV_com_to_sw(2), mapping_basis().EvalJdotV(q, qdot);
  } else {
    ret << JdotV_st_to_com, JdotV_com_to_sw, mapping_basis().EvalJdotV(q, qdot);
  }
  return ret;
}

/// Fixed vertical COM acceleration
const int FixHeightAccel::kDimension = 1;

FixHeightAccel::FixHeightAccel(const MultibodyPlant<double>& plant,
                               const BodyPoint& stance_contact_point,
                               const MonomialFeatures& mapping_basis,
                               const MonomialFeatures& dynamic_basis,
                               const std::set<int>& invariant_elements)
    : ReducedOrderModel(kDimension, 0, MatrixX<double>::Zero(kDimension, 0),
                        1 + mapping_basis.length(), 0 + dynamic_basis.length(),
                        mapping_basis, dynamic_basis, invariant_elements,
                        "Fixed COM vertical acceleration"),
      plant_(plant),
      world_(plant_.world_frame()),
      stance_contact_point_(stance_contact_point),
      pelvis_(std::pair<const Vector3d, const Frame<double>&>(
          Vector3d::Zero(), plant_.GetFrameByName(pelvis_body_name))) {
  // Initialize model parameters (dependant on the feature vectors)
  VectorX<double> theta_y =
      VectorX<double>::Zero(varying_elements().size() * n_feature_y());
  VectorX<double> theta_yddot =
      VectorX<double>::Zero(n_yddot() * n_feature_yddot());
  int row_index = 0;
  for (auto element : varying_elements()) {
    theta_y(element + row_index * n_feature_y()) = 1;
    row_index++;
  }
  SetThetaY(theta_y);
  SetThetaYddot(theta_yddot);

  is_quaternion_ = isQuaternion(plant);

  // Always check dimension after model construction
  CheckModelConsistency();
};
// Copy constructor
FixHeightAccel::FixHeightAccel(const FixHeightAccel& old_obj)
    : ReducedOrderModel(old_obj),
      plant_(old_obj.plant()),
      world_(old_obj.world()),
      is_quaternion_(isQuaternion(old_obj.plant())),
      stance_contact_point_(old_obj.stance_foot()),
      pelvis_(std::pair<const Vector3d, const Frame<double>&>(
          Vector3d::Zero(), plant_.GetFrameByName(pelvis_body_name))) {}

VectorX<double> FixHeightAccel::EvalMappingFeat(
    const VectorX<double>& q, const Context<double>& context) const {
  // Get CoM position
  VectorX<double> CoM = plant_.CalcCenterOfMassPositionInWorld(context);
  // Testing using pelvis
  //  VectorX<double> CoM(3);
  //  plant_.CalcPointsPositions(context, pelvis_.second,
  //                             pelvis_.first, plant_.world_frame(),
  //                             &CoM);
  // Stance foot position
  VectorX<double> left_foot_pos(3);
  plant_.CalcPointsPositions(context, stance_contact_point_.second,
                             stance_contact_point_.first, plant_.world_frame(),
                             &left_foot_pos);
  VectorX<double> st_to_CoM = CoM - left_foot_pos;

  VectorX<double> feature(n_feature_y());
  feature << st_to_CoM(2), mapping_basis().Eval(q);

  return feature;
}
drake::VectorX<double> FixHeightAccel::EvalDynamicFeat(
    const drake::VectorX<double>& y, const drake::VectorX<double>& ydot,
    const drake::VectorX<double>& tau) const {
  VectorX<double> y_and_ydot(2 * kDimension);
  y_and_ydot << y, ydot;

  VectorX<double> feature(n_feature_yddot());
  feature << dynamic_basis().Eval(y_and_ydot);
  return feature;
}
VectorX<double> FixHeightAccel::EvalMappingFeatJV(
    const VectorX<double>& q, const VectorX<double>& v,
    const Context<double>& context) const {
  // Get CoM velocity
  drake::Vector3<double> JV_CoM =
      plant_.CalcCenterOfMassTranslationalVelocityInWorld(context);
  // testing using pelvis
  //  MatrixX<double> J_com(3, plant_.num_velocities());
  //  plant_.CalcJacobianTranslationalVelocity(
  //      context, JacobianWrtVariable::kV, pelvis_.second,
  //      pelvis_.first, world_, world_, &J_com);
  // Stance foot velocity
  MatrixX<double> J_sf(3, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV, stance_contact_point_.second,
      stance_contact_point_.first, world_, world_, &J_sf);
  VectorX<double> JV_st_to_CoM = JV_CoM - J_sf * v;

  // Convert v to qdot
  VectorX<double> qdot(plant_.num_positions());
  plant_.MapVelocityToQDot(context, v, &qdot);

  VectorX<double> ret(n_feature_y());
  ret << JV_st_to_CoM(2), mapping_basis().EvalJV(q, qdot);
  return ret;
}
MatrixX<double> FixHeightAccel::EvalMappingFeatJ(
    const VectorX<double>& q, const Context<double>& context) const {
  // Get CoM velocity
  MatrixX<double> J_com(3, plant_.num_velocities());
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      context, JacobianWrtVariable::kV, world_, world_, &J_com);
  // Testing using pelvis
  //  MatrixX<double> J_com(3, plant_.num_velocities());
  //  plant_.CalcJacobianTranslationalVelocity(
  //      context, JacobianWrtVariable::kV, pelvis_.second,
  //      pelvis_.first, world_, world_, &J_com);
  // Stance foot velocity
  MatrixX<double> J_sf(3, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV, stance_contact_point_.second,
      stance_contact_point_.first, world_, world_, &J_sf);
  MatrixX<double> J_st_to_CoM = J_com - J_sf;

  MatrixX<double> ret(n_feature_y(), plant_.num_velocities());
  ret << J_st_to_CoM.row(2),
      is_quaternion_ ? JwrtqdotToJwrtv(q, mapping_basis().EvalJwrtqdot(q))
                     : mapping_basis().EvalJwrtqdot(q);
  return ret;
}
VectorX<double> FixHeightAccel::EvalMappingFeatJdotV(
    const VectorX<double>& q, const VectorX<double>& v,
    const Context<double>& context) const {
  // Get CoM JdotV
  VectorX<double> JdotV_com =
      plant_.CalcBiasCenterOfMassTranslationalAcceleration(
          context, JacobianWrtVariable::kV, world_, world_);
  // Testing: use pelvis origin
  //  VectorX<double> JdotV_com = plant_.CalcBiasTranslationalAcceleration(
  //      context, JacobianWrtVariable::kV, pelvis_.second,
  //      pelvis_.first, world_, world_);
  // Stance foot JdotV
  VectorX<double> JdotV_st = plant_.CalcBiasTranslationalAcceleration(
      context, JacobianWrtVariable::kV, stance_contact_point_.second,
      stance_contact_point_.first, world_, world_);
  VectorX<double> JdotV_st_to_com = JdotV_com - JdotV_st;

  // Convert v to qdot
  VectorX<double> qdot(plant_.num_positions());
  plant_.MapVelocityToQDot(context, v, &qdot);

  VectorX<double> ret(n_feature_y());
  ret << JdotV_st_to_com(2), mapping_basis().EvalJdotV(q, qdot);
  return ret;
}

/// Fixed vertical COM acceleration + 2D swing foot
const int FixHeightAccelWithSwingFoot::kDimension = 3;

FixHeightAccelWithSwingFoot::FixHeightAccelWithSwingFoot(
    const MultibodyPlant<double>& plant, const BodyPoint& stance_contact_point,
    const BodyPoint& swing_contact_point, const MonomialFeatures& mapping_basis,
    const MonomialFeatures& dynamic_basis,
    const std::set<int>& invariant_elements)
    : ReducedOrderModel(
          kDimension, 2,
          (MatrixX<double>(kDimension, 2) << 0, 0, 1, 0, 0, 1).finished(),
          3 + mapping_basis.length(), 0 + dynamic_basis.length(), mapping_basis,
          dynamic_basis, invariant_elements,
          "Fixed COM vertical acceleration + 2D swing foot"),
      plant_(plant),
      world_(plant_.world_frame()),
      stance_contact_point_(stance_contact_point),
      swing_contact_point_(swing_contact_point) {
  // Initialize model parameters (dependant on the feature vectors)
  VectorX<double> theta_y =
      VectorX<double>::Zero(varying_elements().size() * n_feature_y());
  VectorX<double> theta_yddot =
      VectorX<double>::Zero(n_yddot() * n_feature_yddot());
  int row_index = 0;
  for (auto element : varying_elements()) {
    theta_y(element + row_index * n_feature_y()) = 1;
    row_index++;
  }
  SetThetaY(theta_y);
  SetThetaYddot(theta_yddot);

  is_quaternion_ = isQuaternion(plant);

  // Always check dimension after model construction
  CheckModelConsistency();
};
// Copy constructor
FixHeightAccelWithSwingFoot::FixHeightAccelWithSwingFoot(
    const FixHeightAccelWithSwingFoot& old_obj)
    : ReducedOrderModel(old_obj),
      plant_(old_obj.plant()),
      world_(old_obj.world()),
      stance_contact_point_(old_obj.stance_foot()),
      swing_contact_point_(old_obj.swing_foot()),
      is_quaternion_(isQuaternion(old_obj.plant())) {}

VectorX<double> FixHeightAccelWithSwingFoot::EvalMappingFeat(
    const VectorX<double>& q, const Context<double>& context) const {
  // Get CoM position
  VectorX<double> CoM = plant_.CalcCenterOfMassPositionInWorld(context);
  // Stance foot position
  VectorX<double> left_foot_pos(3);
  plant_.CalcPointsPositions(context, stance_contact_point_.second,
                             stance_contact_point_.first, plant_.world_frame(),
                             &left_foot_pos);
  VectorX<double> st_to_CoM = CoM - left_foot_pos;
  // Swing foot position
  VectorX<double> right_foot_pos(3);
  plant_.CalcPointsPositions(context, swing_contact_point_.second,
                             swing_contact_point_.first, plant_.world_frame(),
                             &right_foot_pos);
  VectorX<double> CoM_to_sw = right_foot_pos - CoM;

  VectorX<double> feature(n_feature_y());
  feature << st_to_CoM(2), CoM_to_sw(0), CoM_to_sw(2), mapping_basis().Eval(q);

  return feature;
}
drake::VectorX<double> FixHeightAccelWithSwingFoot::EvalDynamicFeat(
    const drake::VectorX<double>& y, const drake::VectorX<double>& ydot,
    const drake::VectorX<double>& tau) const {
  VectorX<double> y_and_ydot(2 * kDimension);
  y_and_ydot << y, ydot;

  VectorX<double> feature(n_feature_yddot());
  feature << dynamic_basis().Eval(y_and_ydot);
  return feature;
}
VectorX<double> FixHeightAccelWithSwingFoot::EvalMappingFeatJV(
    const VectorX<double>& q, const VectorX<double>& v,
    const Context<double>& context) const {
  // Get CoM velocity
  drake::Vector3<double> JV_CoM =
      plant_.CalcCenterOfMassTranslationalVelocityInWorld(context);
  // Stance foot velocity
  MatrixX<double> J_sf(3, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV, stance_contact_point_.second,
      stance_contact_point_.first, world_, world_, &J_sf);
  VectorX<double> JV_st_to_CoM = JV_CoM - J_sf * v;
  // Swing foot velocity
  MatrixX<double> J_sw(3, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV, swing_contact_point_.second,
      swing_contact_point_.first, world_, world_, &J_sw);
  VectorX<double> JV_CoM_to_sw = J_sw * v - JV_CoM;

  // Convert v to qdot
  VectorX<double> qdot(plant_.num_positions());
  plant_.MapVelocityToQDot(context, v, &qdot);

  VectorX<double> ret(n_feature_y());
  ret << JV_st_to_CoM(2), JV_CoM_to_sw(0), JV_CoM_to_sw(2),
      mapping_basis().EvalJV(q, qdot);
  return ret;
}
MatrixX<double> FixHeightAccelWithSwingFoot::EvalMappingFeatJ(
    const VectorX<double>& q, const Context<double>& context) const {
  // Get CoM velocity
  MatrixX<double> J_com(3, plant_.num_velocities());
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      context, JacobianWrtVariable::kV, world_, world_, &J_com);
  // Stance foot velocity
  MatrixX<double> J_sf(3, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV, stance_contact_point_.second,
      stance_contact_point_.first, world_, world_, &J_sf);
  MatrixX<double> J_st_to_CoM = J_com - J_sf;
  // Swing foot velocity
  MatrixX<double> J_sw(3, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV, swing_contact_point_.second,
      swing_contact_point_.first, world_, world_, &J_sw);
  MatrixX<double> J_CoM_to_sw = J_sw - J_com;

  MatrixX<double> ret(n_feature_y(), plant_.num_velocities());
  ret << J_st_to_CoM.row(2), J_CoM_to_sw.row(0), J_CoM_to_sw.row(2),
      is_quaternion_ ? JwrtqdotToJwrtv(q, mapping_basis().EvalJwrtqdot(q))
                     : mapping_basis().EvalJwrtqdot(q);
  return ret;
}
VectorX<double> FixHeightAccelWithSwingFoot::EvalMappingFeatJdotV(
    const VectorX<double>& q, const VectorX<double>& v,
    const Context<double>& context) const {
  // Get CoM JdotV
  VectorX<double> JdotV_com =
      plant_.CalcBiasCenterOfMassTranslationalAcceleration(
          context, JacobianWrtVariable::kV, world_, world_);
  // Stance foot JdotV
  VectorX<double> JdotV_st = plant_.CalcBiasTranslationalAcceleration(
      context, JacobianWrtVariable::kV, stance_contact_point_.second,
      stance_contact_point_.first, world_, world_);
  VectorX<double> JdotV_st_to_com = JdotV_com - JdotV_st;
  // Swing foot JdotV
  VectorX<double> JdotV_sw = plant_.CalcBiasTranslationalAcceleration(
      context, JacobianWrtVariable::kV, swing_contact_point_.second,
      swing_contact_point_.first, world_, world_);
  VectorX<double> JdotV_com_to_sw = JdotV_sw - JdotV_com;

  // Convert v to qdot
  VectorX<double> qdot(plant_.num_positions());
  plant_.MapVelocityToQDot(context, v, &qdot);

  VectorX<double> ret(n_feature_y());
  ret << JdotV_st_to_com(2), JdotV_com_to_sw(0), JdotV_com_to_sw(2),
      mapping_basis().EvalJdotV(q, qdot);
  return ret;
}

/// GIP
// TODO: Need to make the code better:
//  - implementation of B. Now that B is not always constant.
// TODO: will need to add a feature to not optimize both mapping and dynamics
// TODO: check again if the following is implmented correclty
//  B
//  n_feature_y
//  n_feature_yddot
//  only update dynamics funciton
Gip::Gip(const MultibodyPlant<double>& plant,
         const BodyPoint& stance_contact_point,
         const MonomialFeatures& mapping_basis,
         const MonomialFeatures& dynamic_basis, int world_dim,
         const std::set<int>& invariant_elements)
    : ReducedOrderModel(world_dim, 1, MatrixX<double>::Zero(world_dim, 1),
                        world_dim + mapping_basis.length(),
                        (world_dim + 1) + dynamic_basis.length(), mapping_basis,
                        dynamic_basis, invariant_elements,
                        to_string(world_dim) + "D GIP"),
      plant_(plant),
      world_(plant_.world_frame()),
      stance_contact_point_(stance_contact_point),
      world_dim_(world_dim),
      pelvis_(std::pair<const Vector3d, const Frame<double>&>(
          Vector3d::Zero(), plant_.GetFrameByName(pelvis_body_name))) {
  DRAKE_DEMAND((world_dim == 2) || (world_dim == 3));

  // Initialize model parameters (dependant on the feature vectors)
  VectorX<double> theta_y =
      VectorX<double>::Zero(varying_elements().size() * n_feature_y());
  int row_index = 0;
  for (auto element : varying_elements()) {
    theta_y(element + row_index * n_feature_y()) = 1;
    row_index++;
  }
  SetThetaY(theta_y);

  VectorX<double> theta_yddot =
      VectorX<double>::Zero(n_yddot() * n_feature_yddot());
  int i;
  for (i = 0; i < world_dim; i++) {
    theta_yddot(i + i * n_feature_yddot()) = 1;
  }
  theta_yddot(i + (i - 1) * n_feature_yddot()) = 1;

  SetThetaYddot(theta_yddot);

  is_quaternion_ = isQuaternion(plant);

  // Get total mass of the robot
  DRAKE_UNREACHABLE();  // I haven't tested CalcTotalMass(). I didn't initial the state here, becasue it's probably not necessary?
  auto context = plant.CreateDefaultContext();
  total_mass_ = plant.CalcTotalMass(*context);
  cout << "total_mass_ = " << total_mass_ << endl;
  DRAKE_DEMAND(total_mass_ > 0);

  // Always check dimension after model construction
  CheckModelConsistency();
}

// Copy constructor
Gip::Gip(const Gip& old_obj)
    : ReducedOrderModel(old_obj),
      plant_(old_obj.plant()),
      world_(old_obj.world()),
      stance_contact_point_(old_obj.stance_foot()),
      is_quaternion_(isQuaternion(old_obj.plant())),
      world_dim_(old_obj.world_dim()),
      total_mass_(old_obj.total_mass()),
      pelvis_(std::pair<const Vector3d, const Frame<double>&>(
          Vector3d::Zero(), plant_.GetFrameByName(pelvis_body_name))) {}

VectorX<double> Gip::EvalMappingFeat(const VectorX<double>& q,
                                     const Context<double>& context) const {
  // Get CoM position
  VectorX<double> CoM(3);
  if (use_pelvis) {
    // testing using pelvis
    plant_.CalcPointsPositions(context, pelvis_.second, pelvis_.first,
                               plant_.world_frame(), &CoM);
  } else {
    CoM = plant_.CalcCenterOfMassPositionInWorld(context);
  }

  // Stance foot position
  VectorX<double> stance_foot_pos(3);
  plant_.CalcPointsPositions(context, stance_contact_point_.second,
                             stance_contact_point_.first, plant_.world_frame(),
                             &stance_foot_pos);
  VectorX<double> st_to_CoM = CoM - stance_foot_pos;

  VectorX<double> feature(n_feature_y());
  if (world_dim_ == 2) {
    feature << st_to_CoM(0), st_to_CoM(2), mapping_basis().Eval(q);
  } else {
    feature << st_to_CoM, mapping_basis().Eval(q);
  }
  return feature;
}
drake::VectorX<double> Gip::EvalDynamicFeat(
    const drake::VectorX<double>& y, const drake::VectorX<double>& ydot,
    const drake::VectorX<double>& tau) const {
  double l = y.norm();
  if (l == 0) {
    cout << "avoid singularity in dynamics_expression\n";
    l = 1e-8;
  }

  VectorX<double> feature_extension(world_dim_ + 1);
  feature_extension.head(world_dim_) = y / l * tau(0) / total_mass_;
  feature_extension.tail(1) << -9.80665;

  VectorX<double> y_ydot_and_tau(2 * n_y() + n_tau());
  y_ydot_and_tau << y, ydot, tau;

  VectorX<double> feature(n_feature_yddot());
  feature << feature_extension, dynamic_basis().Eval(y_ydot_and_tau);
  return feature;
}
VectorX<double> Gip::EvalMappingFeatJV(const VectorX<double>& q,
                                       const VectorX<double>& v,
                                       const Context<double>& context) const {
  // Get CoM velocity
  VectorX<double> JV_CoM(3);
  if (use_pelvis) {
    // testing using pelvis
    MatrixX<double> J_com(3, plant_.num_velocities());
    plant_.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable::kV,
                                             pelvis_.second, pelvis_.first,
                                             world_, world_, &J_com);
    JV_CoM = J_com * v;
  } else {
    JV_CoM = plant_.CalcCenterOfMassTranslationalVelocityInWorld(context);
  }
  // Stance foot velocity
  MatrixX<double> J_sf(3, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV, stance_contact_point_.second,
      stance_contact_point_.first, world_, world_, &J_sf);
  VectorX<double> JV_st_to_CoM = JV_CoM - J_sf * v;

  // Convert v to qdot
  VectorX<double> qdot(plant_.num_positions());
  plant_.MapVelocityToQDot(context, v, &qdot);

  VectorX<double> ret(n_feature_y());
  if (world_dim_ == 2) {
    ret << JV_st_to_CoM(0), JV_st_to_CoM(2), mapping_basis().EvalJV(q, qdot);
  } else {
    ret << JV_st_to_CoM, mapping_basis().EvalJV(q, qdot);
  }
  return ret;
}
MatrixX<double> Gip::EvalMappingFeatJ(const VectorX<double>& q,
                                      const Context<double>& context) const {
  // Get CoM velocity
  MatrixX<double> J_com(3, plant_.num_velocities());
  if (use_pelvis) {
    // testing using pelvis
    plant_.CalcJacobianTranslationalVelocity(context, JacobianWrtVariable::kV,
                                             pelvis_.second, pelvis_.first,
                                             world_, world_, &J_com);
  } else {
    plant_.CalcJacobianCenterOfMassTranslationalVelocity(
        context, JacobianWrtVariable::kV, world_, world_, &J_com);
  }
  // Stance foot velocity
  MatrixX<double> J_sf(3, plant_.num_velocities());
  plant_.CalcJacobianTranslationalVelocity(
      context, JacobianWrtVariable::kV, stance_contact_point_.second,
      stance_contact_point_.first, world_, world_, &J_sf);
  MatrixX<double> J_st_to_CoM = J_com - J_sf;

  MatrixX<double> ret(n_feature_y(), plant_.num_velocities());
  if (world_dim_ == 2) {
    ret << J_st_to_CoM.row(0), J_st_to_CoM.row(2),
        is_quaternion_ ? JwrtqdotToJwrtv(q, mapping_basis().EvalJwrtqdot(q))
                       : mapping_basis().EvalJwrtqdot(q);
  } else {
    ret << J_st_to_CoM,
        is_quaternion_ ? JwrtqdotToJwrtv(q, mapping_basis().EvalJwrtqdot(q))
                       : mapping_basis().EvalJwrtqdot(q);
  }
  return ret;
}
VectorX<double> Gip::EvalMappingFeatJdotV(
    const VectorX<double>& q, const VectorX<double>& v,
    const Context<double>& context) const {
  // Get CoM JdotV
  VectorX<double> JdotV_com(3);
  if (use_pelvis) {
    // Testing: use pelvis origin
    JdotV_com = plant_.CalcBiasTranslationalAcceleration(
        context, JacobianWrtVariable::kV, pelvis_.second, pelvis_.first, world_,
        world_);
  } else {
    JdotV_com = plant_.CalcBiasCenterOfMassTranslationalAcceleration(
        context, JacobianWrtVariable::kV, world_, world_);
  }
  // Stance foot JdotV
  VectorX<double> JdotV_st = plant_.CalcBiasTranslationalAcceleration(
      context, JacobianWrtVariable::kV, stance_contact_point_.second,
      stance_contact_point_.first, world_, world_);
  VectorX<double> JdotV_st_to_com = JdotV_com - JdotV_st;

  // Convert v to qdot
  VectorX<double> qdot(plant_.num_positions());
  plant_.MapVelocityToQDot(context, v, &qdot);

  VectorX<double> ret(n_feature_y());
  if (world_dim_ == 2) {
    ret << JdotV_st_to_com(0), JdotV_st_to_com(2),
        mapping_basis().EvalJdotV(q, qdot);
  } else {
    ret << JdotV_st_to_com, mapping_basis().EvalJdotV(q, qdot);
  }
  return ret;
}

/// State Mirror
StateMirror::StateMirror(std::map<int, int> mirror_pos_index_map,
                         std::set<int> mirror_pos_sign_change_set,
                         std::map<int, int> mirror_vel_index_map,
                         std::set<int> mirror_vel_sign_change_set)
    : mirror_pos_index_map_(mirror_pos_index_map),
      mirror_pos_sign_change_set_(mirror_pos_sign_change_set),
      mirror_vel_index_map_(mirror_vel_index_map),
      mirror_vel_sign_change_set_(mirror_vel_sign_change_set) {}
VectorX<double> StateMirror::MirrorPos(const VectorX<double>& q) const {
  VectorX<double> ret = q;
  for (auto& index_pair : mirror_pos_index_map_) {
    ret(index_pair.first) = q(index_pair.second);
  }
  for (auto& index : mirror_pos_sign_change_set_) {
    ret(index) *= -1;
  }
  return ret;
}
VectorX<double> StateMirror::MirrorVel(const VectorX<double>& v) const {
  VectorX<double> ret = v;
  for (auto& index_pair : mirror_vel_index_map_) {
    ret(index_pair.first) = v(index_pair.second);
  }
  for (auto& index : mirror_vel_sign_change_set_) {
    ret(index) *= -1;
  }
  return ret;
}

/// MirroredReducedOrderModel
MirroredReducedOrderModel::MirroredReducedOrderModel(
    const MultibodyPlant<double>& plant, const ReducedOrderModel& original_rom,
    const StateMirror& state_mirror)
    : ReducedOrderModel(original_rom),
      plant_(plant),
      original_rom_(original_rom),
      state_mirror_(state_mirror),
      x_(VectorX<double>::Zero(plant.num_positions() + plant.num_velocities())),
      x_mirrored_(VectorX<double>::Zero(plant.num_positions() +
                                        plant.num_velocities())),
      context_mirrored_(plant.CreateDefaultContext()) {}
// Copy constructor
MirroredReducedOrderModel::MirroredReducedOrderModel(
    const MirroredReducedOrderModel& old_obj)
    : ReducedOrderModel(old_obj),
      plant_(old_obj.plant()),
      original_rom_(old_obj.original_rom()),
      state_mirror_(old_obj.state_mirror()),
      x_(VectorX<double>::Zero(plant_.num_positions() +
                               plant_.num_velocities())),
      x_mirrored_(VectorX<double>::Zero(plant_.num_positions() +
                                        plant_.num_velocities())),
      context_mirrored_(plant_.CreateDefaultContext()) {}

MatrixX<double> MirroredReducedOrderModel::EvalMappingFuncJ(
    const drake::VectorX<double>& q,
    const drake::systems::Context<double>&) const {
  MirrorPositionAndSetContextIfNew(q);

  MatrixX<double> J_wrt_vm = original_rom_.EvalMappingFuncJ(
      x_mirrored_.head(plant_.num_positions()), *context_mirrored_);

  // Apply chain rules on Jacobian (wrt the original robot state instead of wrt
  // the mirrored robot state).
  // For computational efficiency, we don't multiple J by the second term
  // in chain rule (which is a parse matrix containing 0, 1 and -1). Instead we
  // swap the columns and negate columns (I think this is faster...)
  drake::MatrixX<double> J_wrt_v = J_wrt_vm;
  for (auto& index_pair : state_mirror_.get_mirror_vel_index_map()) {
    J_wrt_v.col(index_pair.first) = J_wrt_vm.col(index_pair.second);
  }
  for (auto& index : state_mirror_.get_mirror_vel_sign_change_set()) {
    J_wrt_v.col(index) *= -1;
  }
  return J_wrt_v;
}

VectorX<double> MirroredReducedOrderModel::EvalMappingFeat(
    const VectorX<double>& q, const Context<double>&) const {
  MirrorPositionAndSetContextIfNew(q);

  return original_rom_.EvalMappingFeat(x_mirrored_.head(plant_.num_positions()),
                                       *context_mirrored_);
}
drake::VectorX<double> MirroredReducedOrderModel::EvalDynamicFeat(
    const drake::VectorX<double>& y, const drake::VectorX<double>& ydot,
    const drake::VectorX<double>& tau) const {
  return original_rom_.EvalDynamicFeat(y, ydot, tau);
}
VectorX<double> MirroredReducedOrderModel::EvalMappingFeatJV(
    const VectorX<double>& q, const VectorX<double>& v,
    const Context<double>&) const {
  VectorX<double> x(plant_.num_positions() + plant_.num_velocities());
  x << q, v;
  MirrorStateAndSetContextIfNew(x);

  return original_rom_.EvalMappingFeatJV(
      x_mirrored_.head(plant_.num_positions()),
      x_mirrored_.tail(plant_.num_velocities()), *context_mirrored_);
}
VectorX<double> MirroredReducedOrderModel::EvalMappingFeatJdotV(
    const VectorX<double>& q, const VectorX<double>& v,
    const Context<double>&) const {
  VectorX<double> x(plant_.num_positions() + plant_.num_velocities());
  x << q, v;
  MirrorStateAndSetContextIfNew(x);

  return original_rom_.EvalMappingFeatJdotV(
      x_mirrored_.head(plant_.num_positions()),
      x_mirrored_.tail(plant_.num_velocities()), *context_mirrored_);
}
void MirroredReducedOrderModel::MirrorPositionAndSetContextIfNew(
    const Eigen::VectorXd& q) const {
  if (!(q == x_.head(plant_.num_positions()))) {
    // Update key
    x_.head(plant_.num_positions()) = q;
    // Update mirrored state
    x_mirrored_.head(plant_.num_positions()) = state_mirror_.MirrorPos(q);
    // Update mirrored context
    plant_.SetPositionsAndVelocities(context_mirrored_.get(), x_mirrored_);
  }
}
void MirroredReducedOrderModel::MirrorStateAndSetContextIfNew(
    const Eigen::VectorXd& x) const {
  if (!(x == x_)) {
    // Update key
    x_ = x;
    // Update mirrored state
    x_mirrored_ << state_mirror_.MirrorPos(x.head(plant_.num_positions())),
        state_mirror_.MirrorVel(x.tail(plant_.num_velocities()));
    // Update mirrored context
    plant_.SetPositionsAndVelocities(context_mirrored_.get(), x_mirrored_);
  }
}

/// 3D Center of mass model (only for testing)
/// Note that this is not LIPM. The COM is not wrt stance foot.
testing::Com::Com(const drake::multibody::MultibodyPlant<double>& plant,
                  const MonomialFeatures& mapping_basis,
                  const MonomialFeatures& dynamic_basis)
    : ReducedOrderModel(3, 0, MatrixX<double>::Zero(3, 0),
                        3 + mapping_basis.length(), 2 + dynamic_basis.length(),
                        mapping_basis, dynamic_basis, {}, "COM"),
      plant_(plant),
      world_(plant_.world_frame()) {
  // Initialize model parameters (dependant on the feature vectors)
  VectorX<double> theta_y = VectorX<double>::Zero(n_y() * n_feature_y());
  theta_y(0) = 1;
  theta_y(1 + n_feature_y()) = 1;
  theta_y(2 + 2 * n_feature_y()) = 1;
  SetThetaY(theta_y);

  VectorX<double> theta_yddot =
      VectorX<double>::Zero(n_yddot() * n_feature_yddot());
  theta_yddot(0) = 1;
  theta_yddot(1 + n_feature_yddot()) = 1;
  SetThetaYddot(theta_yddot);

  is_quaternion_ = isQuaternion(plant);

  // Always check dimension after model construction
  CheckModelConsistency();
};

// Copy constructor for the Clone() method
testing::Com::Com(const testing::Com& old_obj)
    : ReducedOrderModel(old_obj),
      plant_(old_obj.plant()),
      world_(old_obj.world()),
      is_quaternion_(isQuaternion(old_obj.plant())){};

// Evaluators for features of y, yddot, y's Jacobian and y's JdotV
VectorX<double> testing::Com::EvalMappingFeat(
    const VectorX<double>& q,
    const drake::systems::Context<double>& context) const {
  // Get CoM position
  VectorX<double> CoM = plant_.CalcCenterOfMassPositionInWorld(context);

  VectorX<double> feature(n_feature_y());
  feature << CoM, mapping_basis().Eval(q);
  return feature;
};
drake::VectorX<double> testing::Com::EvalDynamicFeat(
    const drake::VectorX<double>& y, const drake::VectorX<double>& ydot,
    const drake::VectorX<double>& tau) const {
  VectorX<double> feature(n_feature_yddot());
  cout << "Warning: EvalDynamicFeat is not implemented\n";
  return feature;
};
VectorX<double> testing::Com::EvalMappingFeatJV(
    const VectorX<double>& q, const VectorX<double>& v,
    const drake::systems::Context<double>& context) const {
  // Get CoM velocity
  MatrixX<double> J_com(3, plant_.num_velocities());
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      context, JacobianWrtVariable::kV, world_, world_, &J_com);
  VectorX<double> JV_CoM = J_com * v;

  // Convert v to qdot
  VectorX<double> qdot(plant_.num_positions());
  plant_.MapVelocityToQDot(context, v, &qdot);

  VectorX<double> ret(n_feature_y());
  ret << JV_CoM, mapping_basis().EvalJV(q, qdot);
  return ret;
};
VectorX<double> testing::Com::EvalMappingFeatJdotV(
    const VectorX<double>& q, const VectorX<double>& v,
    const drake::systems::Context<double>& context) const {
  // Get CoM JdotV
  VectorX<double> JdotV_com =
      plant_.CalcBiasCenterOfMassTranslationalAcceleration(
          context, JacobianWrtVariable::kV, world_, world_);

  // Convert v to qdot
  VectorX<double> qdot(plant_.num_positions());
  plant_.MapVelocityToQDot(context, v, &qdot);

  VectorX<double> ret(n_feature_y());
  ret << JdotV_com, mapping_basis().EvalJdotV(q, qdot);
  return ret;
};
MatrixX<double> testing::Com::EvalMappingFeatJ(
    const VectorX<double>& q,
    const drake::systems::Context<double>& context) const {
  // Get CoM J
  MatrixX<double> J_com(3, plant_.num_velocities());
  plant_.CalcJacobianCenterOfMassTranslationalVelocity(
      context, JacobianWrtVariable::kV, world_, world_, &J_com);

  MatrixX<double> ret(n_feature_y(), plant_.num_velocities());
  ret << J_com, is_quaternion_
                    ? JwrtqdotToJwrtv(q, mapping_basis().EvalJwrtqdot(q))
                    : mapping_basis().EvalJwrtqdot(q);
  return ret;
};

}  // namespace goldilocks_models
}  // namespace dairlib
