#include "examples/Cassie/cassie_rbt_state_estimator.h"
#include <math.h>

namespace dairlib {
namespace systems {

using std::cout;
using std::endl;
using std::string;
using std::vector;
using std::pow;
using std::sqrt;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using Eigen::Isometry3d;
using Eigen::Isometry;
using Eigen::Transform;

using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::systems::DiscreteStateIndex;
using drake::systems::EventStatus;
using drake::systems::LeafSystem;

using dairlib::multibody::GetBodyIndexFromName;
using dairlib::systems::OutputVector;

CassieRbtStateEstimator::CassieRbtStateEstimator(
    const RigidBodyTree<double>& tree, MatrixXd X_init, VectorXd ekf_bias_init,
    bool is_floating_base, MatrixXd P_init, MatrixXd N_prior,
    VectorXd gyro_noise_std, VectorXd accel_noise_std,
    VectorXd contact_noise_std, VectorXd gyro_bias_noise_std,
    VectorXd accel_bias_noise_std, VectorXd joints_noise_std)
    : tree_(tree),
      X_init_(X_init),
      ekf_bias_init_(ekf_bias_init),
      P_init_(P_init),
      is_floating_base_(is_floating_base),
      gyro_noise_std_(gyro_noise_std),
      accel_noise_std_(accel_noise_std),
      contact_noise_std_(contact_noise_std),
      gyro_bias_noise_std_(gyro_bias_noise_std),
      accel_bias_noise_std_(accel_bias_noise_std),
      joints_noise_std_(joints_noise_std) {
  actuatorIndexMap_ = multibody::makeNameToActuatorsMap(tree);
  positionIndexMap_ = multibody::makeNameToPositionsMap(tree);
  velocityIndexMap_ = multibody::makeNameToVelocitiesMap(tree);

  this->DeclareAbstractInputPort("cassie_out_t", drake::Value<cassie_out_t>{});
  this->DeclareVectorOutputPort(
      OutputVector<double>(tree.get_num_positions(), tree.get_num_velocities(),
                           tree.get_num_actuators()),
      &CassieRbtStateEstimator::CopyStateOut);

  DeclarePerStepDiscreteUpdateEvent(&CassieRbtStateEstimator::Update);

  VectorXd ekf_x_init = ComputeEkfX(X_init);
  VectorXd ekf_p_init = ComputeEkfP(P_init);

  // Declaring the discrete states with initial values
  // Verifying the dimensions of the initial state values
  DRAKE_ASSERT(ekf_x_init.size() == num_states_total_);
  DRAKE_ASSERT(ekf_bias_init.size() == num_states_bias_);

  state_idx_ = DeclareDiscreteState(
      VectorXd::Zero(num_states_required_));      // estimated floating base
  ekf_x_idx_ = DeclareDiscreteState(ekf_x_init);  // estimated EKF state
  ekf_bias_idx_ = DeclareDiscreteState(ekf_bias_init);  // estimated bias state
  ekf_p_idx_ = DeclareDiscreteState(ekf_p_init);  // estimated state covariance
  time_idx_ = DeclareDiscreteState(VectorXd::Zero(1));  // previous time
  // }

  // Initialize body indices
  left_thigh_ind_ = GetBodyIndexFromName(tree, "thigh_left");
  right_thigh_ind_ = GetBodyIndexFromName(tree, "thigh_right");
  left_heel_spring_ind_ = GetBodyIndexFromName(tree, "heel_spring_left");
  right_heel_spring_ind_ = GetBodyIndexFromName(tree, "heel_spring_right");
  if (left_thigh_ind_ == -1 || right_thigh_ind_ == -1 ||
      left_heel_spring_ind_ == -1 || right_heel_spring_ind_ == -1)
    std::cout << "In cassie_rbt_state_estimator.cc,"
                 " body indices were not set correctly.\n";

  // Initializing the contact indicator vector. Initially we assume both the
  // feet to be in contact
  contacts_ = new vector<int>(4, 1);

  // The local collision coordinates wrt to the toe frames (Same for both legs)
  // It is obtained from the Cassie_v2 urdf.
  local_collision_pt1_.resize(3);
  local_collision_pt2_.resize(3);
  local_collision_pt1_ << -0.0457, 0.112, 0;
  local_collision_pt2_ << 0.088, 0, 0;

  // Homogeneous collision coordinates
  local_collision_pt1_hom_.resize(4);
  local_collision_pt2_hom_.resize(4);
  local_collision_pt1_hom_.head(3) = local_collision_pt1_;
  local_collision_pt2_hom_.head(3) = local_collision_pt2_;
  local_collision_pt1_hom_(3) = 1;
  local_collision_pt2_hom_(3) = 1;

  // Initializing the constant Eigen constructs
  g_.resize(3);
  g_ << 0, 0, -9.81;

  N_prior_ = N_prior;
}

void CassieRbtStateEstimator::solveFourbarLinkage(
    VectorXd q_init, double& left_heel_spring,
    double& right_heel_spring) const {
  // TODO(yminchen): get the numbers below from tree
  // Get the rod length
  Vector3d rod_on_heel_spring(.11877, -.01, 0.0);
  double spring_length = rod_on_heel_spring.norm();
  // Spring rest angle offset
  double spring_rest_offset =
      atan(rod_on_heel_spring(1) / rod_on_heel_spring(0));

  // Get the rod length projected to thigh-shin plane
  double rod_length = 0.5012;  // from cassie_utils
  Vector3d rod_on_thigh_left(0.0, 0.0, 0.045);
  Vector3d rod_on_thigh_right(0.0, 0.0, -0.045);

  std::vector<Vector3d> rod_on_thigh{rod_on_thigh_left, rod_on_thigh_right};
  std::vector<int> thigh_ind{left_thigh_ind_, right_thigh_ind_};
  std::vector<int> heel_spring_ind{left_heel_spring_ind_,
                                   right_heel_spring_ind_};

  KinematicsCache<double> cache = tree_.doKinematics(q_init);
  for (int i = 0; i < 2; i++) {
    // Get thigh pose and heel spring pose
    const Isometry3d thigh_pose =
        tree_.CalcBodyPoseInWorldFrame(cache, tree_.get_body(thigh_ind[i]));
    const Vector3d thigh_pos = thigh_pose.translation();
    const MatrixXd thigh_rot_mat = thigh_pose.linear();

    const Isometry3d heel_spring_pose = tree_.CalcBodyPoseInWorldFrame(
        cache, tree_.get_body(heel_spring_ind[i]));
    const Vector3d heel_spring_pos = heel_spring_pose.translation();
    const MatrixXd heel_spring_rot_mat = heel_spring_pose.linear();

    // Get r_heel_spring_base_to_thigh_ball_joint
    Vector3d r_though_ball_joint = thigh_pos + thigh_rot_mat * rod_on_thigh[i];
    Vector3d r_heel_spring_base_to_thigh_ball_joint =
        r_though_ball_joint - heel_spring_pos;
    Vector3d r_thigh_ball_joint_wrt_heel_spring_base =
        heel_spring_rot_mat.transpose() *
        r_heel_spring_base_to_thigh_ball_joint;

    // Get the projected rod length in the xy plane of heel spring base
    double projected_rod_length =
        sqrt(pow(rod_length, 2) -
             pow(r_thigh_ball_joint_wrt_heel_spring_base(2), 2));

    // Get the vector of the deflected spring direction
    // Below solves for the intersections of two circles on a plane
    double x_tbj_wrt_hb = r_thigh_ball_joint_wrt_heel_spring_base(0);
    double y_tbj_wrt_hb = r_thigh_ball_joint_wrt_heel_spring_base(1);

    double k = -y_tbj_wrt_hb / x_tbj_wrt_hb;
    double c = (pow(spring_length, 2) - pow(projected_rod_length, 2) +
                pow(x_tbj_wrt_hb, 2) + pow(y_tbj_wrt_hb, 2)) /
               (2 * x_tbj_wrt_hb);

    double y_sol_1 =
        (-k * c + sqrt(pow(k * c, 2) -
                       (pow(k, 2) + 1) * (pow(c, 2) - pow(spring_length, 2)))) /
        (pow(k, 2) + 1);
    double y_sol_2 =
        (-k * c - sqrt(pow(k * c, 2) -
                       (pow(k, 2) + 1) * (pow(c, 2) - pow(spring_length, 2)))) /
        (pow(k, 2) + 1);
    double x_sol_1 = k * y_sol_1 + c;
    double x_sol_2 = k * y_sol_2 + c;

    Vector3d sol_1_wrt_heel_base(x_sol_1, y_sol_1, 0);
    Vector3d sol_2_wrt_heel_base(x_sol_2, y_sol_2, 0);
    Vector3d sol_1_cross_sol_2 = sol_1_wrt_heel_base.cross(sol_2_wrt_heel_base);

    Vector3d r_sol_wrt_heel_base =
        (sol_1_cross_sol_2(2) >= 0) ? sol_2_wrt_heel_base : sol_1_wrt_heel_base;

    // Get the heel spring deflection direction and magnitude
    const Vector3d spring_rest_dir(1, 0, 0);
    double heel_spring_angle =
        acos(r_sol_wrt_heel_base.dot(spring_rest_dir) /
             (r_sol_wrt_heel_base.norm() * spring_rest_dir.norm()));
    Vector3d r_rest_dir_cross_r_hs_to_sol =
        spring_rest_dir.cross(r_sol_wrt_heel_base);
    int spring_deflect_sign = (r_rest_dir_cross_r_hs_to_sol(2) >= 0) ? 1 : -1;
    if (i == 0)
      left_heel_spring =
          spring_deflect_sign * heel_spring_angle - spring_rest_offset;
    else
      right_heel_spring =
          spring_deflect_sign * heel_spring_angle - spring_rest_offset;
  }  // end for
}

MatrixXd CassieRbtStateEstimator::ExtractRotationMatrix(VectorXd ekf_x) const {
  DRAKE_ASSERT(ekf_x.size() == num_states_total_);

  MatrixXd R(3, 3);
  R.row(0) = ekf_x.segment(0, 3);
  R.row(1) = ekf_x.segment(3, 3);
  R.row(2) = ekf_x.segment(6, 3);

  // Making sure that R is orthogonal
  DRAKE_ASSERT(MatrixXd::Identity(3, 3).isApprox(R.transpose() * R));

  return R;
}

VectorXd CassieRbtStateEstimator::ExtractFloatingBaseVelocities(
    VectorXd ekf_x) const {
  DRAKE_ASSERT(ekf_x.size() == num_states_total_);

  return ekf_x.segment(9, 3);
}

VectorXd CassieRbtStateEstimator::ExtractFloatingBasePositions(
    VectorXd ekf_x) const {
  DRAKE_ASSERT(ekf_x.size() == num_states_total_);

  return ekf_x.segment(12, 3);
}

int CassieRbtStateEstimator::ComputeNumContacts() const {
  return std::count(contacts_->begin(), contacts_->end(), 1);
}

MatrixXd CassieRbtStateEstimator::ExtractContactPositions(
    VectorXd ekf_x) const {
  DRAKE_ASSERT(ekf_x.size() == num_states_total_);

  MatrixXd d = MatrixXd::Zero(3, num_contacts_);
  for (int i = 0; i < num_contacts_; ++i) {
    d.col(i) = ekf_x.segment(15 + 3 * i, 3);
  }

  return d;
}

MatrixXd CassieRbtStateEstimator::CreateSkewSymmetricMatrix(VectorXd s) const {
  // Making sure the the input vector is of the right size
  DRAKE_ASSERT(s.size() == 3);

  MatrixXd S = MatrixXd::Zero(3, 3);
  S(0, 1) = -s(2);
  S(0, 2) = s(1);
  S(1, 0) = s(2);
  S(1, 2) = -s(0);
  S(2, 0) = -s(1);
  S(2, 1) = s(0);

  return S;
}

MatrixXd CassieRbtStateEstimator::ComputeLieExponential(VectorXd xi) const {
  DRAKE_ASSERT(xi.size() == 21);

  MatrixXd Lg = MatrixXd::Zero(5 + num_contacts_, 5 + num_contacts_);
  Lg.block(0, 0, 3, 3) = CreateSkewSymmetricMatrix(xi.head(3));
  Lg.block(0, 3, 3, 1) = xi.segment(3, 3);
  Lg.block(0, 4, 3, 1) = xi.segment(6, 3);

  for (int i = 0; i < num_contacts_; ++i) {
    Lg.block(0, 5 + i, 3, 1) = xi.segment(9 + 3 * i, 3);
  }

  return Lg.exp();
}

Eigen::MatrixXd CassieRbtStateEstimator::ComputeTransformationToeLeftWrtIMU(
    Eigen::VectorXd q) const {
  DRAKE_ASSERT(q.size() == tree_.get_num_positions());

  KinematicsCache<double> cache = tree_.doKinematics(q);
  int pelvis_ind = GetBodyIndexFromName(tree_, "pelvis");
  int toe_left_ind = GetBodyIndexFromName(tree_, "toe_left");
  Transform<double, 3, Isometry> T =
      tree_.relativeTransform(cache, pelvis_ind, toe_left_ind);

  return T.matrix();
}

Eigen::MatrixXd CassieRbtStateEstimator::ComputeTransformationToeRightWrtIMU(
    Eigen::VectorXd q) const {
  DRAKE_ASSERT(q.size() == tree_.get_num_positions());

  KinematicsCache<double> cache = tree_.doKinematics(q);
  int pelvis_ind = GetBodyIndexFromName(tree_, "pelvis");
  int toe_right_ind = GetBodyIndexFromName(tree_, "toe_right");
  Transform<double, 3, Isometry> T =
      tree_.relativeTransform(cache, pelvis_ind, toe_right_ind);

  return T.matrix();
}

Eigen::MatrixXd CassieRbtStateEstimator::ComputeRotationToeLeftWrtIMU(
    Eigen::VectorXd q) const {
  DRAKE_ASSERT(q.size() == tree_.get_num_positions());
  return ComputeTransformationToeLeftWrtIMU(q).block(0, 0, 3, 3);
}

Eigen::MatrixXd CassieRbtStateEstimator::ComputeRotationToeRightWrtIMU(
    Eigen::VectorXd q) const {
  DRAKE_ASSERT(q.size() == tree_.get_num_positions());
  return ComputeTransformationToeRightWrtIMU(q).block(0, 0, 3, 3);
}

Eigen::MatrixXd CassieRbtStateEstimator::ComputeToeLeftCollisionPointsWrtIMU(
    Eigen::VectorXd q) const {
  DRAKE_ASSERT(q.size() == tree_.get_num_positions());
  MatrixXd T_left = ComputeTransformationToeLeftWrtIMU(q);

  MatrixXd collision_pts = MatrixXd::Zero(3, 2);
  VectorXd collision_pt1_hom = T_left * local_collision_pt1_hom_;
  VectorXd collision_pt2_hom = T_left * local_collision_pt2_hom_;
  collision_pts.col(0) = collision_pt1_hom.head(3);
  collision_pts.col(1) = collision_pt2_hom.head(3);

  return collision_pts;
}

Eigen::MatrixXd CassieRbtStateEstimator::ComputeToeRightCollisionPointsWrtIMU(
    Eigen::VectorXd q) const {
  DRAKE_ASSERT(q.size() == tree_.get_num_positions());
  MatrixXd T_right = ComputeTransformationToeRightWrtIMU(q);

  MatrixXd collision_pts = MatrixXd::Zero(3, 2);
  VectorXd collision_pt1_hom = T_right * local_collision_pt1_hom_;
  VectorXd collision_pt2_hom = T_right * local_collision_pt2_hom_;
  collision_pts.col(0) = collision_pt1_hom.head(3);
  collision_pts.col(1) = collision_pt2_hom.head(3);

  return collision_pts;
}

Eigen::MatrixXd CassieRbtStateEstimator::ComputeToeLeftJacobianWrtIMU(
    VectorXd q, VectorXd p) const {
  DRAKE_ASSERT(q.size() == tree_.get_num_positions());
  DRAKE_ASSERT(p.size() == 3);

  KinematicsCache<double> cache = tree_.doKinematics(q);
  int pelvis_ind = GetBodyIndexFromName(tree_, "pelvis");
  int toe_left_ind = GetBodyIndexFromName(tree_, "toe_left");
  MatrixXd J =
      tree_.transformPointsJacobian(cache, p, toe_left_ind, pelvis_ind, false);

  // Returning the columns that correspond to the angles (ignoring the floating
  // base coordinates) as the linearization only depends on the measured joint
  // angles and is independent of the floating base coordinates.
  return J.block(0, J.cols() - num_joints_, J.rows(), num_joints_);
}

Eigen::MatrixXd CassieRbtStateEstimator::ComputeToeRightJacobianWrtIMU(
    VectorXd q, VectorXd p) const {
  DRAKE_ASSERT(q.size() == tree_.get_num_positions());
  DRAKE_ASSERT(p.size() == 3);

  KinematicsCache<double> cache = tree_.doKinematics(q);
  int pelvis_ind = GetBodyIndexFromName(tree_, "pelvis");
  int toe_right_ind = GetBodyIndexFromName(tree_, "toe_right");
  MatrixXd J =
      tree_.transformPointsJacobian(cache, p, toe_right_ind, pelvis_ind, false);

  // Returning the columns that correspond to the angles (ignoring the floating
  // base coordinates) as the linearization only depends on the measured joint
  // angles and is independent of the floating base coordinates.
  return J.block(0, J.cols() - num_joints_, J.rows(), num_joints_);
}

MatrixXd CassieRbtStateEstimator::ComputeX(VectorXd ekf_x) const {
  DRAKE_ASSERT(ekf_x.size() == num_states_total_);

  MatrixXd R = ExtractRotationMatrix(ekf_x);
  VectorXd v = ExtractFloatingBaseVelocities(ekf_x);
  VectorXd p = ExtractFloatingBasePositions(ekf_x);
  MatrixXd d = ExtractContactPositions(ekf_x);

  int n = 9;

  MatrixXd X = MatrixXd::Zero(n, n);

  X.block(0, 0, 3, 3) = ExtractRotationMatrix(ekf_x);

  // Floating base velocitie
  X.block(0, 3, 3, 1) = ExtractFloatingBaseVelocities(ekf_x);

  // Floating base position
  X.block(0, 4, 3, 1) = ExtractFloatingBasePositions(ekf_x);

  // Contact points
  X.block(0, 5, 3, num_contacts_) = d;

  // Identity block
  X.block(3, 3, num_contacts_ + 2, num_contacts_ + 2) =
      MatrixXd::Identity(num_contacts_ + 2, num_contacts_ + 2);

  return X;
}

MatrixXd CassieRbtStateEstimator::ComputeX(MatrixXd R, VectorXd v, VectorXd p,
                                           MatrixXd d) const {
  DRAKE_ASSERT(R.rows() == 3);
  DRAKE_ASSERT(R.cols() == 3);
  DRAKE_ASSERT(v.size() == 3);
  DRAKE_ASSERT(p.size() == 3);
  DRAKE_ASSERT(d.rows() == 3);
  DRAKE_ASSERT(d.cols() == 4);

  int n = 3 + 2 + num_contacts_;

  MatrixXd X = MatrixXd::Zero(n, n);

  X.block(0, 0, 3, 3) = R;
  X.block(0, 3, 3, 1) = v;
  X.block(0, 4, 3, 1) = p;
  X.block(0, 5, 3, num_contacts_) = d;
  X.block(3, 3, num_contacts_ + 2, num_contacts_ + 2) =
      MatrixXd::Identity(num_contacts_ + 2, num_contacts_ + 2);

  return X;
}

VectorXd CassieRbtStateEstimator::ComputeEkfX(MatrixXd X) const {
  DRAKE_ASSERT(X.rows() == 9);
  DRAKE_ASSERT(X.cols() == 9);

  MatrixXd R = X.block(0, 0, 3, 3);
  VectorXd r1 = R.row(0);
  VectorXd r2 = R.row(1);
  VectorXd r3 = R.row(2);
  VectorXd v = X.block(0, 3, 3, 1);
  VectorXd p = X.block(0, 4, 3, 1);
  VectorXd d1 = X.block(0, 5, 3, 1);
  VectorXd d2 = X.block(0, 6, 3, 1);
  VectorXd d3 = X.block(0, 7, 3, 1);
  VectorXd d4 = X.block(0, 8, 3, 1);

  VectorXd ekf_x(num_states_total_);
  ekf_x << r1, r2, r3, v, p, d1, d2, d3, d4;
  return ekf_x;
}

MatrixXd CassieRbtStateEstimator::ComputeP(VectorXd ekf_p) const {
  int n = sqrt(ekf_p.size());
  MatrixXd P(n, n);
  for (int i = 0; i < n; ++i) {
    P.row(i) = ekf_p.segment(i * n, n);
  }

  return P;
}

VectorXd CassieRbtStateEstimator::ComputeEkfP(MatrixXd P) const {
  DRAKE_ASSERT(P.rows() == P.cols());
  int n = P.rows();
  VectorXd ekf_p(n * n);
  for (int i = 0; i < n; ++i) {
    ekf_p.segment(i * n, n) = P.row(i);
  }

  return ekf_p;
}

MatrixXd CassieRbtStateEstimator::PredictX(VectorXd ekf_x, VectorXd ekf_bias,
                                           VectorXd u, VectorXd q,
                                           double dt) const {
  DRAKE_ASSERT(ekf_x.size() == num_states_total_);
  DRAKE_ASSERT(ekf_bias.size() == num_states_bias_);
  DRAKE_ASSERT(u.size() == num_inputs_);
  DRAKE_ASSERT(q.size() == tree_.get_num_positions());

  // Extracting the current states
  MatrixXd R = ExtractRotationMatrix(ekf_x);
  VectorXd v = ExtractFloatingBaseVelocities(ekf_x);
  VectorXd p = ExtractFloatingBasePositions(ekf_x);
  MatrixXd d = ExtractContactPositions(ekf_x);

  VectorXd angular_velocity_bias = ekf_bias.head(3);
  VectorXd linear_acceleration_bias = ekf_bias.tail(3);

  VectorXd angular_velocity = u.head(3);
  VectorXd linear_acceleration = u.tail(3);

  VectorXd corrected_angular_velocity =
      angular_velocity - angular_velocity_bias;
  VectorXd corrected_linear_acceleration =
      linear_acceleration - linear_acceleration_bias;

  // Predicted components
  MatrixXd R_pred =
      R * (CreateSkewSymmetricMatrix(corrected_angular_velocity * dt).exp());
  VectorXd v_pred = v + (R * corrected_linear_acceleration + g_) * dt;
  VectorXd p_pred =
      p + v * dt + 0.5 * (R * corrected_linear_acceleration + g_) * dt * dt;

  // Contact prediction
  MatrixXd collision_pts_left = ComputeToeLeftCollisionPointsWrtIMU(q);
  MatrixXd collision_pts_right = ComputeToeRightCollisionPointsWrtIMU(q);

  // Matrix stacking both the collision coordinates so the prediction step may
  // be completed in a single loop.
  MatrixXd collision_pts(3, num_contacts_);
  collision_pts << collision_pts_left, collision_pts_right;

  MatrixXd d_pred = d;

  // If the respective contact point is active (In contact at this step), then
  // the prediction is unchanged. If not, the prediction is changed to the
  // computed predicted coordinate of the collision point.
  for (int i = 0; i < num_contacts_; ++i) {
    if (!contacts_->at(i)) {
      d_pred.col(i) = p_pred + (R_pred * collision_pts.col(i));
    }
  }

  // Constructing and returning the state matrix
  return ComputeX(R_pred, v_pred, p_pred, d_pred);
}

VectorXd CassieRbtStateEstimator::PredictBias(VectorXd ekf_bias,
                                              double dt) const {
  DRAKE_ASSERT(ekf_bias.size() == num_states_bias_);
  return ekf_bias;
}

MatrixXd CassieRbtStateEstimator::ComputeAdjointOperator(
    Eigen::VectorXd ekf_x) const {
  DRAKE_ASSERT(ekf_x.size() == num_states_total_);

  // The matrix consists of 3 + num_contacts blocks of 3x3
  int n = 3 * (3 + num_contacts_);

  // Filling up the rotation matrices
  MatrixXd Adj = MatrixXd::Zero(n, n);
  MatrixXd R = ExtractRotationMatrix(ekf_x);
  VectorXd v = ExtractFloatingBaseVelocities(ekf_x);
  VectorXd p = ExtractFloatingBasePositions(ekf_x);
  MatrixXd d = ExtractContactPositions(ekf_x);

  for (int i = 0; i < 3 + num_contacts_; ++i) {
    Adj.block(i * 3, i * 3, 3, 3) = R;
  }

  // Filling up the skew dependent terms
  Adj.block(3, 0, 3, 3) = CreateSkewSymmetricMatrix(v) * R;
  Adj.block(6, 0, 3, 3) = CreateSkewSymmetricMatrix(p) * R;

  // Filling up the skew dependent contact terms
  for (int i = 0; i < num_contacts_; ++i) {
    Adj.block(9 + i * 3, 0, 3, 3) = CreateSkewSymmetricMatrix(d.col(i)) * R;
  }

  return Adj;
}

MatrixXd CassieRbtStateEstimator::ComputeA(VectorXd ekf_x) const {
  DRAKE_ASSERT(ekf_x.size() == num_states_total_);

  // Size of the A matrix
  // 9 = 3 + 2 + 4 (3 for R, p and v, 2 for the bias terms, and 4 for the
  // contacts)
  int n = 3 * (3 + 2 + num_contacts_);

  MatrixXd R = ExtractRotationMatrix(ekf_x);
  VectorXd v = ExtractFloatingBaseVelocities(ekf_x);
  VectorXd p = ExtractFloatingBasePositions(ekf_x);
  MatrixXd d = ExtractContactPositions(ekf_x);

  MatrixXd A = MatrixXd::Zero(n, n);
  MatrixXd g_skew = CreateSkewSymmetricMatrix(g_);

  A.block(3, 0, 3, 3) = g_skew;
  A.block(6, 3, 3, 3) = MatrixXd::Identity(3, 3);

  // Skew terms
  A.block(0, n - 6, 3, 3) = -R;
  A.block(3, n - 3, 3, 3) = -R;
  A.block(3, n - 6, 3, 3) = -CreateSkewSymmetricMatrix(v) * R;
  A.block(6, n - 6, 3, 3) = -CreateSkewSymmetricMatrix(p) * R;

  // Contact skew terms
  for (int i = 0; i < num_contacts_; ++i) {
    A.block(9 + i * 3, n - 6, 3, 3) = -CreateSkewSymmetricMatrix(d.col(i)) * R;
  }

  return A;
}

Eigen::MatrixXd CassieRbtStateEstimator::ComputeCov(Eigen::VectorXd q) const {
  DRAKE_ASSERT(q.size() == tree_.get_num_positions());
  MatrixXd Qg = MatrixXd::Zero(3, 3);
  MatrixXd Qa = MatrixXd::Zero(3, 3);
  MatrixXd Qc = MatrixXd::Zero(3, 3);
  MatrixXd Qbg = MatrixXd::Zero(3, 3);
  MatrixXd Qba = MatrixXd::Zero(3, 3);

  // Individual covariance matrices
  for (int i = 0; i < 3; ++i) {
    Qg(i, i) = pow(gyro_noise_std_(i), 2);
    Qa(i, i) = pow(accel_noise_std_(i), 2);
    Qc(i, i) = pow(contact_noise_std_(i), 2);
    Qbg(i, i) = pow(gyro_bias_noise_std_(i), 2);
    Qba(i, i) = pow(accel_bias_noise_std_(i), 2);
  }

  // wg, wa, 0, wbg, wba and 4 contacts (5 + num_contacts)
  int n = 3 * (5 + num_contacts_);
  MatrixXd Cov = MatrixXd::Zero(n, n);

  MatrixXd R_left = ComputeRotationToeLeftWrtIMU(q);
  MatrixXd R_right = ComputeRotationToeRightWrtIMU(q);

  Cov.block(0, 0, 3, 3) = Qg;
  Cov.block(3, 3, 3, 3) = Qa;
  Cov.block(6, 6, 3, 3) = MatrixXd::Zero(3, 3);

  // For the contact points, we increase the covariance values if there is
  // no active contact.
  Cov.block(9, 9, 3, 3) =
      R_left *
      (Qc + ((1 - contacts_->at(0)) * 1e4 * MatrixXd::Identity(3, 3))) *
      (R_left.transpose());
  Cov.block(12, 12, 3, 3) =
      R_left *
      (Qc + ((1 - contacts_->at(1)) * 1e4 * MatrixXd::Identity(3, 3))) *
      (R_left.transpose());
  Cov.block(15, 15, 3, 3) =
      R_right *
      (Qc + ((1 - contacts_->at(2)) * 1e4 * MatrixXd::Identity(3, 3))) *
      (R_right.transpose());
  Cov.block(18, 18, 3, 3) =
      R_right *
      (Qc + ((1 - contacts_->at(3)) * 1e4 * MatrixXd::Identity(3, 3))) *
      (R_right.transpose());

  // Bias covariances
  Cov.block(21, 21, 3, 3) = Qbg;
  Cov.block(24, 24, 3, 3) = Qba;

  return Cov;
}

Eigen::MatrixXd CassieRbtStateEstimator::PredictP(VectorXd ekf_x,
                                                  VectorXd ekf_p, VectorXd q,
                                                  double dt) const {
  DRAKE_ASSERT(ekf_x.size() == num_states_total_);
  DRAKE_ASSERT(ekf_p.size() == num_states_total_ * num_states_total_);
  DRAKE_ASSERT(q.size() == tree_.get_num_positions());

  MatrixXd Adj = ComputeAdjointOperator(ekf_x);

  // Adjoint including bias
  MatrixXd Adj_bias = MatrixXd::Zero(Adj.rows() + 6, Adj.cols() + 6);
  Adj_bias.block(0, 0, Adj.rows(), Adj.cols()) = Adj;
  Adj_bias.block(Adj.rows(), Adj.cols(), 6, 6) = MatrixXd::Identity(6, 6);

  MatrixXd A = ComputeA(ekf_x);
  MatrixXd Cov = ComputeCov(q);

  // Discretizing A
  MatrixXd A_k = MatrixXd::Identity(A.rows(), A.cols()) + A * dt;

  // Constructing the P matrix
  MatrixXd P = ComputeP(ekf_p);

  // Discretized noise matrix
  MatrixXd Q_k =
      A_k * Adj_bias * Cov * Adj_bias.transpose() * A_k.transpose() * dt;

  MatrixXd P_pred = A_k * P * A_k.transpose() + Q_k;

  return P_pred;
}

void CassieRbtStateEstimator::ComputeUpdateParams(
    VectorXd ekf_x_predicted, VectorXd ekf_p_predicted, VectorXd q, VectorXd& y,
    VectorXd& b, MatrixXd& H, MatrixXd& N, MatrixXd& Pi, MatrixXd& X_full,
    MatrixXd& S, MatrixXd& K, VectorXd& delta) const {
  DRAKE_ASSERT(ekf_x_predicted.size() == num_states_total_);
  DRAKE_ASSERT(ekf_p_predicted.size() == num_states_total_ * num_states_total_);
  DRAKE_ASSERT(q.size() == tree_.get_num_positions());

  // The ekf_x argument is the state vector after the prediction step.
  MatrixXd R_predicted = ExtractRotationMatrix(ekf_x_predicted);
  MatrixXd X_predicted = ComputeX(ekf_x_predicted);

  MatrixXd collision_pts_left = ComputeToeLeftCollisionPointsWrtIMU(q);
  MatrixXd collision_pts_right = ComputeToeRightCollisionPointsWrtIMU(q);

  int y_size = 5 + num_contacts_;
  int b_size = y_size;
  int h_size = (3 + num_contacts_ + 2) * 3;
  int pi_size = 3 + num_contacts_ + 2;

  VectorXd y1(y_size), y2(y_size), y3(y_size), y4(y_size);
  VectorXd b1(b_size), b2(b_size), b3(b_size), b4(b_size);
  MatrixXd H1, H2, H3, H4;
  MatrixXd N1, N2, N3, N4;

  // The Pi sub matrix that needs to be repeated
  MatrixXd Pi_sub = MatrixXd::Zero(3, pi_size);
  Pi_sub.block(0, 0, 3, 3) = MatrixXd::Identity(3, 3);

  // Joint covariance
  MatrixXd Qj = MatrixXd::Zero(num_joints_, num_joints_);
  for (int i = 0; i < num_joints_; ++i) {
    Qj(i, i) = pow(joints_noise_std_(i), 2);
  }

  if (contacts_->at(0)) {
    // Concatenating to the observation vector y
    y1 << collision_pts_left.col(0), 0, 1, VectorXd::Zero(num_contacts_);
    y1(5) = -1;
    VectorXd y_tmp(y.size() + y1.size());
    y_tmp << y, y1;
    y = y_tmp;

    // Concatenating to the vector b
    b1 << VectorXd::Zero(3), 0, 1, VectorXd::Zero(num_contacts_);
    b1(5) = -1;
    VectorXd b_tmp(b.size() + b1.size());
    b_tmp << b, b1;
    b = b_tmp;

    // H matrix concatenation
    H1 = MatrixXd::Zero(3, h_size);
    H1.block(0, 6, 3, 3) = -MatrixXd::Identity(3, 3);
    H1.block(0, 9, 3, 3) = MatrixXd::Identity(3, 3);
    MatrixXd H_tmp(H.rows() + H1.rows(), h_size);
    H_tmp << H, H1;
    H = H_tmp;

    // N matrix diagonal concatenation
    MatrixXd J = ComputeToeLeftJacobianWrtIMU(q, local_collision_pt1_);
    N1 = R_predicted * J * Qj * J.transpose() * R_predicted.transpose() +
         N_prior_;
    MatrixXd N_tmp = MatrixXd::Zero(N.rows() + N1.rows(), N.cols() + N1.cols());
    N_tmp.block(0, 0, N.rows(), N.cols()) = N;
    N_tmp.block(N.rows(), N.cols(), N1.rows(), N1.cols()) = N1;
    N = N_tmp;

    // Pi matrix concatenation
    MatrixXd Pi_tmp =
        MatrixXd::Zero(Pi.rows() + Pi_sub.rows(), Pi.cols() + Pi_sub.cols());
    Pi_tmp.block(0, 0, Pi.rows(), Pi.cols()) = Pi;
    Pi_tmp.block(Pi.rows(), Pi.cols(), Pi_sub.rows(), Pi_sub.cols()) = Pi_sub;
    Pi = Pi_tmp;

    // Full state matrix concatenation
    MatrixXd X_full_tmp = MatrixXd::Zero(X_full.rows() + X_predicted.rows(),
                                         X_full.cols() + X_predicted.cols());
    X_full_tmp.block(0, 0, X_full.rows(), X_full.cols()) = X_full;
    X_full_tmp.block(X_full.rows(), X_full.cols(), X_predicted.rows(),
                     X_predicted.cols()) = X_predicted;
    X_full = X_full_tmp;
  }

  if (contacts_->at(1)) {
    // Concatenating to the observation vector y
    y2 << collision_pts_left.col(1), 0, 1, VectorXd::Zero(num_contacts_);
    y2(6) = -1;
    VectorXd y_tmp(y.size() + y2.size());
    y_tmp << y, y2;
    y = y_tmp;

    // Concatenating to the vector b
    b2 << VectorXd::Zero(3), 0, 1, VectorXd::Zero(num_contacts_);
    b2(6) = -1;
    VectorXd b_tmp(b.size() + b2.size());
    b_tmp << b, b2;
    b = b_tmp;

    // H matrix concatenation
    H2 = MatrixXd::Zero(3, h_size);
    H2.block(0, 6, 3, 3) = -MatrixXd::Identity(3, 3);
    H2.block(0, 12, 3, 3) = MatrixXd::Identity(3, 3);
    MatrixXd H_tmp(H.rows() + H2.rows(), h_size);
    H_tmp << H, H2;
    H = H_tmp;

    // N matrix diagonal concatenation
    MatrixXd J = ComputeToeLeftJacobianWrtIMU(q, local_collision_pt2_);
    N2 = R_predicted * J * Qj * J.transpose() * R_predicted.transpose() +
         N_prior_;
    MatrixXd N_tmp = MatrixXd::Zero(N.rows() + N2.rows(), N.cols() + N2.cols());
    N_tmp.block(0, 0, N.rows(), N.cols()) = N;
    N_tmp.block(N.rows(), N.cols(), N2.rows(), N2.cols()) = N2;
    N = N_tmp;

    // Pi matrix concatenation
    MatrixXd Pi_tmp =
        MatrixXd::Zero(Pi.rows() + Pi_sub.rows(), Pi.cols() + Pi_sub.cols());
    Pi_tmp.block(0, 0, Pi.rows(), Pi.cols()) = Pi;
    Pi_tmp.block(Pi.rows(), Pi.cols(), Pi_sub.rows(), Pi_sub.cols()) = Pi_sub;
    Pi = Pi_tmp;

    // Full state matrix concatenation
    MatrixXd X_full_tmp = MatrixXd::Zero(X_full.rows() + X_predicted.rows(),
                                         X_full.cols() + X_predicted.cols());
    X_full_tmp.block(0, 0, X_full.rows(), X_full.cols()) = X_full;
    X_full_tmp.block(X_full.rows(), X_full.cols(), X_predicted.rows(),
                     X_predicted.cols()) = X_predicted;
    X_full = X_full_tmp;
  }

  if (contacts_->at(2)) {
    // Concatenating to the observation vector y
    y3 << collision_pts_right.col(0), 0, 1, VectorXd::Zero(num_contacts_);
    y3(7) = -1;
    VectorXd y_tmp(y.size() + y3.size());
    y_tmp << y, y3;
    y = y_tmp;

    // Concatenating to the vector b
    b3 << VectorXd::Zero(3), 0, 1, VectorXd::Zero(num_contacts_);
    b3(7) = -1;
    VectorXd b_tmp(b.size() + b3.size());
    b_tmp << b, b3;
    b = b_tmp;

    // H matrix concatenation
    H3 = MatrixXd::Zero(3, h_size);
    H3.block(0, 6, 3, 3) = -MatrixXd::Identity(3, 3);
    H3.block(0, 15, 3, 3) = MatrixXd::Identity(3, 3);
    MatrixXd H_tmp(H.rows() + H3.rows(), h_size);
    H_tmp << H, H3;
    H = H_tmp;

    // N matrix diagonal concatenation
    MatrixXd J = ComputeToeRightJacobianWrtIMU(q, local_collision_pt1_);
    N3 = R_predicted * J * Qj * J.transpose() * R_predicted.transpose() +
         N_prior_;
    MatrixXd N_tmp = MatrixXd::Zero(N.rows() + N3.rows(), N.cols() + N3.cols());
    N_tmp.block(0, 0, N.rows(), N.cols()) = N;
    N_tmp.block(N.rows(), N.cols(), N3.rows(), N3.cols()) = N3;
    N = N_tmp;

    // Pi matrix concatenation
    MatrixXd Pi_tmp =
        MatrixXd::Zero(Pi.rows() + Pi_sub.rows(), Pi.cols() + Pi_sub.cols());
    Pi_tmp.block(0, 0, Pi.rows(), Pi.cols()) = Pi;
    Pi_tmp.block(Pi.rows(), Pi.cols(), Pi_sub.rows(), Pi_sub.cols()) = Pi_sub;
    Pi = Pi_tmp;

    // Full state matrix concatenation
    MatrixXd X_full_tmp = MatrixXd::Zero(X_full.rows() + X_predicted.rows(),
                                         X_full.cols() + X_predicted.cols());
    X_full_tmp.block(0, 0, X_full.rows(), X_full.cols()) = X_full;
    X_full_tmp.block(X_full.rows(), X_full.cols(), X_predicted.rows(),
                     X_predicted.cols()) = X_predicted;
    X_full = X_full_tmp;
  }

  if (contacts_->at(3)) {
    // Concatenating to the observation vector y
    y4 << collision_pts_right.col(1), 0, 1, VectorXd::Zero(num_contacts_);
    y4(8) = -1;
    VectorXd y_tmp(y.size() + y4.size());
    y_tmp << y, y4;
    y = y_tmp;

    // Concatenating to the vector b
    b4 << VectorXd::Zero(3), 0, 1, VectorXd::Zero(num_contacts_);
    b4(8) = -1;
    VectorXd b_tmp(b.size() + b4.size());
    b_tmp << b, b4;
    b = b_tmp;

    // H matrix concatenation
    H4 = MatrixXd::Zero(3, h_size);
    H4.block(0, 6, 3, 3) = -MatrixXd::Identity(3, 3);
    H4.block(0, 18, 3, 3) = MatrixXd::Identity(3, 3);
    MatrixXd H_tmp(H.rows() + H4.rows(), h_size);
    H_tmp << H, H4;
    H = H_tmp;

    // N matrix diagonal concatenation
    MatrixXd J = ComputeToeRightJacobianWrtIMU(q, local_collision_pt2_);
    N4 = R_predicted * J * Qj * J.transpose() * R_predicted.transpose() +
         N_prior_;
    MatrixXd N_tmp = MatrixXd::Zero(N.rows() + N4.rows(), N.cols() + N4.cols());
    N_tmp.block(0, 0, N.rows(), N.cols()) = N;
    N_tmp.block(N.rows(), N.cols(), N4.rows(), N4.cols()) = N4;
    N = N_tmp;

    // Pi matrix concatenation
    MatrixXd Pi_tmp =
        MatrixXd::Zero(Pi.rows() + Pi_sub.rows(), Pi.cols() + Pi_sub.cols());
    Pi_tmp.block(0, 0, Pi.rows(), Pi.cols()) = Pi;
    Pi_tmp.block(Pi.rows(), Pi.cols(), Pi_sub.rows(), Pi_sub.cols()) = Pi_sub;
    Pi = Pi_tmp;

    // Full state matrix concatenation
    MatrixXd X_full_tmp = MatrixXd::Zero(X_full.rows() + X_predicted.rows(),
                                         X_full.cols() + X_predicted.cols());
    X_full_tmp.block(0, 0, X_full.rows(), X_full.cols()) = X_full;
    X_full_tmp.block(X_full.rows(), X_full.cols(), X_predicted.rows(),
                     X_predicted.cols()) = X_predicted;
    X_full = X_full_tmp;
  }

  MatrixXd P = ComputeP(ekf_p_predicted);

  // Computing S and K for the update step
  S = H * P * H.transpose() + N;
  K = P * H.transpose() * (S.inverse());
  delta = K * Pi * (X_full * y - b);
}

MatrixXd CassieRbtStateEstimator::UpdateX(MatrixXd X_predicted,
                                          VectorXd delta) const {
  DRAKE_ASSERT(delta.size() == num_states_total_);

  // Vector length to represent the state in the lie algebra
  int lie_dim = 3 * (3 + num_contacts_);
  MatrixXd dX = ComputeLieExponential(delta.head(lie_dim));
  return X_predicted * dX;
}

VectorXd CassieRbtStateEstimator::UpdateBias(VectorXd bias_predicted,
                                             VectorXd delta) const {
  DRAKE_ASSERT(delta.size() == num_states_total_);

  return bias_predicted + delta.tail(bias_predicted.size());
}

MatrixXd CassieRbtStateEstimator::UpdateP(VectorXd ekf_p_predicted, MatrixXd H,
                                          MatrixXd K, MatrixXd N) const {
  DRAKE_ASSERT(ekf_p_predicted.size() == num_states_total_ * num_states_total_);
  DRAKE_ASSERT(K.rows() == num_states_total_);
  DRAKE_ASSERT(K.cols() == H.rows());
  DRAKE_ASSERT(K.cols() == N.rows());

  MatrixXd P = ComputeP(ekf_p_predicted);

  // Identity matrix of size P
  MatrixXd I = MatrixXd::Identity(P.rows(), P.cols());

  return (I - K * H) * P * (I - K * H).transpose() + K * N * K.transpose();
}

void CassieRbtStateEstimator::set_contacts(vector<int> contacts) {
  for (uint i = 0; i < contacts.size(); ++i) {
    contacts_->at(i) = contacts.at(i);
  }
}

void CassieRbtStateEstimator::set_contacts(int left_contact1, int left_contact2,
                                           int right_contact1,
                                           int right_contact2) {
  contacts_->at(0) = left_contact1;
  contacts_->at(1) = left_contact2;
  contacts_->at(2) = right_contact1;
  contacts_->at(3) = right_contact2;
}

void CassieRbtStateEstimator::AssignNonFloatingBaseToOutputVector(
    OutputVector<double>* output, const cassie_out_t& cassie_out) const {
  // Copy the robot state excluding floating base
  // TODO(yuming): check what cassie_out.leftLeg.footJoint.position is.
  // Similarly, the other leg and the velocity of these joints.
  output->SetPositionAtIndex(positionIndexMap_.at("hip_roll_left"),
                             cassie_out.leftLeg.hipRollDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("hip_yaw_left"),
                             cassie_out.leftLeg.hipYawDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("hip_pitch_left"),
                             cassie_out.leftLeg.hipPitchDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("knee_left"),
                             cassie_out.leftLeg.kneeDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("toe_left"),
                             cassie_out.leftLeg.footDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("knee_joint_left"),
                             cassie_out.leftLeg.shinJoint.position);
  output->SetPositionAtIndex(positionIndexMap_.at("ankle_joint_left"),
                             cassie_out.leftLeg.tarsusJoint.position);
  output->SetPositionAtIndex(positionIndexMap_.at("ankle_spring_joint_left"),
                             0.0);

  output->SetPositionAtIndex(positionIndexMap_.at("hip_roll_right"),
                             cassie_out.rightLeg.hipRollDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("hip_yaw_right"),
                             cassie_out.rightLeg.hipYawDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("hip_pitch_right"),
                             cassie_out.rightLeg.hipPitchDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("knee_right"),
                             cassie_out.rightLeg.kneeDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("toe_right"),
                             cassie_out.rightLeg.footDrive.position);
  output->SetPositionAtIndex(positionIndexMap_.at("knee_joint_right"),
                             cassie_out.rightLeg.shinJoint.position);
  output->SetPositionAtIndex(positionIndexMap_.at("ankle_joint_right"),
                             cassie_out.rightLeg.tarsusJoint.position);
  output->SetPositionAtIndex(positionIndexMap_.at("ankle_spring_joint_right"),
                             0.0);

  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_roll_leftdot"),
                             cassie_out.leftLeg.hipRollDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_yaw_leftdot"),
                             cassie_out.leftLeg.hipYawDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_pitch_leftdot"),
                             cassie_out.leftLeg.hipPitchDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("knee_leftdot"),
                             cassie_out.leftLeg.kneeDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("toe_leftdot"),
                             cassie_out.leftLeg.footDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("knee_joint_leftdot"),
                             cassie_out.leftLeg.shinJoint.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("ankle_joint_leftdot"),
                             cassie_out.leftLeg.tarsusJoint.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("ankle_spring_joint_leftdot"),
                             0.0);

  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_roll_rightdot"),
                             cassie_out.rightLeg.hipRollDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_yaw_rightdot"),
                             cassie_out.rightLeg.hipYawDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("hip_pitch_rightdot"),
                             cassie_out.rightLeg.hipPitchDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("knee_rightdot"),
                             cassie_out.rightLeg.kneeDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("toe_rightdot"),
                             cassie_out.rightLeg.footDrive.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("knee_joint_rightdot"),
                             cassie_out.rightLeg.shinJoint.velocity);
  output->SetVelocityAtIndex(velocityIndexMap_.at("ankle_joint_rightdot"),
                             cassie_out.rightLeg.tarsusJoint.velocity);
  output->SetVelocityAtIndex(
      velocityIndexMap_.at("ankle_spring_joint_rightdot"), 0.0);

  // Copy actuators
  // We don't need to copy the value of motor torque, since we are not
  // passing
  // it to the controller. However, we can pass it if we are doing torque
  // control, for example.
  /*output->SetEffortAtIndex(actuatorIndexMap_.at("hip_roll_left_motor"),
                           cassie_out.leftLeg.hipRollDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("hip_yaw_left_motor"),
                           cassie_out.leftLeg.hipYawDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("hip_pitch_left_motor"),
                           cassie_out.leftLeg.hipPitchDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("knee_left_motor"),
                           cassie_out.leftLeg.kneeDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("toe_left_motor"),
                           cassie_out.leftLeg.footDrive.torque);

  output->SetEffortAtIndex(actuatorIndexMap_.at("hip_roll_right_motor"),
                           cassie_out.rightLeg.hipRollDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("hip_yaw_right_motor"),
                           cassie_out.rightLeg.hipYawDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("hip_pitch_right_motor"),
                           cassie_out.rightLeg.hipPitchDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("knee_right_motor"),
                           cassie_out.rightLeg.kneeDrive.torque);
  output->SetEffortAtIndex(actuatorIndexMap_.at("toe_right_motor"),
                           cassie_out.rightLeg.footDrive.torque);*/
}

EventStatus CassieRbtStateEstimator::Update(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Testing
  const auto& cassie_out =
      this->EvalAbstractInput(context, 0)->get_value<cassie_out_t>();
  cout << "In per-step update: lcm_time = "
       << cassie_out.pelvis.targetPc.taskExecutionTime << endl;
  cout << "In per-step update: context_time = " << context.get_time() << endl;

  // Get current time and previous time
  double current_time = context.get_time();
  double prev_t = discrete_state->get_mutable_vector(time_idx_).get_value()(0);
  VectorXd ekf_x =
      discrete_state->get_mutable_vector(ekf_x_idx_).get_mutable_value();
  VectorXd ekf_bias =
      discrete_state->get_mutable_vector(ekf_bias_idx_).get_mutable_value();
  VectorXd ekf_p =
      discrete_state->get_mutable_vector(ekf_p_idx_).get_mutable_value();

  // Reading the IMU values
  VectorXd angular_velocity(3), linear_acceleration(3), u(6);
  for (int i = 0; i < 3; ++i) {
    angular_velocity(i) = cassie_out.pelvis.vectorNav.angularVelocity[i];
    linear_acceleration(i) = cassie_out.pelvis.vectorNav.linearAcceleration[i];
  }
  u << angular_velocity, linear_acceleration;

  // Testing
  // current_time = cassie_out.pelvis.targetPc.taskExecutionTime;

  if (current_time > prev_t) {
    double dt = current_time - prev_t;

    // Updating the time state with the current time.
    discrete_state->get_mutable_vector(time_idx_).get_mutable_value()
        << current_time;

    // Testing
    cout << "In per-step update: updated state_time = "
         << discrete_state->get_mutable_vector(time_idx_).get_mutable_value()
         << endl;

    VectorXd ekf_x_predicted, ekf_bias_predicted, ekf_p_predicted;
    MatrixXd X_predicted, P_predicted;
    VectorXd ekf_x_updated, ekf_bias_updated, ekf_p_updated;
    MatrixXd X_updated, P_updated;

    VectorXd y, b, delta;
    MatrixXd H, N, Pi, X_full, S, K;

    // Perform State Estimation (in several steps)

    // Step 1 - Solve for the unknown joint angle
    // TODO(yminchen): Is there a way to avoid copying state two times?
    // Currently, we copy in Update() and CopyStateOut().
    const auto& cassie_out =
        this->EvalAbstractInput(context, 0)->get_value<cassie_out_t>();
    OutputVector<double> output(tree_.get_num_positions(),
                                tree_.get_num_velocities(),
                                tree_.get_num_actuators());
    AssignNonFloatingBaseToOutputVector(&output, cassie_out);

    // Current configuration
    VectorXd q = output.GetMutablePositions();
    VectorXd q_zero = tree_.getZeroConfiguration();

    // The floating base values are not set and are thus nan.
    // Replacing the nan's with the tree's zero configuration.
    // RBT's zero configuration is used to provide a generic solution that
    // handles quaternions as well (Unit normal condition).
    for (int i = 0; i < q.size(); ++i) {
      if (std::isnan(q(i))) {
        q(i) = q_zero(i);
      }
    }

    double left_heel_spring = 0;
    double right_heel_spring = 0;
    solveFourbarLinkage(q, left_heel_spring, right_heel_spring);

    // TODO(yminchen):
    // You can test the contact force estimator here using fixed based.
    // You can implement step 3 independently of the EKF.

    // The concern when moving to floating based simulation:
    // The simulation update rate is about 30-60 Hz.

    // Step 2 - EKF (Prediction step)
    // Prediction step

    ekf_x_predicted = ComputeEkfX(PredictX(ekf_x, ekf_bias, u, q, dt));
    ekf_bias_predicted = PredictBias(ekf_bias, dt);
    ekf_p_predicted = ComputeEkfP(PredictP(ekf_x, ekf_p, q, dt));
    X_predicted = ComputeX(ekf_x_predicted);
    P_predicted = ComputeP(ekf_p_predicted);

    // Updating

    // Step 3 - Estimate which foot/feet are in contact with the ground
    if (std::abs(left_heel_spring) > left_spring_contact_threshold_) {
      // Left leg in contact with the ground
      // Updating the contact indicator (Both colliders on the left leg are
      // assumed to be in contact.
      contacts_->at(0) = 1;
      contacts_->at(1) = 1;
      // std::cout << "LEFT ";
    } else {
      // Not in contact
      contacts_->at(0) = 0;
      contacts_->at(1) = 0;
    }
    if (std::abs(right_heel_spring) > right_spring_contact_threshold_) {
      // Right leg in contact with the ground
      // Updating the contact indicator (Both colliders on the right leg are
      // assumed to be in contact.
      contacts_->at(2) = 1;
      contacts_->at(3) = 1;
      // std::cout << "RIGHT ";
    } else {
      // Not in contact
      contacts_->at(2) = 0;
      contacts_->at(3) = 0;
    }

    // Step 4 - EKF (measurement step)

    if (ComputeNumContacts()) {
      // Computing update step parameters
      ComputeUpdateParams(ekf_x_predicted, ekf_p_predicted, q, y, b, H, N, Pi,
                          X_full, S, K, delta);
      X_updated = UpdateX(ComputeX(ekf_x_predicted), delta);
      ekf_bias_updated = UpdateBias(ekf_bias_predicted, delta);
      P_updated = UpdateP(ekf_p_predicted, H, K, N);

      ekf_x_updated = ComputeEkfX(X_updated);
      ekf_p_updated = ComputeEkfP(P_updated);

    } else {
      // If there are no contacts, there is no update step.
      ekf_x_updated = ekf_x_predicted;
      ekf_bias_updated = ekf_bias_predicted;
      ekf_p_updated = ekf_p_predicted;
    }

    std::cout << "---------------" << std::endl;
    //std::cout << ExtractFloatingBasePositions(ekf_x_updated) << std::endl;
    //std::cout << X_updated << std::endl;
    std::cout << q.transpose() << std::endl;
    //std::cout << u.transpose() << std::endl;
    std::cout << "---------------" << std::endl;

    // Step 5 - Assign values to states
    // Below is how you should assign the state at the end of this Update
    // discrete_state->get_mutable_vector(ekf_x_idx_).get_mutable_value() =
    // ...;
    // discrete_state->get_mutable_vector(time_idx_).get_mutable_value() =
    // ...;

    // discrete_state->get_mutable_vector(ekf_x_idx_).get_mutable_value() =
    //    ekf_x_updated;
    // discrete_state->get_mutable_vector(ekf_bias_idx_).get_mutable_value() =
    //    ekf_bias_updated;
    // discrete_state->get_mutable_vector(ekf_p_idx_).get_mutable_value() =
    //    ekf_p_updated;

    // You can convert a rotational matrix to quaternion using Eigen
    // https://stackoverflow.com/questions/21761909/eigen-convert-matrix3d-rotation-to-quaternion
    // http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

    // Then convert Eigen::Quaterion to (w,x,y,z) by drake's
    // QuaternionToVectorWxyz()
    // https://drake.mit.edu/doxygen_cxx/namespacedrake_1_1multibody.html#ad1b559878de179a7e363846fa67f58c0

    // Question: Do we need to filter the gyro value?
    // We will get the bias (parameter) from EKF

    // discrete_state->get_mutable_vector(time_idx_).get_mutable_value() <<

    // discrete_state->get_mutable_vector(time_idx_).get_mutable_value()(0) =
    //    current_time;
  }

  return EventStatus::Succeeded();
}

/// Workhorse state estimation function. Given a `cassie_out_t`, compute the
/// esitmated state as an OutputVector
/// Since it needs to map from a struct to a vector, and no assumptions on
/// the
/// ordering of the vector are made, utilizies index maps to make this
/// mapping.
void CassieRbtStateEstimator::CopyStateOut(const Context<double>& context,
                                           OutputVector<double>* output) const {
  const auto& cassie_out =
      this->EvalAbstractInput(context, 0)->get_value<cassie_out_t>();

  // It's necessary for initialization. Might be a better way to initialize?
  auto data =
      output->get_mutable_data();  // This doesn't affect timestamp value
  data = Eigen::VectorXd::Zero(data.size());

  // Assign the values
  // Copy the robot state excluding floating base
  AssignNonFloatingBaseToOutputVector(output, cassie_out);

  // Floating base coordinates
  if (is_floating_base_) {
    // Assign the values
    auto state_est = context.get_discrete_state(state_idx_).get_value();

    // TODO(yminchen): The name of the joitn name need to be change when we
    // move
    // to MBP
    output->SetPositionAtIndex(positionIndexMap_.at("base_x"), state_est(0));
    output->SetPositionAtIndex(positionIndexMap_.at("base_y"), state_est(1));
    output->SetPositionAtIndex(positionIndexMap_.at("base_z"), state_est(2));
    output->SetPositionAtIndex(positionIndexMap_.at("base_qw"), state_est(3));
    output->SetPositionAtIndex(positionIndexMap_.at("base_qx"), state_est(4));
    output->SetPositionAtIndex(positionIndexMap_.at("base_qy"), state_est(5));
    output->SetPositionAtIndex(positionIndexMap_.at("base_qz"), state_est(6));

    output->SetVelocityAtIndex(velocityIndexMap_.at("base_wx"), state_est(7));
    output->SetVelocityAtIndex(velocityIndexMap_.at("base_wy"), state_est(8));
    output->SetVelocityAtIndex(velocityIndexMap_.at("base_wz"), state_est(9));
    output->SetVelocityAtIndex(velocityIndexMap_.at("base_vx"), state_est(10));
    output->SetVelocityAtIndex(velocityIndexMap_.at("base_vy"), state_est(11));
    output->SetVelocityAtIndex(velocityIndexMap_.at("base_vz"), state_est(12));
  }  //  end if(is_floating_base)

  // Testing
  auto state_time = context.get_discrete_state(time_idx_).get_value();
  // cout << "  In copyStateOut: lcm_time = "
  //     << cassie_out.pelvis.targetPc.taskExecutionTime << endl;
  // cout << "  In copyStateOut: state_time = " << state_time << endl;
  // cout << "  In copyStateOut: context_time = " << context.get_time() << endl;

  // Testing
  // cout << endl << "****bodies****" << endl;
  // for (int i = 0; i < tree_.get_num_bodies(); i++)
  //   cout << tree_.getBodyOrFrameName(i) << endl;
  // cout << endl << "****actuators****" << endl;
  // for (int i = 0; i < tree_.get_num_actuators(); i++)
  //   cout << tree_.actuators[i].name_ << endl;
  // cout << endl << "****positions****" << endl;
  // for (int i = 0; i < tree_.get_num_positions(); i++)
  //   cout << tree_.get_position_name(i) << endl;
  // cout << endl << "****velocities****" << endl;
  // for (int i = 0; i < tree_.get_num_velocities(); i++)
  // cout << tree_.get_velocity_name(i) << endl;
}

}  // namespace systems
}  // namespace dairlib
