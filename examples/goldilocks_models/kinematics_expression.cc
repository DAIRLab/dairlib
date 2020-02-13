#include "examples/goldilocks_models/kinematics_expression.h"


namespace dairlib {
namespace goldilocks_models {

template <typename T>
KinematicsExpression<T>::KinematicsExpression(int n_s, int n_feature,
    int robot_option) {
  n_feature_ = n_feature;
  n_s_ = n_s;
  robot_option_ = robot_option;
}

template <typename T>
KinematicsExpression<T>::KinematicsExpression(int n_s, int n_feature,
    const MultibodyPlant<T>* plant, int robot_option) {
  n_feature_ = n_feature;
  n_s_ = n_s;
  plant_ = plant;
  context_ = plant->CreateDefaultContext();
  robot_option_ = robot_option;

  // robot 1 param
  pt_front_contact_ << -0.0457, 0.112, 0;
  pt_rear_contact_ << 0.088, 0, 0;
  mid_disp_ = (pt_front_contact_ + pt_rear_contact_) / 2;
}

template <typename T>
int KinematicsExpression<T>::getDimFeature() {
  return n_feature_;
}

template <typename T>
template <typename U, typename V>
VectorX<T> KinematicsExpression<T>::getExpression(
  const U & theta, const V & q) const {
  // DRAKE_DEMAND(n_s_ * n_feature_ == theta.size());  // check theta size
  // DRAKE_DEMAND(n_feature_ == getFeature(q).size());  // check feature size
  VectorX<T> expression(n_s_);
  for (int i = 0; i < n_s_ ; i++)
    expression(i) =
      theta.segment(i * n_feature_, n_feature_).dot(getFeature(q));
  return expression;
}
template <typename T>
template <typename U, typename V>
VectorX<T> KinematicsExpression<T>::getExpressionDot(
  const U & theta, const V & q, const V & v) const {
  // DRAKE_DEMAND(n_s_ * n_feature_ == theta.size());  // check theta size
  // DRAKE_DEMAND(n_feature_ == getFeatureDot(q).size());  // check feature size
  VectorX<T> expression(n_s_);
  for (int i = 0; i < n_s_ ; i++)
    expression(i) =
      theta.segment(i * n_feature_, n_feature_).dot(getFeatureDot(q, v));
  return expression;
}

template <typename T>
template <typename U>
VectorX<U> KinematicsExpression<T>::getFeature(const VectorX<U> & q) const {

  // Implement your choice of features below
  // Be careful that the dimension should match with n_feature_
  // TODO(yminchen): find a way to avoid hard coding the features here

  if (robot_option_ == 0) {
    //////////// Version 1: for kinematics_expression_test ///////////////////////
    // VectorX<U> feature(5);
    // feature << q(0),
    //            q(1)*q(1)*q(1),
    //            q(0) * q(1),
    //            cos(q(0)),
    //            sqrt(q(1));

    //////////// Version 2: testing //////////////////////////////////////////////
    // VectorX<U> feature(1);
    // feature << q(1);

    //////////// Version 3: testing //////////////////////////////////////////////
    // VectorX<U> feature(2);
    // feature << q(1), q(1+q.size()/2);

    //////////// Version 4: testing MBP //////////////////////////////////////////
    // // If you use plant functions, then it's required that T = U?
    // auto context = plant_->CreateDefaultContext();
    // plant_->SetPositions(context.get(), q);

    // const auto & right_lower_leg = plant_->GetBodyByName("right_lower_leg_mass");
    // const auto & right_lower_leg_pose = plant_->EvalBodyPoseInWorld(
    //                                       *context, right_lower_leg);

    // Vector3d disp(0, 0, -0.5);
    // const VectorX<U> right_lower_leg_Pos = right_lower_leg_pose.translation();
    // const MatrixX<U> right_lower_leg_Rotmat = right_lower_leg_pose.linear();
    // VectorX<U> feature = right_lower_leg_Pos + right_lower_leg_Rotmat * disp;

    //////////// Version 5: testing r_stance_foot_to_CoM /////////////////////////
    /*// If you use plant functions, then it's required that T = U?
    // Get CoM position and stance foot position in autoDiff
    auto context = plant_->CreateDefaultContext();
    plant_->SetPositions(context.get(), q);

    // CoM
    const auto & torso = plant_->GetBodyByName("torso_mass");
    const auto & torso_pose = plant_->EvalBodyPoseInWorld(*context, torso);
    VectorX<U> CoM = 0.5 * torso_pose.translation();

    for (int i = 0; i < 4; i++) {
      const auto & body = plant_->GetBodyByName(leg_link_names_[i]);
      const auto & body_pose = plant_->EvalBodyPoseInWorld(*context, body);

      CoM += (body_pose.translation() + body_pose.linear() * mass_disp_) / 8.0;
    }

    // Stance foot position (left foot)
    const auto & left_lower_leg = plant_->GetBodyByName("left_lower_leg_mass");
    const auto & left_lower_leg_pose = plant_->EvalBodyPoseInWorld(
                                          *context, left_lower_leg);
    const VectorX<U> left_lower_leg_Pos = left_lower_leg_pose.translation();
    const MatrixX<U> left_lower_leg_Rotmat = left_lower_leg_pose.linear();
    VectorX<U> left_foot_pos = left_lower_leg_Pos + left_lower_leg_Rotmat * foot_disp_;

    VectorX<U> foot_to_CoM = CoM - left_foot_pos;
    VectorX<U> feature(1);
    feature << foot_to_CoM.norm();*/

    //////////// Version 6: SLIP /////////////////////////////////////////////////
    // // If you use plant functions, then it's required that T = U? I guess so.
    // // Get CoM position and stance foot position in autoDiff
    // auto context = plant_->CreateDefaultContext();
    // plant_->SetPositions(context.get(), q);

    // // CoM
    // const auto & torso = plant_->GetBodyByName("torso_mass");
    // const auto & torso_pose = plant_->EvalBodyPoseInWorld(*context, torso);
    // VectorX<U> CoM = 0.5 * torso_pose.translation();

    // for (int i = 0; i < 4; i++) {
    //   const auto & body = plant_->GetBodyByName(leg_link_names_[i]);
    //   const auto & body_pose = plant_->EvalBodyPoseInWorld(*context, body);

    //   CoM += (body_pose.translation() + body_pose.linear() * mass_disp_) / 8.0;
    // }

    // // Stance foot position (left foot)
    // const auto & left_lower_leg = plant_->GetBodyByName("left_lower_leg_mass");
    // const auto & left_lower_leg_pose = plant_->EvalBodyPoseInWorld(
    //                                       *context, left_lower_leg);
    // const VectorX<U> left_lower_leg_Pos = left_lower_leg_pose.translation();
    // const MatrixX<U> left_lower_leg_Rotmat = left_lower_leg_pose.linear();
    // VectorX<U> left_foot_pos = left_lower_leg_Pos + left_lower_leg_Rotmat * foot_disp_;

    // VectorX<U> foot_to_CoM = CoM - left_foot_pos;

    // VectorX<U> feature(2);
    // feature << foot_to_CoM.norm(),
    //            atan(foot_to_CoM(0)/foot_to_CoM(2));

    //////////// Version 7: Quadratic combinations /////////////////////////////
    // elements:
    // q(0),
    // q(1),
    // sin(q(2)), cos(q(2))
    // sin(q(3)), cos(q(3))
    // sin(q(4)), cos(q(4))
    // sin(q(5)), cos(q(5))
    // sin(q(6)), cos(q(6))

    /*VectorX<U> feature(68);  // 1 + 12 + 10C2 = 1 + 12 + 55 = 68
    feature <<1,
              q(0),
              q(1),
              sin(q(2)),
              cos(q(2)),
              sin(q(3)),
              cos(q(3)),
              sin(q(4)),
              cos(q(4)),
              sin(q(5)),
              cos(q(5)),
              sin(q(6)),
              cos(q(6)),  // linear until here, below are quadratic
              // 1
              sin(q(2)) * sin(q(2)),
              cos(q(2)) * sin(q(2)),
              sin(q(3)) * sin(q(2)),
              cos(q(3)) * sin(q(2)),
              sin(q(4)) * sin(q(2)),
              cos(q(4)) * sin(q(2)),
              sin(q(5)) * sin(q(2)),
              cos(q(5)) * sin(q(2)),
              sin(q(6)) * sin(q(2)),
              cos(q(6)) * sin(q(2)),
              // 2
              cos(q(2)) * cos(q(2)),
              sin(q(3)) * cos(q(2)),
              cos(q(3)) * cos(q(2)),
              sin(q(4)) * cos(q(2)),
              cos(q(4)) * cos(q(2)),
              sin(q(5)) * cos(q(2)),
              cos(q(5)) * cos(q(2)),
              sin(q(6)) * cos(q(2)),
              cos(q(6)) * cos(q(2)),
              // 3
              sin(q(3)) * sin(q(3)),
              cos(q(3)) * sin(q(3)),
              sin(q(4)) * sin(q(3)),
              cos(q(4)) * sin(q(3)),
              sin(q(5)) * sin(q(3)),
              cos(q(5)) * sin(q(3)),
              sin(q(6)) * sin(q(3)),
              cos(q(6)) * sin(q(3)),
              // 4
              cos(q(3)) * cos(q(3)),
              sin(q(4)) * cos(q(3)),
              cos(q(4)) * cos(q(3)),
              sin(q(5)) * cos(q(3)),
              cos(q(5)) * cos(q(3)),
              sin(q(6)) * cos(q(3)),
              cos(q(6)) * cos(q(3)),
              // 5
              sin(q(4)) * sin(q(4)),
              cos(q(4)) * sin(q(4)),
              sin(q(5)) * sin(q(4)),
              cos(q(5)) * sin(q(4)),
              sin(q(6)) * sin(q(4)),
              cos(q(6)) * sin(q(4)),
              // 6
              cos(q(4)) * cos(q(4)),
              sin(q(5)) * cos(q(4)),
              cos(q(5)) * cos(q(4)),
              sin(q(6)) * cos(q(4)),
              cos(q(6)) * cos(q(4)),
              // 7
              sin(q(5)) * sin(q(5)),
              cos(q(5)) * sin(q(5)),
              sin(q(6)) * sin(q(5)),
              cos(q(6)) * sin(q(5)),
              // 8
              cos(q(5)) * cos(q(5)),
              sin(q(6)) * cos(q(5)),
              cos(q(6)) * cos(q(5)),
              // 9
              sin(q(6)) * sin(q(6)),
              cos(q(6)) * sin(q(6)),
              // 10
              cos(q(6)) * cos(q(6));*/

    //////////// Version 8: debug B matrix ///////////////////////////////////////
    // VectorX<U> feature(1);
    // feature << q(1);
    //////////// Version 9: debug B matrix ///////////////////////////////////////
    // VectorX<U> feature(1);
    // feature << q(1)*q(1);


    //////////// Version 10: features contain LIPM & swing foot //////////////////
    // Get CoM position and stance foot position in autoDiff
    ///// Way 1: caluclate from MBP /////////// This is computationally heavy
    /*// auto context = plant_->CreateDefaultContext();
    plant_->SetPositions(context_.get(), q);

    // CoM
    const auto & torso = plant_->GetBodyByName("torso_mass");
    const auto & torso_pose = plant_->EvalBodyPoseInWorld(*context_, torso);
    VectorX<U> CoM = 0.5 * torso_pose.translation();
    for (int i = 0; i < 4; i++) {
      const auto & body = plant_->GetBodyByName(leg_link_names_[i]);
      const auto & body_pose = plant_->EvalBodyPoseInWorld(*context_, body);

      CoM += (body_pose.translation() + body_pose.linear() * mass_disp_) / 8.0;
    }

    // Stance foot position (left foot)
    const auto & left_lower_leg = plant_->GetBodyByName("left_lower_leg_mass");
    const auto & left_lower_leg_pose = plant_->EvalBodyPoseInWorld(
                                         *context_, left_lower_leg);
    const VectorX<U> left_lower_leg_Pos = left_lower_leg_pose.translation();
    const MatrixX<U> left_lower_leg_Rotmat = left_lower_leg_pose.linear();
    VectorX<U> left_foot_pos = left_lower_leg_Pos +
                               left_lower_leg_Rotmat * foot_disp_;
    VectorX<U> st_to_CoM = CoM - left_foot_pos;

    VectorX<U> feature_base(2);
    feature_base << st_to_CoM(0),  // CoM_x
                 st_to_CoM(2);  // CoM_z*/

    ///// Way 2: caluclate by hand //////////// Speed increased by 35 times!!!
    // Calculate the CoM by hand instead of by using MBP
    /*VectorX<U> CoM_xz(2);
    CoM_xz << q(0) + (0.3 * sin(q(2))) / 2.0 + (
             - 0.25 * sin(q(2) + q(3)) +
             - 0.5 * sin(q(2) + q(3)) - 0.25 * sin(q(2) + q(3) + q(5)) +
             - 0.25 * sin(q(2) + q(4)) +
             - 0.5 * sin(q(2) + q(4)) - 0.25 * sin(q(2) + q(4) + q(6))) / 8.0,
             q(1) + (0.3 * cos(q(2))) / 2.0 + (
               - 0.25 * cos(q(2) + q(3)) +
               - 0.5 * cos(q(2) + q(3)) - 0.25 * cos(q(2) + q(3) + q(5)) +
               - 0.25 * cos(q(2) + q(4)) +
               - 0.5 * cos(q(2) + q(4)) - 0.25 * cos(q(2) + q(4) + q(6))) / 8.0;

    // Calculate the stance foot by hand instead of by using MBP
    VectorX<U> left_foot_pos_xz(2);
    left_foot_pos_xz <<
                     q(0) - 0.5 * sin(q(2) + q(3)) - 0.5 * sin(q(2) + q(3) + q(5)),
                     q(1) - 0.5 * cos(q(2) + q(3)) - 0.5 * cos(q(2) + q(3) + q(5));
    VectorX<U> st_to_CoM_xz = CoM_xz - left_foot_pos_xz;
    // cout << "st_to_CoM_xz = " << st_to_CoM_xz.transpose() << endl;
    // cout << "st_to_CoM from MBP = " << st_to_CoM(0) << " " << st_to_CoM(2) << endl;

    VectorX<U> feature_base(2);
    feature_base << st_to_CoM_xz;


    // elements:
    // q(0),
    // q(1),
    // sin(q(2)), cos(q(2))
    // sin(q(3)), cos(q(3))
    // sin(q(4)), cos(q(4))
    // sin(q(5)), cos(q(5))
    // sin(q(6)), cos(q(6))
    // feature_base(0), feature_base(1)

    VectorX<U> feature(70);  // 2 + 1 + 12 + 10C2 = 2 + 1 + 12 + 55 = 70
    feature << feature_base,
            1,
            q(0),
            q(1),
            sin(q(2)),
            cos(q(2)),
            sin(q(3)),
            cos(q(3)),
            sin(q(4)),
            cos(q(4)),
            sin(q(5)),
            cos(q(5)),
            sin(q(6)),
            cos(q(6)),  // linear until here, below are quadratic
            // 1
            sin(q(2)) * sin(q(2)),
            cos(q(2)) * sin(q(2)),
            sin(q(3)) * sin(q(2)),
            cos(q(3)) * sin(q(2)),
            sin(q(4)) * sin(q(2)),
            cos(q(4)) * sin(q(2)),
            sin(q(5)) * sin(q(2)),
            cos(q(5)) * sin(q(2)),
            sin(q(6)) * sin(q(2)),
            cos(q(6)) * sin(q(2)),
            // 2
            cos(q(2)) * cos(q(2)),
            sin(q(3)) * cos(q(2)),
            cos(q(3)) * cos(q(2)),
            sin(q(4)) * cos(q(2)),
            cos(q(4)) * cos(q(2)),
            sin(q(5)) * cos(q(2)),
            cos(q(5)) * cos(q(2)),
            sin(q(6)) * cos(q(2)),
            cos(q(6)) * cos(q(2)),
            // 3
            sin(q(3)) * sin(q(3)),
            cos(q(3)) * sin(q(3)),
            sin(q(4)) * sin(q(3)),
            cos(q(4)) * sin(q(3)),
            sin(q(5)) * sin(q(3)),
            cos(q(5)) * sin(q(3)),
            sin(q(6)) * sin(q(3)),
            cos(q(6)) * sin(q(3)),
            // 4
            cos(q(3)) * cos(q(3)),
            sin(q(4)) * cos(q(3)),
            cos(q(4)) * cos(q(3)),
            sin(q(5)) * cos(q(3)),
            cos(q(5)) * cos(q(3)),
            sin(q(6)) * cos(q(3)),
            cos(q(6)) * cos(q(3)),
            // 5
            sin(q(4)) * sin(q(4)),
            cos(q(4)) * sin(q(4)),
            sin(q(5)) * sin(q(4)),
            cos(q(5)) * sin(q(4)),
            sin(q(6)) * sin(q(4)),
            cos(q(6)) * sin(q(4)),
            // 6
            cos(q(4)) * cos(q(4)),
            sin(q(5)) * cos(q(4)),
            cos(q(5)) * cos(q(4)),
            sin(q(6)) * cos(q(4)),
            cos(q(6)) * cos(q(4)),
            // 7
            sin(q(5)) * sin(q(5)),
            cos(q(5)) * sin(q(5)),
            sin(q(6)) * sin(q(5)),
            cos(q(6)) * sin(q(5)),
            // 8
            cos(q(5)) * cos(q(5)),
            sin(q(6)) * cos(q(5)),
            cos(q(6)) * cos(q(5)),
            // 9
            sin(q(6)) * sin(q(6)),
            cos(q(6)) * sin(q(6)),
            // 10
            cos(q(6)) * cos(q(6));*/

    //////////// Version 10: features contain LIPM & swing foot //////////////////
    //// Way 1: Get CoM and foot position from MBP //////
    /*// Get CoM position and stance foot position in autoDiff
    plant_->SetPositions(context_.get(), q);
    // CoM
    const auto & torso = plant_->GetBodyByName("torso_mass");
    const auto & torso_pose = plant_->EvalBodyPoseInWorld(*context_, torso);
    VectorX<U> CoM = 0.5 * torso_pose.translation();
    for (int i = 0; i < 4; i++) {
      const auto & body = plant_->GetBodyByName(leg_link_names_[i]);
      const auto & body_pose = plant_->EvalBodyPoseInWorld(*context_, body);

      CoM += (body_pose.translation() + body_pose.linear() * mass_disp_) / 8.0;
    }
    // Stance foot position (left foot)
    const auto & left_lower_leg = plant_->GetBodyByName("left_lower_leg_mass");
    const auto & left_lower_leg_pose = plant_->EvalBodyPoseInWorld(
                                         *context_, left_lower_leg);
    const VectorX<U> left_lower_leg_Pos = left_lower_leg_pose.translation();
    const MatrixX<U> left_lower_leg_Rotmat = left_lower_leg_pose.linear();
    VectorX<U> left_foot_pos = left_lower_leg_Pos +
                               left_lower_leg_Rotmat * foot_disp_;
    VectorX<U> st_to_CoM = CoM - left_foot_pos;
    // Swing foot position (right foot)
    const auto & right_lower_leg = plant_->GetBodyByName("right_lower_leg_mass");
    const auto & right_lower_leg_pose = plant_->EvalBodyPoseInWorld(
                                          *context_, right_lower_leg);
    const VectorX<U> right_lower_leg_Pos = right_lower_leg_pose.translation();
    const MatrixX<U> right_lower_leg_Rotmat = right_lower_leg_pose.linear();
    VectorX<U> right_foot_pos = right_lower_leg_Pos +
                                right_lower_leg_Rotmat * foot_disp_;
    VectorX<U> CoM_to_sw = right_foot_pos - CoM;
    // cout << "CoM from MBP = " << CoM(0) << " " << CoM(2) << endl;
    // cout << "st_to_CoM from MBP = " << st_to_CoM(0) << " " << st_to_CoM(2) << endl;
    // cout << "CoM_to_sw from MBP = " << CoM_to_sw(0) << " " << CoM_to_sw(2) << endl;*/

    //// Way 2: Get CoM and foot position by hand //////
    // Calculate the CoM by hand instead of by using MBP
    VectorX<U> CoM_xz(2);
    CoM_xz << q(0) + (0.3 * sin(q(2))) / 2.0 + (
             - 0.25 * sin(q(2) + q(3)) +
             - 0.5 * sin(q(2) + q(3)) - 0.25 * sin(q(2) + q(3) + q(5)) +
             - 0.25 * sin(q(2) + q(4)) +
             - 0.5 * sin(q(2) + q(4)) - 0.25 * sin(q(2) + q(4) + q(6))) / 8.0,
             q(1) + (0.3 * cos(q(2))) / 2.0 + (
               - 0.25 * cos(q(2) + q(3)) +
               - 0.5 * cos(q(2) + q(3)) - 0.25 * cos(q(2) + q(3) + q(5)) +
               - 0.25 * cos(q(2) + q(4)) +
               - 0.5 * cos(q(2) + q(4)) - 0.25 * cos(q(2) + q(4) + q(6))) / 8.0;
    // Calculate the stance foot by hand instead of by using MBP
    VectorX<U> left_foot_pos_xz(2);
    left_foot_pos_xz <<
                     q(0) - 0.5 * sin(q(2) + q(3)) - 0.5 * sin(q(2) + q(3) + q(5)),
                     q(1) - 0.5 * cos(q(2) + q(3)) - 0.5 * cos(q(2) + q(3) + q(5));
    VectorX<U> st_to_CoM_xz = CoM_xz - left_foot_pos_xz;
    // Calculate the swing foot by hand instead of by using MBP
    VectorX<U> right_foot_pos_xz(2);
    right_foot_pos_xz <<
                      q(0) - 0.5 * sin(q(2) + q(4)) - 0.5 * sin(q(2) + q(4) + q(6)),
                      q(1) - 0.5 * cos(q(2) + q(4)) - 0.5 * cos(q(2) + q(4) + q(6));
    VectorX<U> CoM_to_sw_xz = right_foot_pos_xz - CoM_xz;
    // cout << "CoM_xz = " << CoM_xz.transpose() << endl;
    // cout << "st_to_CoM_xz = " << st_to_CoM_xz.transpose() << endl;
    // cout << "CoM_to_sw_xz = " << CoM_to_sw_xz.transpose() << endl;

    VectorX<U> feature_base(4);
    feature_base << st_to_CoM_xz, CoM_to_sw_xz;

    // elements:
    // q(0),
    // q(1),
    // sin(q(2)), cos(q(2))
    // sin(q(3)), cos(q(3))
    // sin(q(4)), cos(q(4))
    // sin(q(5)), cos(q(5))
    // sin(q(6)), cos(q(6))
    // feature_base(0), feature_base(1), feature_base(2), feature_base(3)

    /*VectorX<U> feature(72);  // 4 + 1 + 12 + (10C2 + 10) = 4 + 1 + 12 + 55 = 72
    feature << feature_base,
            1,
            q(0),
            q(1),
            sin(q(2)),
            cos(q(2)),
            sin(q(3)),
            cos(q(3)),
            sin(q(4)),
            cos(q(4)),
            sin(q(5)),
            cos(q(5)),
            sin(q(6)),
            cos(q(6)),  // linear until here, below are quadratic
            // 1
            sin(q(2)) * sin(q(2)),
            cos(q(2)) * sin(q(2)),
            sin(q(3)) * sin(q(2)),
            cos(q(3)) * sin(q(2)),
            sin(q(4)) * sin(q(2)),
            cos(q(4)) * sin(q(2)),
            sin(q(5)) * sin(q(2)),
            cos(q(5)) * sin(q(2)),
            sin(q(6)) * sin(q(2)),
            cos(q(6)) * sin(q(2)),
            // 2
            cos(q(2)) * cos(q(2)),
            sin(q(3)) * cos(q(2)),
            cos(q(3)) * cos(q(2)),
            sin(q(4)) * cos(q(2)),
            cos(q(4)) * cos(q(2)),
            sin(q(5)) * cos(q(2)),
            cos(q(5)) * cos(q(2)),
            sin(q(6)) * cos(q(2)),
            cos(q(6)) * cos(q(2)),
            // 3
            sin(q(3)) * sin(q(3)),
            cos(q(3)) * sin(q(3)),
            sin(q(4)) * sin(q(3)),
            cos(q(4)) * sin(q(3)),
            sin(q(5)) * sin(q(3)),
            cos(q(5)) * sin(q(3)),
            sin(q(6)) * sin(q(3)),
            cos(q(6)) * sin(q(3)),
            // 4
            cos(q(3)) * cos(q(3)),
            sin(q(4)) * cos(q(3)),
            cos(q(4)) * cos(q(3)),
            sin(q(5)) * cos(q(3)),
            cos(q(5)) * cos(q(3)),
            sin(q(6)) * cos(q(3)),
            cos(q(6)) * cos(q(3)),
            // 5
            sin(q(4)) * sin(q(4)),
            cos(q(4)) * sin(q(4)),
            sin(q(5)) * sin(q(4)),
            cos(q(5)) * sin(q(4)),
            sin(q(6)) * sin(q(4)),
            cos(q(6)) * sin(q(4)),
            // 6
            cos(q(4)) * cos(q(4)),
            sin(q(5)) * cos(q(4)),
            cos(q(5)) * cos(q(4)),
            sin(q(6)) * cos(q(4)),
            cos(q(6)) * cos(q(4)),
            // 7
            sin(q(5)) * sin(q(5)),
            cos(q(5)) * sin(q(5)),
            sin(q(6)) * sin(q(5)),
            cos(q(6)) * sin(q(5)),
            // 8
            cos(q(5)) * cos(q(5)),
            sin(q(6)) * cos(q(5)),
            cos(q(6)) * cos(q(5)),
            // 9
            sin(q(6)) * sin(q(6)),
            cos(q(6)) * sin(q(6)),
            // 10
            cos(q(6)) * cos(q(6));*/

    //////////// Version 11: Previous version without x and z ////////////////////
    VectorX<U> feature(70);  // 4 + 1 + 10 + (10C2 + 10) = 4 + 1 + 10 + 55 = 70
    feature << feature_base,
            1,
            sin(q(2)),
            cos(q(2)),
            sin(q(3)),
            cos(q(3)),
            sin(q(4)),
            cos(q(4)),
            sin(q(5)),
            cos(q(5)),
            sin(q(6)),
            cos(q(6)),  // linear until here, below are quadratic
            // 1
            sin(q(2)) * sin(q(2)),
            cos(q(2)) * sin(q(2)),
            sin(q(3)) * sin(q(2)),
            cos(q(3)) * sin(q(2)),
            sin(q(4)) * sin(q(2)),
            cos(q(4)) * sin(q(2)),
            sin(q(5)) * sin(q(2)),
            cos(q(5)) * sin(q(2)),
            sin(q(6)) * sin(q(2)),
            cos(q(6)) * sin(q(2)),
            // 2
            cos(q(2)) * cos(q(2)),
            sin(q(3)) * cos(q(2)),
            cos(q(3)) * cos(q(2)),
            sin(q(4)) * cos(q(2)),
            cos(q(4)) * cos(q(2)),
            sin(q(5)) * cos(q(2)),
            cos(q(5)) * cos(q(2)),
            sin(q(6)) * cos(q(2)),
            cos(q(6)) * cos(q(2)),
            // 3
            sin(q(3)) * sin(q(3)),
            cos(q(3)) * sin(q(3)),
            sin(q(4)) * sin(q(3)),
            cos(q(4)) * sin(q(3)),
            sin(q(5)) * sin(q(3)),
            cos(q(5)) * sin(q(3)),
            sin(q(6)) * sin(q(3)),
            cos(q(6)) * sin(q(3)),
            // 4
            cos(q(3)) * cos(q(3)),
            sin(q(4)) * cos(q(3)),
            cos(q(4)) * cos(q(3)),
            sin(q(5)) * cos(q(3)),
            cos(q(5)) * cos(q(3)),
            sin(q(6)) * cos(q(3)),
            cos(q(6)) * cos(q(3)),
            // 5
            sin(q(4)) * sin(q(4)),
            cos(q(4)) * sin(q(4)),
            sin(q(5)) * sin(q(4)),
            cos(q(5)) * sin(q(4)),
            sin(q(6)) * sin(q(4)),
            cos(q(6)) * sin(q(4)),
            // 6
            cos(q(4)) * cos(q(4)),
            sin(q(5)) * cos(q(4)),
            cos(q(5)) * cos(q(4)),
            sin(q(6)) * cos(q(4)),
            cos(q(6)) * cos(q(4)),
            // 7
            sin(q(5)) * sin(q(5)),
            cos(q(5)) * sin(q(5)),
            sin(q(6)) * sin(q(5)),
            cos(q(6)) * sin(q(5)),
            // 8
            cos(q(5)) * cos(q(5)),
            sin(q(6)) * cos(q(5)),
            cos(q(6)) * cos(q(5)),
            // 9
            sin(q(6)) * sin(q(6)),
            cos(q(6)) * sin(q(6)),
            // 10
            cos(q(6)) * cos(q(6));

    return feature;
  } else if (robot_option_ == 1) {

    //////////// Version 1: features contain LIPM & swing foot //////////////////
    // Currently it doesn't include floating base coordinates
    // Didn't use sin and cos just to keep the size small for now

    // Get CoM position
    plant_->SetPositions(context_.get(), q);
    VectorX<U> CoM = plant_->CalcCenterOfMassPosition(*context_);
    // Stance foot position (left foot)
    VectorX<U> left_foot_pos(3);
    const auto & left_toe = plant_->GetBodyByName("toe_left");
    plant_->CalcPointsPositions(*context_, left_toe.body_frame(), mid_disp_,
                                plant_->world_frame(), &left_foot_pos);
    VectorX<U> st_to_CoM = CoM - left_foot_pos;
    // Swing foot position (right foot)
    const auto & right_toe = plant_->GetBodyByName("toe_right");
    VectorX<U> right_foot_pos(3);
    plant_->CalcPointsPositions(*context_, right_toe.body_frame(), mid_disp_,
                                plant_->world_frame(), &right_foot_pos);
    VectorX<U> CoM_to_sw = right_foot_pos - CoM;
    // cout << "CoM = " << CoM.transpose() << endl;
    // cout << "left_foot_pos = " << left_foot_pos.transpose() << endl;
    // cout << "right_foot_pos = " << right_foot_pos.transpose() << endl;
    // cout << "CoM from MBP = " << CoM(0) << " " << CoM(2) << endl;
    // cout << "st_to_CoM from MBP = " << st_to_CoM(0) << " " << st_to_CoM(2) << endl;
    // cout << "CoM_to_sw from MBP = " << CoM_to_sw(0) << " " << CoM_to_sw(2) << endl;

    VectorX<U> feature_base(4);
    feature_base << st_to_CoM(0), st_to_CoM(2), CoM_to_sw(0), CoM_to_sw(2);

    // elements: (no ankle joint, since it's redundant info for fixed_spring model)
    // q(7)
    // q(8)
    // q(9)
    // q(10)
    // q(11)
    // q(12)
    // q(13)
    // q(14)
    // q(17)
    // q(18)
    // feature_base(0), feature_base(1), feature_base(2), feature_base(3)

    VectorX<U> feature(70);  // 4 + 1 + 10 + (10C2 + 10) = 4 + 1 + 10 + 55 = 70
    feature << feature_base,
            1,
            q(7),
            q(8),
            q(9),
            q(10),
            q(11),
            q(12),
            q(13),
            q(14),
            q(17),
            q(18),  // linear until here, below are quadratic
            // 1
            q(7) * q(7),
            q(8) * q(7),
            q(9) * q(7),
            q(10) * q(7),
            q(11) * q(7),
            q(12) * q(7),
            q(13) * q(7),
            q(14) * q(7),
            q(17) * q(7),
            q(18) * q(7),
            // 2
            q(8) * q(8),
            q(9) * q(8),
            q(10) * q(8),
            q(11) * q(8),
            q(12) * q(8),
            q(13) * q(8),
            q(14) * q(8),
            q(17) * q(8),
            q(18) * q(8),
            // 3
            q(9) * q(9),
            q(10) * q(9),
            q(11) * q(9),
            q(12) * q(9),
            q(13) * q(9),
            q(14) * q(9),
            q(17) * q(9),
            q(18) * q(9),
            // 4
            q(10) * q(10),
            q(11) * q(10),
            q(12) * q(10),
            q(13) * q(10),
            q(14) * q(10),
            q(17) * q(10),
            q(18) * q(10),
            // 5
            q(11) * q(11),
            q(12) * q(11),
            q(13) * q(11),
            q(14) * q(11),
            q(17) * q(11),
            q(18) * q(11),
            // 6
            q(12) * q(12),
            q(13) * q(12),
            q(14) * q(12),
            q(17) * q(12),
            q(18) * q(12),
            // 7
            q(13) * q(13),
            q(14) * q(13),
            q(17) * q(13),
            q(18) * q(13),
            // 8
            q(14) * q(14),
            q(17) * q(14),
            q(18) * q(14),
            // 9
            q(17) * q(17),
            q(18) * q(17),
            // 10
            q(18) * q(18);

    return feature;
  }

  DRAKE_DEMAND(false);  // shouldn't reach to this line of code
}



template <typename T>
template <typename U>
VectorX<U> KinematicsExpression<T>::getFeatureDot(
  const VectorX<U> & q, const VectorX<U> & v) const {

  if (robot_option_ == 0) {
    //////////// Version 11: Previous version without x and z ////////////////////
    //// Way 2: Get CoM and foot position by hand //////
    // Calculate the CoM by hand instead of by using MBP
    VectorX<U> CoM_xz_dot(2);
    CoM_xz_dot << v(0) + (0.3 * cos(q(2))*v(2)) / 2.0 + (
                 - 0.75 * cos(q(2) + q(3)) * (v(2) + v(3))
                 - 0.25 * cos(q(2) + q(3) + q(5)) * (v(2) + v(3) + v(5))
                 - 0.75 * cos(q(2) + q(4)) * (v(2) + v(4))
                 - 0.25 * cos(q(2) + q(4) + q(6)) * (v(2) + v(4) + v(6))) / 8.0,
                 v(1) + (- 0.3 * sin(q(2))*v(2)) / 2.0 + (
                   + 0.75 * sin(q(2) + q(3)) * (v(2) + v(3))
                   + 0.25 * sin(q(2) + q(3) + q(5)) * (v(2) + v(3) + v(5))
                   + 0.75 * sin(q(2) + q(4)) * (v(2) + v(4))
                   + 0.25 * sin(q(2) + q(4) + q(6)) * (v(2) + v(4) + v(6))) / 8.0;
    // Calculate the stance foot by hand instead of by using MBP
    VectorX<U> left_foot_pos_xz_dot(2);
    left_foot_pos_xz_dot <<
                         v(0) - 0.5 * cos(q(2) + q(3))*(v(2) + v(3))
                         - 0.5 * cos(q(2) + q(3) + q(5))*(v(2) + v(3) + v(5)),
                         v(1) + 0.5 * sin(q(2) + q(3))*(v(2) + v(3))
                         + 0.5 * sin(q(2) + q(3) + q(5))*(v(2) + v(3) + v(5));
    VectorX<U> st_to_CoM_xz_dot = CoM_xz_dot - left_foot_pos_xz_dot;
    // Calculate the swing foot by hand instead of by using MBP
    VectorX<U> right_foot_pos_xz_dot(2);
    right_foot_pos_xz_dot <<
                          v(0) - 0.5 * cos(q(2) + q(4))*(v(2) + v(4))
                          - 0.5 * cos(q(2) + q(4) + q(6))*(v(2) + v(4) + v(6)),
                          v(1) + 0.5 * sin(q(2) + q(4))*(v(2) + v(4))
                          + 0.5 * sin(q(2) + q(4) + q(6))*(v(2) + v(4) + v(6));
    VectorX<U> CoM_to_sw_xz_dot = right_foot_pos_xz_dot - CoM_xz_dot;
    // cout << "CoM_xz_dot = " << CoM_xz_dot.transpose() << endl;
    // cout << "st_to_CoM_xz_dot = " << st_to_CoM_xz_dot.transpose() << endl;
    // cout << "CoM_to_sw_xz_dot = " << CoM_to_sw_xz_dot.transpose() << endl;

    VectorX<U> feature_base_dot(4);
    feature_base_dot << st_to_CoM_xz_dot, CoM_to_sw_xz_dot;

    // elements:
    // sin(q(2)), cos(q(2))
    // sin(q(3)), cos(q(3))
    // sin(q(4)), cos(q(4))
    // sin(q(5)), cos(q(5))
    // sin(q(6)), cos(q(6))
    // feature_base_dot(0), feature_base_dot(1), feature_base_dot(2), feature_base_dot(3)

    /*
    VectorX<U> feature_dot(70);  // 4 + 1 + 10 + (10C2 + 10) = 4 + 1 + 10 + 55 = 70
    feature_dot << feature_base_dot,
                0,
                 cos(q(2)) *v(2),
                -sin(q(2)) *v(2),
                 cos(q(3)) *v(3),
                -sin(q(3)) *v(3),
                 cos(q(4)) *v(4),
                -sin(q(4)) *v(4),
                 cos(q(5)) *v(5),
                -sin(q(5)) *v(5),
                 cos(q(6)) *v(6),
                -sin(q(6)) *v(6),  // linear until here, below are quadratic
                // 1
                2 * sin(q(2)) * cos(q(2)) * v(2),
                -sin(q(2)) * v(2) * sin(q(2)) + cos(q(2)) * cos(q(2)) * v(2),
                 cos(q(3)) * v(3) * sin(q(2)) + sin(q(3)) * cos(q(2)) * v(2),
                -sin(q(3)) * v(3) * sin(q(2)) + cos(q(3)) * cos(q(2)) * v(2),
                 cos(q(4)) * v(4) * sin(q(2)) + sin(q(4)) * cos(q(2)) * v(2),
                -sin(q(4)) * v(4) * sin(q(2)) + cos(q(4)) * cos(q(2)) * v(2),
                 cos(q(5)) * v(5) * sin(q(2)) + sin(q(5)) * cos(q(2)) * v(2),
                -sin(q(5)) * v(5) * sin(q(2)) + cos(q(5)) * cos(q(2)) * v(2),
                 cos(q(6)) * v(6) * sin(q(2)) + sin(q(6)) * cos(q(2)) * v(2),
                -sin(q(6)) * v(6) * sin(q(2)) + cos(q(6)) * cos(q(2)) * v(2),
                // 2
                -sin(q(2)) * v(2) * cos(q(2)) + cos(q(2)) * (-sin(q(2))) * v(2),
                 cos(q(3)) * v(3) * cos(q(2)) + sin(q(3)) * (-sin(q(2))) * v(2),
                -sin(q(3)) * v(3) * cos(q(2)) + cos(q(3)) * (-sin(q(2))) * v(2),
                 cos(q(4)) * v(4) * cos(q(2)) + sin(q(4)) * (-sin(q(2))) * v(2),
                -sin(q(4)) * v(4) * cos(q(2)) + cos(q(4)) * (-sin(q(2))) * v(2),
                 cos(q(5)) * v(5) * cos(q(2)) + sin(q(5)) * (-sin(q(2))) * v(2),
                -sin(q(5)) * v(5) * cos(q(2)) + cos(q(5)) * (-sin(q(2))) * v(2),
                 cos(q(6)) * v(6) * cos(q(2)) + sin(q(6)) * (-sin(q(2))) * v(2),
                -sin(q(6)) * v(6) * cos(q(2)) + cos(q(6)) * (-sin(q(2))) * v(2),
                // 3
                 cos(q(3)) * v(3) * sin(q(3)) + sin(q(3)) * cos(q(3)) * v(3),
                -sin(q(3)) * v(3) * sin(q(3)) + cos(q(3)) * cos(q(3)) * v(3),
                 cos(q(4)) * v(4) * sin(q(3)) + sin(q(4)) * cos(q(3)) * v(3),
                -sin(q(4)) * v(4) * sin(q(3)) + cos(q(4)) * cos(q(3)) * v(3),
                 cos(q(5)) * v(5) * sin(q(3)) + sin(q(5)) * cos(q(3)) * v(3),
                -sin(q(5)) * v(5) * sin(q(3)) + cos(q(5)) * cos(q(3)) * v(3),
                 cos(q(6)) * v(6) * sin(q(3)) + sin(q(6)) * cos(q(3)) * v(3),
                -sin(q(6)) * v(6) * sin(q(3)) + cos(q(6)) * cos(q(3)) * v(3),
                // 4
                -sin(q(3)) * v(3) * cos(q(3)) + cos(q(3)) * (-sin(q(3))) * v(3),
                 cos(q(4)) * v(4)* cos(q(3)) + sin(q(4)) * (-sin(q(3))) * v(3),
                -sin(q(4)) * v(4) * cos(q(3)) + cos(q(4)) * (-sin(q(3))) * v(3),
                 cos(q(5)) * v(5)* cos(q(3)) + sin(q(5)) * (-sin(q(3))) * v(3),
                -sin(q(5)) * v(5) * cos(q(3)) + cos(q(5)) * (-sin(q(3))) * v(3),
                 cos(q(6)) * v(6)* cos(q(3)) + sin(q(6)) * (-sin(q(3))) * v(3),
                -sin(q(6)) * v(6) * cos(q(3)) + cos(q(6)) * (-sin(q(3))) * v(3),
                // 5
                 cos(q(4)) * v(4) * sin(q(4)) + sin(q(4)) * cos(q(4)) * v(4),
                -sin(q(4)) * v(4) * sin(q(4)) + cos(q(4)) * cos(q(4)) * v(4),
                 cos(q(5)) * v(5) * sin(q(4)) + sin(q(5)) * cos(q(4)) * v(4),
                -sin(q(5)) * v(5) * sin(q(4)) + cos(q(5)) * cos(q(4)) * v(4),
                 cos(q(6)) * v(6) * sin(q(4)) + sin(q(6)) * cos(q(4)) * v(4),
                -sin(q(6)) * v(6) * sin(q(4)) + cos(q(6)) * cos(q(4)) * v(4),
                // 6
                -sin(q(4)) * v(4)* cos(q(4)) + cos(q(4)) * (-sin(q(4))) * v(4),
                 cos(q(5)) * v(5)* cos(q(4)) + sin(q(5)) * (-sin(q(4))) * v(4),
                -sin(q(5)) * v(5)* cos(q(4)) + cos(q(5)) * (-sin(q(4))) * v(4),
                 cos(q(6)) * v(6)* cos(q(4)) + sin(q(6)) * (-sin(q(4))) * v(4),
                -sin(q(6)) * v(6)* cos(q(4)) + cos(q(6)) * (-sin(q(4))) * v(4),
                // 7
                 cos(q(5)) * v(5) * sin(q(5)) + sin(q(5)) * cos(q(5)) * v(5),
                -sin(q(5)) * v(5) * sin(q(5)) + cos(q(5)) * cos(q(5)) * v(5),
                 cos(q(6)) * v(6) * sin(q(5)) + sin(q(6)) * cos(q(5)) * v(5),
                -sin(q(6)) * v(6) * sin(q(5)) + cos(q(6)) * cos(q(5)) * v(5),
                // 8
                -sin(q(5)) * v(5) * cos(q(5)) + cos(q(5)) * (-sin(q(5))) * v(5),
                 cos(q(6)) * v(6) * cos(q(5)) + sin(q(6)) * (-sin(q(5))) * v(5),
                -sin(q(6)) * v(6) * cos(q(5)) + cos(q(6)) * (-sin(q(5))) * v(5),
                // 9
                 cos(q(6)) * v(6)* sin(q(6)) + sin(q(6)) * cos(q(6)) * v(6),
                -sin(q(6)) * v(6)* sin(q(6)) + cos(q(6)) * cos(q(6)) * v(6),
                // 10
                -2 * cos(q(6)) * sin(q(6)) * v(6);*/

    VectorX<U> feature_dot(70);  // 4 + 1 + 10 + (10C2 + 10) = 4 + 1 + 10 + 55 = 70

    U sin_q2 = sin(q(2));
    U cos_q2 = cos(q(2));
    U sin_q3 = sin(q(3));
    U cos_q3 = cos(q(3));
    U sin_q4 = sin(q(4));
    U cos_q4 = cos(q(4));
    U sin_q5 = sin(q(5));
    U cos_q5 = cos(q(5));
    U sin_q6 = sin(q(6));
    U cos_q6 = cos(q(6));

    U d_sin_q2 = cos_q2 * v(2);
    U d_cos_q2 = -sin_q2 * v(2);
    U d_sin_q3 = cos_q3 * v(3);
    U d_cos_q3 = -sin_q3 * v(3);
    U d_sin_q4 = cos_q4 * v(4);
    U d_cos_q4 = -sin_q4 * v(4);
    U d_sin_q5 = cos_q5 * v(5);
    U d_cos_q5 = -sin_q5 * v(5);
    U d_sin_q6 = cos_q6 * v(6);
    U d_cos_q6 = -sin_q6 * v(6);

    feature_dot << feature_base_dot,
                0,
                d_sin_q2,
                d_cos_q2,
                d_sin_q3,
                d_cos_q3,
                d_sin_q4,
                d_cos_q4,
                d_sin_q5,
                d_cos_q5,
                d_sin_q6,
                d_cos_q6,  // linear until here, below are quadratic
                // 1
                2 * sin_q2 * d_sin_q2,
                d_cos_q2 * sin_q2 + cos_q2 * d_sin_q2,
                d_sin_q3 * sin_q2 + sin_q3 * d_sin_q2,
                d_cos_q3 * sin_q2 + cos_q3 * d_sin_q2,
                d_sin_q4 * sin_q2 + sin_q4 * d_sin_q2,
                d_cos_q4 * sin_q2 + cos_q4 * d_sin_q2,
                d_sin_q5 * sin_q2 + sin_q5 * d_sin_q2,
                d_cos_q5 * sin_q2 + cos_q5 * d_sin_q2,
                d_sin_q6 * sin_q2 + sin_q6 * d_sin_q2,
                d_cos_q6 * sin_q2 + cos_q6 * d_sin_q2,
                // 2
                d_cos_q2 * cos_q2 + cos_q2 * d_cos_q2,
                d_sin_q3 * cos_q2 + sin_q3 * d_cos_q2,
                d_cos_q3 * cos_q2 + cos_q3 * d_cos_q2,
                d_sin_q4 * cos_q2 + sin_q4 * d_cos_q2,
                d_cos_q4 * cos_q2 + cos_q4 * d_cos_q2,
                d_sin_q5 * cos_q2 + sin_q5 * d_cos_q2,
                d_cos_q5 * cos_q2 + cos_q5 * d_cos_q2,
                d_sin_q6 * cos_q2 + sin_q6 * d_cos_q2,
                d_cos_q6 * cos_q2 + cos_q6 * d_cos_q2,
                // 3
                d_sin_q3 * sin_q3 + sin_q3 * d_sin_q3,
                d_cos_q3 * sin_q3 + cos_q3 * d_sin_q3,
                d_sin_q4 * sin_q3 + sin_q4 * d_sin_q3,
                d_cos_q4 * sin_q3 + cos_q4 * d_sin_q3,
                d_sin_q5 * sin_q3 + sin_q5 * d_sin_q3,
                d_cos_q5 * sin_q3 + cos_q5 * d_sin_q3,
                d_sin_q6 * sin_q3 + sin_q6 * d_sin_q3,
                d_cos_q6 * sin_q3 + cos_q6 * d_sin_q3,
                // 4
                d_cos_q3 * cos_q3 + cos_q3 * d_cos_q3,
                d_sin_q4 * cos_q3 + sin_q4 * d_cos_q3,
                d_cos_q4 * cos_q3 + cos_q4 * d_cos_q3,
                d_sin_q5 * cos_q3 + sin_q5 * d_cos_q3,
                d_cos_q5 * cos_q3 + cos_q5 * d_cos_q3,
                d_sin_q6 * cos_q3 + sin_q6 * d_cos_q3,
                d_cos_q6 * cos_q3 + cos_q6 * d_cos_q3,
                // 5
                d_sin_q4 * sin_q4 + sin_q4 * d_sin_q4,
                d_cos_q4 * sin_q4 + cos_q4 * d_sin_q4,
                d_sin_q5 * sin_q4 + sin_q5 * d_sin_q4,
                d_cos_q5 * sin_q4 + cos_q5 * d_sin_q4,
                d_sin_q6 * sin_q4 + sin_q6 * d_sin_q4,
                d_cos_q6 * sin_q4 + cos_q6 * d_sin_q4,
                // 6
                d_cos_q4 * cos_q4 + cos_q4 * d_cos_q4,
                d_sin_q5 * cos_q4 + sin_q5 * d_cos_q4,
                d_cos_q5 * cos_q4 + cos_q5 * d_cos_q4,
                d_sin_q6 * cos_q4 + sin_q6 * d_cos_q4,
                d_cos_q6 * cos_q4 + cos_q6 * d_cos_q4,
                // 7
                d_sin_q5 * sin_q5 + sin_q5 * d_sin_q5,
                d_cos_q5 * sin_q5 + cos_q5 * d_sin_q5,
                d_sin_q6 * sin_q5 + sin_q6 * d_sin_q5,
                d_cos_q6 * sin_q5 + cos_q6 * d_sin_q5,
                // 8
                d_cos_q5 * cos_q5 + cos_q5 * d_cos_q5,
                d_sin_q6 * cos_q5 + sin_q6 * d_cos_q5,
                d_cos_q6 * cos_q5 + cos_q6 * d_cos_q5,
                // 9
                d_sin_q6 * sin_q6 + sin_q6 * d_sin_q6,
                d_cos_q6 * sin_q6 + cos_q6 * d_sin_q6,
                // 10
                2 * cos_q6 * d_cos_q6;

    return feature_dot;
  } else if (robot_option_ == 1) {
  }

  DRAKE_DEMAND(false);  // shouldn't reach to this line of code
}





// Instantiation

// class KinematicsExpression //////////////////////////////////////////////////
template class KinematicsExpression<double>;
template class KinematicsExpression<AutoDiffXd>;

// method getExpression ////////////////////////////////////////////////////////
template VectorX<double> KinematicsExpression<double>::getExpression(
  const VectorX<double> &, const VectorX<double> &) const;

// template VectorX<AutoDiffXd> KinematicsExpression<AutoDiffXd>::getExpression(
//   const VectorX<double> &, const VectorX<double> &) const;
template VectorX<AutoDiffXd> KinematicsExpression<AutoDiffXd>::getExpression(
  const VectorX<double> &, const VectorX<AutoDiffXd> &) const;
// We can have the one below if we have MBP in the feature.
// template VectorX<AutoDiffXd> KinematicsExpression<AutoDiffXd>::getExpression(
//   const VectorX<AutoDiffXd> &, const VectorX<double> &) const;

// Seems that the case when theta and q are both autoDiff doesn't work. (I
// tested with an example.)
// template VectorX<AutoDiffXd> KinematicsExpression<AutoDiffXd>::getExpression(
//   const VectorX<AutoDiffXd> &, const VectorX<AutoDiffXd> &) const;

// method getFeature ///////////////////////////////////////////////////////////
template VectorX<double> KinematicsExpression<double>::getFeature(
  const VectorX<double> &) const;

// template VectorX<double> KinematicsExpression<AutoDiffXd>::getFeature(
//   const VectorX<double> &) const;
template VectorX<AutoDiffXd> KinematicsExpression<AutoDiffXd>::getFeature(
  const VectorX<AutoDiffXd> &) const;

// method getExpressionDot /////////////////////////////////////////////////////
template VectorX<double> KinematicsExpression<double>::getExpressionDot(
  const VectorX<double> &,
  const VectorX<double> &, const VectorX<double> &) const;
template VectorX<AutoDiffXd> KinematicsExpression<AutoDiffXd>::getExpressionDot(
  const VectorX<double> &,
  const VectorX<AutoDiffXd> &, const VectorX<AutoDiffXd> &) const;
// method getFeatureDot ////////////////////////////////////////////////////////
template VectorX<double> KinematicsExpression<double>::getFeatureDot(
  const VectorX<double> &, const VectorX<double> &) const;
template VectorX<AutoDiffXd> KinematicsExpression<AutoDiffXd>::getFeatureDot(
  const VectorX<AutoDiffXd> &, const VectorX<AutoDiffXd> &) const;

}  // namespace goldilocks_models
}  // namespace dairlib

