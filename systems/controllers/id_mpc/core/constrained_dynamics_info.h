#pragma once

#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "drake/common/eigen_autodiff_types.h"

namespace dairlib::systems::controllers::id_mpc {

template <typename T>
using ContactConstraintMap = std::unordered_map<
    std::string,
    std::unique_ptr<const multibody::WorldPointEvaluator<T>>>;

using drake::AutoDiffXd;
using drake::multibody::MultibodyPlant;

class ConstrainedDynamicsInfo {
 public:
  /*!
   * Wrapper class to hold the dynamics and contact info for an MPC problem
   * @param urdf the filepath of the URDF for the plant
   */
  explicit ConstrainedDynamicsInfo(std::string urdf);

  const MultibodyPlant<double>& get_plant() const {
    DRAKE_ASSERT(plant_!= nullptr);
    return *plant_;
  }

  const MultibodyPlant<AutoDiffXd>& get_plant_ad() const {
    DRAKE_ASSERT(plant_ad_ != nullptr);
    return *plant_ad_;
  }

  int variable_count() const {
    return nq_ + nv_ + nu_ + nh_ + nc_;
  }

  int nq() const { return nq_; }
  int nv() const { return nv_; }
  int nu() const { return nu_; }
  int nh() const { return nh_; }
  int nc() const { return nc_; }
  int nc_active() const { return nc_active_; }
  int nx() const { return nq_ + nv_; }
  int n_constraint_total() const { return nh_ + nc_active_; }

  /*!
   * Add a distance constraint between two points
   * @param body_A Name of the first body. The owned plant must have a body
   * with this name
   * @param pt_A Point on Body A
   * @param body_B Name of the second Body
   * @param pt_B Point on Body B
   * @param distance the distance of the constraint
   */
  void AddDistanceConstraint(std::string body_A, const Eigen::Vector3d& pt_A,
                             std::string body_B, const Eigen::Vector3d& pt_B,
                             double distance);

  /*!
   * Add a contact point for a contact constraint
   * @param name name of the contact point. Must be unique.
   * @param body name of the body on which the contact point lives.
   * @param point_in_body_frame contact point in the body frame
   * @param active_constraint_directions used to ensure the contact Jacobian
   * has no redundant rows.
   */
  void AddContactPoint(std::string name, std::string body,
                       const Eigen::Vector3d& point_in_body_frame,
                       std::vector<int> active_constraint_directions,
                       double friction_coefficient);

  /*!
   * Set the state of a plant context via the internal plant
   * @tparam T scalar type (double or AutoDiffXd)
   * @param context context to set
   */
  template<typename T>
  void SetPlantStateIfNew(const drake::VectorX<T>& x,
                     drake::systems::Context<T>* context) const;

  /*!
   * Make the context for the plant with the appropriate scalar type
   * @tparam T double or AutoDiffXd
   * @return unique_ptr of the plant context
   */
  template<typename T>
  std::unique_ptr<drake::systems::Context<T>> MakeContext() const;

  /*!
   * Placeholder struct for the results of evaluating the manipulator dynamics
   * @tparam T scalar type
   */
  template<typename T>
  struct DynamicsEvaluation {
    drake::VectorX<T> qdot_;
    drake::VectorX<T> vdot_;
    drake::VectorX<T> c_;
    drake::VectorX<T> cdot_;
    drake::VectorX<T> cddot_;

    friend std::ostream& operator<<(
        std::ostream& os, const DynamicsEvaluation<T>& eval) {
      os << "qdot: " << eval.qdot_.transpose() << "\n"
         << "vdot: " << eval.vdot_.transpose() << "\n"
         << "c: " << eval.c_.transpose() << "\n"
         << "cdot: " << eval.cdot_.transpose() << "\n"
         << "cddot: " << eval.cddot_.transpose() << "\n";
      return os;
    }
  };

  template<typename T>
  DynamicsEvaluation<T> MakeEmptyDynamicsEvaluation() const;

  template<typename T>
  DynamicsEvaluation<T> EvaluateDynamics(
      const drake::systems::Context<T>& context,
      const drake::VectorX<T>& u, const drake::VectorX<T>& lh,
      const drake::VectorX<T>& lc,
      const std::vector<std::string>& active_contacts) const;

  template<typename T>
  DynamicsEvaluation<T> EvaluateDynamics(
      drake::systems::Context<T>* context,
      const drake::VectorX<T>& all_vars,
      const std::vector<std::string>& active_contacts) const;

 private:

  template<typename T>
  void DoEvaluate(
      const drake::multibody::MultibodyPlant<T>& plant,
      const drake::systems::Context<T>& context,
      const multibody::KinematicEvaluatorSet<T>* holonomic_constraints,
      const ContactConstraintMap<T>& contact_constraint_evaluators,
      const drake::VectorX<T>& u, const drake::VectorX<T>& lh,
      const drake::VectorX<T>& lc,
      const std::vector<std::string>& active_contacts,
      DynamicsEvaluation<T>& eval) const;

  std::unique_ptr<drake::multibody::MultibodyPlant<AutoDiffXd>> plant_ad_;
  std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_;

  // Holonomic Constraints
  std::unique_ptr<multibody::KinematicEvaluatorSet<double>>
    holonomic_constraints_ = nullptr;
  std::unique_ptr<multibody::KinematicEvaluatorSet<drake::AutoDiffXd>>
    holonomic_constraints_ad_ = nullptr;

  std::vector<std::unique_ptr<const multibody::KinematicEvaluator<double>>>
    holonomic_constraint_storage_;
  std::vector<std::unique_ptr<const multibody::KinematicEvaluator<AutoDiffXd>>>
    holonomic_constraint_ad_storage_;

  // Contact Constraints
  ContactConstraintMap<double> contact_constraint_evaluators_{};
  ContactConstraintMap<drake::AutoDiffXd> contact_constraint_evaluators_ad_{};

  // Contact Constraint Book Keeping
  std::unordered_map<std::string, int> lambda_c_start_idxs_{};
  std::unordered_map<std::string, int> Jc_active_start_idxs_{};
  std::unordered_map<std::string, double> mu_map_{};

  // dimensions
  int nq_{};
  int nv_{};
  int nu_{};
  int nh_ = 0;
  int nc_ = 0;
  int nc_active_ = 0;
};

}