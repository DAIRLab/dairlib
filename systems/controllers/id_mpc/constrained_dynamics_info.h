#include "id_mpc_types.h"

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
                       std::vector<int> active_constraint_directions);

 private:
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