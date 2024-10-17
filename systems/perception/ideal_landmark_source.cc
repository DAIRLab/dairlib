#include "ideal_landmark_source.h"
#include "multibody/multibody_utils.h"
#include "systems/framework/output_vector.h"

namespace dairlib {
namespace perception {

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Matrix3Xd;

using drake::systems::Context;
using drake::math::RigidTransform;
using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBodyFrame;

using systems::OutputVector;

IdealLandmarkSource::IdealLandmarkSource(
    const MultibodyPlant<double> &plant, Context<double> *context,
    const RigidBodyFrame<double>& landmark_frame) :
    plant_(plant), landmark_frame_(landmark_frame), context_(context) {

  DRAKE_DEMAND(&(landmark_frame.GetParentPlant()) == &plant);

  DeclareVectorInputPort("x, u, t", OutputVector<double>(plant));
  DeclareAbstractOutputPort(
      "lcmt_landmark_array", &IdealLandmarkSource::CalcLandmarks);
}

void IdealLandmarkSource::CalcLandmarks(
    const Context<double> &context, lcmt_landmark_array *out) const {

  Matrix3Xd landmarks = Matrix3Xd::Zero(3, 7);
  landmarks.rightCols<3>() = Eigen::Matrix3d::Identity();
  landmarks.block<3,3>(0, 3) = -Eigen::Matrix3d::Identity();

  out->num_landmarks = 7;
  out->parent_frame = landmark_frame_.name();
  out->utime = 1e6 * context.get_time();
  out->landmarks.clear();

  for (int i = 0; i < 7; ++i) {

  }
}

}
}