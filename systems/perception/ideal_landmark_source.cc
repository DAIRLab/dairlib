#include "ideal_landmark_source.h"
#include "multibody/multibody_utils.h"

namespace dairlib {
namespace perception {

using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::Matrix3Xd;

using drake::systems::Context;
using drake::math::RigidTransform;
using drake::multibody::MultibodyPlant;
using drake::multibody::BodyFrame;

IdealLandmarkSource::IdealLandmarkSource(
    const MultibodyPlant<double> &plant, Context<double> *context,
    const BodyFrame<double>& landmark_frame) :
    plant_(plant), landmark_frame_(landmark_frame), context_(context) {

  DRAKE_DEMAND(&(landmark_frame.GetParentPlant()) == &plant);

  DeclareVectorInputPort("x", plant.num_positions() + plant.num_velocities());
  DeclareAbstractOutputPort(
      "lcmt_landmark_array", &IdealLandmarkSource::CalcLandmarks);
}

void IdealLandmarkSource::CalcLandmarks(
    const Context<double> &context, lcmt_landmark_array *out) const {

  int num_landmarks = 25;

  Matrix3Xd landmarks = Matrix3Xd::Zero(3, num_landmarks);
  for (int i = 0; i < num_landmarks; ++i) {
    landmarks.col(i) = Vector3d(0.1 * i * cos(i), 0.1 * i * sin(i), 0.3 * cos(i));
  }

  out->num_landmarks = num_landmarks;
  out->parent_frame = landmark_frame_.name();
  out->utime = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now().time_since_epoch()).count();
  out->landmarks.clear();

  const VectorXd& x = EvalVectorInput(context, 0)->get_value();

  multibody::SetPositionsIfNew<double>(
      plant_, x.head(plant_.num_positions()), context_);

  const RigidTransform<double> frame_pose =
      landmark_frame_.CalcPoseInWorld(*context_);

  for (int i = 0; i < num_landmarks; ++i) {
    Vector3d landmark_pos = frame_pose.inverse() * landmarks.col(i);
    lcmt_landmark landmark{};
    landmark.id = i;
    memcpy(landmark.position, landmark_pos.data(), 3 * sizeof(double));
    out->landmarks.push_back(landmark);
  }

  out->num_expired = 3;
  out->expired_landmark_ids.clear();
  for (int i = 0; i < out->num_expired ; ++i) {
    int to_delete = ((int)(context.get_time() * 2) + i) % num_landmarks;
    out->expired_landmark_ids.push_back(to_delete);
  }

}

}
}