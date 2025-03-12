// STL
#include <utility>

// dairlib
#include "alip_mpfc_meshcat_visualizer.h"
#include "systems/framework/output_vector.h"

//drake
#include "drake/geometry/shape_specification.h"
#include "drake/geometry/rgba.h"


namespace dairlib::perceptive_locomotion {
using geometry::ConvexPolygon;
using geometry::ConvexPolygonSet;
using Eigen::Matrix3d;
using Eigen::Matrix3Xd;
using systems::OutputVector;

AlipMPFCMeshcatVisualizer::AlipMPFCMeshcatVisualizer(
    std::shared_ptr<drake::geometry::Meshcat> meshcat,
    const drake::multibody::MultibodyPlant<double>& plant) :
    meshcat_(std::move(meshcat)) {

  mpc_debug_input_port_ = DeclareAbstractInputPort(
      "mpc_debug", drake::Value<lcmt_alip_mpfc_debug_complete>()
    ).get_index();

  legacy_mpc_debug_input_port_ = DeclareAbstractInputPort(
      "legacy_mpc_debug", drake::Value<lcmt_alip_s2s_mpfc_debug>()
  ).get_index();

  foothold_input_port_ = DeclareAbstractInputPort(
      "terrain", drake::Value<lcmt_foothold_set>()
  ).get_index();


  state_input_port_ = DeclareVectorInputPort(
      "x, u, t", OutputVector<double>(
          plant.num_positions(), plant.num_velocities(), plant.num_actuators())
      ).get_index();

  n_footholds_idx_ = DeclareDiscreteState(1);

  DeclarePeriodicUnrestrictedUpdateEvent(1.0/15.0, 0.,
      &AlipMPFCMeshcatVisualizer::UnrestrictedUpdate);
}


void AlipMPFCMeshcatVisualizer::DrawFootsteps(
    const std::vector<std::vector<double>>& footstep_sol,
    const Eigen::Matrix3d &R_yaw) const {

  std::vector<drake::geometry::Rgba> rgb = {
      drake::geometry::Rgba(1, 0, 0, 0.8),
      drake::geometry::Rgba(0, 0, 1, 0.8)
  };

  for (size_t n = 1; n < footstep_sol.size(); n++) {
    Eigen::Matrix4d X = Eigen::Matrix4d::Identity();
    std::string path = "footstep_sol_" + std::to_string(n);
    X.block<3, 1>(0, 3) =
        R_yaw * Eigen::Vector3d::Map(footstep_sol.at(n).data());
    const auto sphere = drake::geometry::Sphere(.025);
    meshcat_->SetObject(path, sphere, rgb.at(n % 2));
    meshcat_->SetTransform(path, X);
  }
}

void AlipMPFCMeshcatVisualizer::DrawFootholds(ConvexPolygonSet& foothold_set,
                                              int n_prev,
                                              const std::string& prefix) const {
  std::vector<drake::geometry::Rgba> rgb = {
      drake::geometry::Rgba(1, 0, 0, 0.5),
      drake::geometry::Rgba(0, 1, 0, 0.5),
      drake::geometry::Rgba(0, 0, 1, 0.5)
  };
  for (int i = 0; i < foothold_set.size(); i++) {
    auto foothold = foothold_set.polygons().at(i);
    const auto [verts, faces] = foothold.GetSurfaceMesh();
    auto faces_reversed = faces;
    Eigen::Matrix3Xd verts_line = verts;
    verts_line.rightCols<1>() = verts_line.leftCols<1>();
    faces_reversed.row(0).swap(faces_reversed.row(2));
    meshcat_->SetTriangleMesh(prefix + make_foothold_path(i) + "top", verts, faces, rgb.at(1));
    meshcat_->SetTriangleMesh(prefix + make_foothold_path(i) + "bottom", verts, faces_reversed, rgb.at(1));
    meshcat_->SetLine(prefix + make_foothold_path(i) + "boundary", verts_line, 2.0, drake::geometry::Rgba(0, 0, 0, 0));
  }
  for (int i = foothold_set.size(); i < n_prev; i++) {
    meshcat_->Delete(prefix + make_foothold_path(i) + "top");
    meshcat_->Delete(prefix + make_foothold_path(i) + "bottom");
    meshcat_->Delete(prefix + make_foothold_path(i) + "boundary");
  }
}

Eigen::Matrix3d AlipMPFCMeshcatVisualizer::R_WB(
    const Eigen::Vector4d& wxyz) {
  if (wxyz.norm() < 0.98 or wxyz.norm() > 1.02) {
    return Matrix3d::Identity();
  }
  Eigen::Vector4d xyzw = Eigen::Vector4d::Zero();
  xyzw.head<3>() = wxyz.tail<3>();
  xyzw(3) = wxyz(0);
  Eigen::Vector3d base_x = Eigen::Quaterniond(xyzw).toRotationMatrix().col(0);
  return drake::math::RotationMatrixd::MakeZRotation(
      atan2(base_x(1), base_x(0))
  ).matrix();
}

drake::systems::EventStatus AlipMPFCMeshcatVisualizer::UnrestrictedUpdate(
    const drake::systems::Context<double> &context,
    drake::systems::State<double> *state) const {

  const auto robot_output = dynamic_cast<const OutputVector<double>*>(
      this->EvalVectorInput(context, state_input_port_));
  Eigen::Vector4d quat = robot_output->GetState().head<4>();
  const Eigen::Matrix3d R_yaw = R_WB(quat);

  bool use_legacy = false;
  lcmt_alip_mpfc_debug_complete mpc_debug;
  lcmt_alip_s2s_mpfc_debug mpc_debug_legacy;

  if (get_input_port_mpc().HasValue(context)) {
    mpc_debug = EvalAbstractInput(
        context, mpc_debug_input_port_
    )->get_value<lcmt_alip_mpfc_debug_complete>();
  } else {
    use_legacy = true;
    mpc_debug_legacy = EvalAbstractInput(
        context, legacy_mpc_debug_input_port_
    )->get_value<lcmt_alip_s2s_mpfc_debug>();
  }

  ConvexPolygonSet foothold_set;
  if (get_input_port_terrain().HasValue(context)) {
    auto foothold_set_msg = EvalAbstractInput(
        context, foothold_input_port_)->get_value<lcmt_foothold_set>();
    foothold_set = ConvexPolygonSet::CopyFromLcm(foothold_set_msg);
  } else {
    if (not use_legacy) {
      foothold_set = ConvexPolygonSet::CopyFromLcm(mpc_debug.all_footholds);
    } else {
      foothold_set = ConvexPolygonSet::CopyFromLcm(mpc_debug_legacy.foothold_sequence);
    }

    foothold_set.ReExpressInNewFrame(R_yaw.transpose());
  }

  if (not use_legacy) {
    DrawFootsteps(mpc_debug.pp, R_yaw);
  } else {
    DrawFootsteps(mpc_debug_legacy.pp, R_yaw);
  }

  int n_prev = state->get_discrete_state(n_footholds_idx_).get_value()(0);
  DrawFootholds(foothold_set, n_prev);
  state->get_mutable_discrete_state(n_footholds_idx_).set_value(
      Eigen::VectorXd::Constant(1, foothold_set.size()));

//  meshcat_->Flush();
  return drake::systems::EventStatus::Succeeded();
}

}