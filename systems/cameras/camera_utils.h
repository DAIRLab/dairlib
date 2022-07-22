#pragma once
#include "drake/common/eigen_types.h"
#include "drake/geometry/render/render_engine.h"
#include "drake/math/rigid_transform.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/diagram.h"

// Original code by Hersh Sanghvi, Extended by Brian Acosta

namespace dairlib {
namespace camera {



std::map<int, drake::systems::sensors::CameraInfo> load_realsense_calib_info

std::pair<drake::geometry::render::ColorRenderCamera,
drake::geometry::render::DepthRenderCamera> MakeD415CameraModel(
    const std::string &renderer_name);

std::pair<drake::geometry::render::ColorRenderCamera,
drake::geometry::render::DepthRenderCamera> MakeDairD455CameraModel(
    const std::string& renderer_name);


std::pair<drake::geometry::render::ColorRenderCamera,
drake::geometry::render::DepthRenderCamera> MakeGenericCameraModel(
    const std::string &renderer_name, const int kHeight, const int kWidth);

std::pair <drake::geometry::render::ColorRenderCamera,
drake::geometry::render::DepthRenderCamera> MakeWideCameraModel(
    const std::string &renderer_name);

}
}

