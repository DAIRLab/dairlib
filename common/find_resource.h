#pragma once

#include "drake/common/find_resource.h"

namespace dairlib {

/// Modeled after a subset of drake::FindResourceOrThrow, this searches
/// for the root dairlib directory and searches from there
std::string FindResourceOrThrow(std::string resource_path);

} 
