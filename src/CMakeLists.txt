# -*- mode: cmake -*-
# vi: set ft=cmake :

set(CMAKE_BUILD_TYPE Debug)

add_library(goldilocks manifold_constraint.cc file_utils.cc)
target_link_libraries(goldilocks dircon drake::drake)

set_target_properties(goldilocks PROPERTIES
  PUBLIC_HEADER "manifold_constraint.h;file_utils.h;")