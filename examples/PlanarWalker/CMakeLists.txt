# -*- mode: cmake -*-
# vi: set ft=cmake :

set(CMAKE_BUILD_TYPE Debug)

configure_file(PlanarWalker.urdf "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)
configure_file(PlanarWalkerWithTorso.urdf "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)

find_package(GFlags MODULE REQUIRED COMPONENTS shared)

add_executable(run_gait_dircon run_gait_dircon.cc)

target_link_libraries(run_gait_dircon goldilocks
 dircon drake::drake drake::drake-common-text-logging-gflags gflags_shared
)

add_executable(sgd_iter sgd_iter.cc)

target_link_libraries(sgd_iter goldilocks
 dircon drake::drake drake::drake-common-text-logging-gflags gflags_shared
)