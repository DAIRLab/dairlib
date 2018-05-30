# -*- mode: cmake -*-
# vi: set ft=cmake :

#set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_FLAGS "-O2")

configure_file(PlanarWalker.urdf "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)
configure_file(PlanarWalkerWithTorso.urdf "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)
configure_file(z_save.csv "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)

find_package(GFlags MODULE REQUIRED COMPONENTS shared)

add_executable(run_gait_dircon run_gait_dircon.cc)

target_link_libraries(run_gait_dircon goldilocks
 dircon drake::drake drake::drake-common-text-logging-gflags gflags_shared
)

add_executable(runSGD runSGD.cc sgd_iter.cc)

set_target_properties(runSGD PROPERTIES
  PUBLIC_HEADER "sgd_iter.h;")

target_link_libraries(runSGD goldilocks
 dircon drake::drake drake::drake-common-text-logging-gflags gflags_shared
)

add_executable(generate_nominal_gaits generate_nominal_gaits.cc sgd_iter.cc)

set_target_properties(generate_nominal_gaits PROPERTIES
  PUBLIC_HEADER "sgd_iter.h;")

target_link_libraries(generate_nominal_gaits goldilocks
 dircon drake::drake drake::drake-common-text-logging-gflags gflags_shared
)

add_executable(runSGDIter runSGDIter.cc sgd_iter.cc)

set_target_properties(runSGDIter PROPERTIES
  PUBLIC_HEADER "sgd_iter.h;")

target_link_libraries(runSGDIter goldilocks
 dircon drake::drake drake::drake-common-text-logging-gflags gflags_shared
)

add_executable(visualize_gait visualize_gait.cc)


target_link_libraries(visualize_gait goldilocks
 dircon drake::drake drake::drake-common-text-logging-gflags gflags_shared
)