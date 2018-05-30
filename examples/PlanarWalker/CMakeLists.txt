# -*- mode: cmake -*-
# vi: set ft=cmake :

#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS "-O2")

configure_file(PlanarWalker.urdf "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)

find_package(GFlags MODULE REQUIRED COMPONENTS shared)

add_executable(run_passive run_passive.cc)
target_link_libraries(run_passive 
  drake::drake drake::drake-common-text-logging-gflags gflags_shared
)

add_executable(test_contact test_contact.cc)
target_link_libraries(test_contact 
  drake::drake drake::drake-common-text-logging-gflags gflags_shared
)

add_executable(run_gait_dircon run_gait_dircon.cc)
target_include_directories(run_gait_dircon PRIVATE
"${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(run_gait_dircon
 dircon drake::drake drake::drake-common-text-logging-gflags gflags_shared
)
