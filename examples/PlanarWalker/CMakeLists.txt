# -*- mode: cmake -*-
# vi: set ft=cmake :

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
