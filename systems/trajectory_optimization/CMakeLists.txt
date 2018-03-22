# -*- mode: cmake -*-
# vi: set ft=cmake :


find_package(GFlags MODULE REQUIRED COMPONENTS shared)

add_library(dircon_kinematic_constraint dircon_kinematic_constraint.cc dircon_kinematic_constraint.h dircon_position_constraint.cc dircon_position_constraint.h)
target_link_libraries(dircon_kinematic_constraint drake::drake)

add_library(dircon dircon_options.cc dircon_options.h)
target_link_libraries(dircon drake::drake)

add_executable(kinematic_constraints_test kinematic_constraints_test.cc)
target_include_directories(kinematic_constraints_test PRIVATE
"${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(kinematic_constraints_test
dircon_kinematic_constraint drake::drake drake::drake-common-text-logging-gflags gflags_shared
)


#add_executable(run_passive run_passive.cc)
#target_link_libraries(run_passive 
#  drake::drake drake::drake-common-text-logging-gflags gflags_shared
#)
