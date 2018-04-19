# -*- mode: cmake -*-
# vi: set ft=cmake :

#set(CMAKE_BUILD_TYPE Debug)

find_package(GFlags MODULE REQUIRED COMPONENTS shared)

add_library(dircon dircon_options.cc  dircon.cc
         dircon_opt_constraints.cc dircon_kinematic_data_set.cc 
        dircon_kinematic_data.cc  dircon_position_data.cc 
         hybrid_dircon.cc dircon_util.cc)
target_link_libraries(dircon drake::drake)

set_target_properties(dircon PROPERTIES
  PUBLIC_HEADER "dircon_options.h;dircon.h;dircon_opt_constraints.h;dircon_kinematic_data_set.h;
  dircon_kinematic_data.h;dircon_position_data.h;hybrid_dircon.h;dircon_util.h")

#target_include_directories(dircon PUBLIC ${CMAKE_SOURCE_DIR})

#add_executable(kinematic_constraints_test kinematic_constraints_test.cc)
#target_include_directories(kinematic_constraints_test PRIVATE
#"${CMAKE_CURRENT_SOURCE_DIR}"
#)
#target_link_libraries(kinematic_constraints_test
#dircon drake::drake drake::drake-common-text-logging-gflags gflags_shared
#)

add_executable(dircon_test dircon_test.cc)
target_include_directories(dircon_test PRIVATE
"${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(dircon_test
 dircon drake::drake drake::drake-common-text-logging-gflags gflags_shared
)


add_executable(test_util test_util.cc)
target_include_directories(test_util PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
        )
target_link_libraries(test_util
        dircon drake::drake drake::drake-common-text-logging-gflags gflags_shared
        )


#add_executable(run_passive run_passive.cc)
#target_link_libraries(run_passive 
#  drake::drake drake::drake-common-text-logging-gflags gflags_shared
#)


install(TARGETS dircon
  EXPORT DirconTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/systems/trajectory_optimization/"
    COMPONENT dev)