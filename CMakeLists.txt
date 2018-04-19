# -*- mode: cmake -*-
# vi: set ft=cmake :

cmake_minimum_required(VERSION 3.5.1)
project(dircon)
set(DIRCON_VERSION 0.0.1)

# N.B. This is a temporary flag. It only really applies to Linux, as Mac
# does not need X11.
option(RUN_X11_TESTS "Run tests that require X11" OFF)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

include(CTest)

find_package(drake CONFIG REQUIRED)

find_package(PythonInterp 2.7 EXACT MODULE REQUIRED)
set(PYTHONPATH "${drake_PYTHON_DIRS}:$ENV{PYTHONPATH}")


# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/CMake/DIRCON)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# set up include-directories
include_directories(
  "${PROJECT_SOURCE_DIR}"
  "${PROJECT_BINARY_DIR}")

# Add sub-directories
add_subdirectory(examples)
add_subdirectory(systems)

# The interesting stuff goes here
# ===============================

# Add all targets to the build-tree export set
export(TARGETS dircon
  FILE "${PROJECT_BINARY_DIR}/DirconTargets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE DIRCON)

# Create the dirconConfig.cmake and dirconConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}")
configure_file(dirconConfig.cmake.in
  "${PROJECT_BINARY_DIR}/dirconConfig.cmake" @ONLY)
# ... for the install tree
set(CONF_INCLUDE_DIRS "\${DIRCON_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(dirconConfig.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/dirconConfig.cmake" @ONLY)
# ... for both
configure_file(dirconConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/dirconConfigVersion.cmake" @ONLY)

# Install the dirconConfig.cmake and dirconConfigVersion.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/dirconConfig.cmake"
  "${PROJECT_BINARY_DIR}/dirconConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT DirconTargets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)