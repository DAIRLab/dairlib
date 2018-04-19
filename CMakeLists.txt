# -*- mode: cmake -*-
# vi: set ft=cmake :


cmake_minimum_required(VERSION 3.5.1)
project(goldilocks)


# N.B. This is a temporary flag. It only really applies to Linux, as Mac
# does not need X11.
option(RUN_X11_TESTS "Run tests that require X11" OFF)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/modules")

include(CTest)

find_package(drake CONFIG REQUIRED)

find_package(dircon CONFIG REQUIRED)
include_directories(${DIRCON_INCLUDE_DIRS})

find_package(PythonInterp 2.7 EXACT MODULE REQUIRED)
set(PYTHONPATH "${drake_PYTHON_DIRS}:$ENV{PYTHONPATH}")

add_subdirectory(src)
add_subdirectory(examples)