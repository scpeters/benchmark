cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(benchmark_gazebo)
enable_testing()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED filesystem thread system)

find_package(GAZEBO 11)
if (NOT GAZEBO_FOUND)
  message (STATUS "Looking for gazebo - not found")
  message (FATAL_ERROR "Missing: Gazebo version 11.")
endif()
include_directories(${GAZEBO_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/gtest/include
  ${PROJECT_SOURCE_DIR}/gtest
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build gtest
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest tbb)

include (${PROJECT_SOURCE_DIR}/tools/TestMacro.cmake)
set(WORLDS_DIR_PATH "${PROJECT_SOURCE_DIR}/worlds")
set(TEST_TYPE "BENCHMARK")
configure_file(${PROJECT_SOURCE_DIR}/PathConfig.h.in PathConfig.h)

# Boxes tests
set(BOXES_TEST_FILES
  boxes_dt.cc
  boxes_model_count.cc
)
set(GZ_BUILD_TESTS_EXTRA_EXE_SRCS
  boxes.cc
)
gz_build_tests(${BOXES_TEST_FILES})

set_tests_properties(BENCHMARK_boxes_dt PROPERTIES TIMEOUT 500)
set_tests_properties(BENCHMARK_boxes_model_count PROPERTIES TIMEOUT 3000)

# Collide sphere tests
set(COLLIDE_SPHERES_TEST_FILES
  collide_spheres_dt.cc
)
set(GZ_BUILD_TESTS_EXTRA_EXE_SRCS
  collide_spheres.cc
)
gz_build_tests(${COLLIDE_SPHERES_TEST_FILES})

#Stack boxes test
set(BOXES_TEST_FILES_
  stack_boxes_inertial_test.cc
)

set(GZ_BUILD_TESTS_EXTRA_EXE_SRCS
  stack_boxes.cc
)

gz_build_tests(${BOXES_TEST_FILES_})
set_tests_properties(BENCHMARK_stack_boxes_inertial_test PROPERTIES TIMEOUT 500)
