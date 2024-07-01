cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(benchmark_gazebo)
enable_testing()

include(FindBoost)
find_package(PkgConfig REQUIRED)
pkg_check_modules(zstd REQUIRED IMPORTED_TARGET libzstd)
pkg_check_modules(lz4 REQUIRED IMPORTED_TARGET liblz4)
find_package(protobuf CONFIG)
if(NOT Protobuf_FOUND)
  find_package(Protobuf REQUIRED)
endif()
if(NOT protobuf_PROTOC_EXE)
  set(protobuf_PROTOC_EXE "protobuf::protoc")
endif()


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
set(PROTO_FILES proto/boxes_msg.proto)
include(${PROJECT_SOURCE_DIR}/tools/TestMacro.cmake)
set(MCAP_DEPENDENCIES PkgConfig::lz4 PkgConfig::zstd)
set(WORLDS_DIR_PATH "${PROJECT_SOURCE_DIR}/worlds")
set(TEST_RESULT_DIR "${PROJECT_SOURCE_DIR}/test_results")
set(TEST_TYPE "BENCHMARK")
include_directories(${Protobuf_INCLUDE_DIRS})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
configure_file(${PROJECT_SOURCE_DIR}/PathConfig.h.in PathConfig.h)

# Boxes tests
set(BOXES_TEST_FILES
  boxes_dt.cc
  boxes_model_count.cc
)
set(GZ_BUILD_TESTS_EXTRA_EXE_SRCS
  boxes.cc
)
set(PROTOBUF_DESCRIPTION_DIR ${CMAKE_SOURCE_DIR}/mcap/cpp/examples/protobuf/)
set(PROTOBUF_DESCRIPTION_SRC 
    ${PROTOBUF_DESCRIPTION_DIR}/BuildFileDescriptorSet.cpp
)
gz_build_tests(${BOXES_TEST_FILES})

set_tests_properties(BENCHMARK_boxes_dt PROPERTIES TIMEOUT 500)
set_tests_properties(BENCHMARK_boxes_model_count PROPERTIES TIMEOUT 3000)

# # Collide sphere tests
# set(COLLIDE_SPHERES_TEST_FILES
#   collide_spheres_dt.cc
# )
# set(GZ_BUILD_TESTS_EXTRA_EXE_SRCS
#   collide_spheres.cc
# )
# gz_build_tests(${COLLIDE_SPHERES_TEST_FILES})
