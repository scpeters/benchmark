cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(benchmark_gazebo)
enable_testing()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED filesystem thread system)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/gtest/include
  ${PROJECT_SOURCE_DIR}/gtest
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build gtest
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)

include (${PROJECT_SOURCE_DIR}/tools/TestMacro.cmake)
set(TEST_TYPE "BENCHMARK")

# Boxes tests
set(BOXES_TEST_FILES
  boxes_dt.cc
  boxes_model_count.cc
)
set(GZ_BUILD_TESTS_EXTRA_EXE_SRCS
  boxes.cc
)
gz_build_tests(${BOXES_TEST_FILES})
#foreach(BOXES_TEST_file ${BOXES_TEST_FILES})
#  string(REGEX REPLACE ".cc" "" BINARY_NAME ${BOXES_TEST_file})
#  set(BINARY_NAME ${TEST_PREFIX}_${BINARY_NAME})
#  add_executable(${BINARY_NAME}
#    ${BOXES_TEST_file}
#    boxes.cc
#  )
#  target_link_libraries(${BINARY_NAME}
#    gtest
#    gtest_main
#    gazebo_test_fixture
#    ${GAZEBO_LIBRARIES}
#    ${Boost_LIBRARIES}
#  )
#
#  add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
#    --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml
#  )
#  set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 5000)
#  # Check that the test produced a result and create a failure if it didn't.
#  # Guards against crashed and timed out tests.
#  add_test(check_${BINARY_NAME} ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py
#	  ${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
#
#  install(TARGETS ${BINARY_NAME}
#    RUNTIME DESTINATION bin
#  )
#endforeach()
