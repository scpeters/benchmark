#################################################
# Hack: extra sources to build binaries can be supplied to gz_build_tests in
# the variable GZ_BUILD_TESTS_EXTRA_EXE_SRCS. This variable will be clean up
# at the end of the function
macro (gz_build_tests)
  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})
    add_executable(${BINARY_NAME} ${GTEST_SOURCE_file}
                   ${GZ_BUILD_TESTS_EXTRA_EXE_SRCS})

    target_link_libraries(${BINARY_NAME}
      gtest
      gtest_main
      gazebo_test_fixture
      ${GAZEBO_LIBRARIES}
      ${Boost_LIBRARIES}
    )

    add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
	    --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

    set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 240)

    # Check that the test produced a result and create a failure if it didn't.
    # Guards against crashed and timed out tests.
    add_test(check_${BINARY_NAME} ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py
	    ${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

    install(TARGETS ${BINARY_NAME}
      RUNTIME DESTINATION bin
    )
  endforeach()

  set(GZ_BUILD_TESTS_EXTRA_EXE_SRCS "")
endmacro()
