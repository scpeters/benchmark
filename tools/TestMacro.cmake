#################################################
# Hack: extra sources to build binaries can be supplied to gz_build_tests in
# the variable GZ_BUILD_TESTS_EXTRA_EXE_SRCS. This variable will be clean up
# at the end of the function
macro (gz_build_tests)
  # Build all the tests
  foreach(GTEST_SOURCE_file ${ARGN})
    string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
    set(BINARY_NAME ${TEST_TYPE}_${BINARY_NAME})
    
    # Check if the file exists
    if (EXISTS "${WORLDS_DIR_PATH}/${BINARY_NAME}")
          message(STATUS "${WORLDS_DIR_PATH}/${BINARY_NAME} exists!")
    else()
          execute_process(COMMAND mkdir ${WORLDS_DIR_PATH}/${BINARY_NAME})
          message(STATUS "${WORLDS_DIR_PATH}/${BINARY_NAME} created")
    endif()

    if (EXISTS "${TEST_RESULT_DIR}/${BINARY_NAME}")
          message(STATUS "${TEST_RESULT_DIR}/${BINARY_NAME}/MCAP exists!")
    else()
          execute_process(COMMAND mkdir ${TEST_RESULT_DIR}/${BINARY_NAME})
          execute_process(COMMAND mkdir ${TEST_RESULT_DIR}/${BINARY_NAME}/MCAP)

          message(STATUS "${TEST_RESULT_DIR}/${BINARY_NAME}/MCAP created")
    endif()
  
    add_executable(${BINARY_NAME} ${GTEST_SOURCE_file}
                   ${GZ_BUILD_TESTS_EXTRA_EXE_SRCS} ${PROTOBUF_DESCRIPTION_SRC} ${PROTO_SRCS} ${PROTO_HDRS})
                  
    target_include_directories(${BINARY_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
    target_include_directories(${BINARY_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/mcap/cpp/mcap/include)
    target_include_directories(${BINARY_NAME} PUBLIC ${CMAKE_SOURCE_DIR}/mcap/cpp/examples)
    


    target_link_libraries(${BINARY_NAME}
      gtest
      gtest_main
      gazebo_test_fixture
      ${GAZEBO_LIBRARIES}
      ${Boost_LIBRARIES}
      ${Protobuf_LIBRARIES}
      ${MCAP_DEPENDENCIES}
    )
    
    target_compile_definitions(${BINARY_NAME} PRIVATE TEST_NAME="${BINARY_NAME}")
    # target_compile_definitions(${BINARY_NAME} PRIVATE LOG_ALL= 1)

    add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME})

    set(_env_vars)
    list(APPEND _env_vars "GAZEBO_MODEL_PATH=${CMAKE_SOURCE_DIR}/models:${GAZEBO_MODEL_PATH}")
    # list(APPEND _env_vars "GAZEBO_RESOURCE_PATH=${CMAKE_SOURCE_DIR}:${GAZEBO_RESOURCE_PATH}")
    set_tests_properties(${BINARY_NAME} PROPERTIES
      TIMEOUT 240
      ENVIRONMENT "${_env_vars}"
    )

    # Check that the test produced a result and create a failure if it didn't.
    # Guards against crashed and timed out tests.
    # add_test(check_${BINARY_NAME} python3 ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py
	  #   ${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)

    # Convert junit file to csv and place in test_results in source folder.
    add_test(NAME csv_${BINARY_NAME}
      COMMAND python3 ${PROJECT_SOURCE_DIR}/tools/mcap_to_csv.py ${BINARY_NAME}
    )

    install(TARGETS ${BINARY_NAME}
      RUNTIME DESTINATION bin
    )
  endforeach()

  set(GZ_BUILD_TESTS_EXTRA_EXE_SRCS "")
endmacro()
