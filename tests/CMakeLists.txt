# Basic usage of catch2 for build config:
# These tests can use the Catch2-provided main
#add_executable(tests test.cpp)
#target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)

# These tests need their own main
#add_executable(custom-main-tests test.cpp test-main.cpp)
#target_link_libraries(custom-main-tests PRIVATE Catch2::Catch2)

# Set additional properties for Catch2 test targets
set(CATCH2_TEST_PROPERTIES "--output-on-failure;--reporter=compact")

# Exclude EXCLUDED_LIST from the list of tests
set(EXCLUDED_LIST "test_to_exclude")
set(TESTS_LIST "")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add subdirectories
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/template_test/CMakeLists.txt")
  add_subdirectory(template_test)
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/template_fixtures/CMakeLists.txt")
  add_subdirectory(template_fixtures)
endif()

# Add tests to build
add_tests(${project_name} EXCLUDED_LIST TESTS_LIST ${CUDA_COMPILE_TARGET} CATCH2_TEST_PROPERTIES Catch2::Catch2WithMain)

message(STATUS "List of test targets: ${TESTS_LIST}")
