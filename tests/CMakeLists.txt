# Basic usage of catch2 for build config:
# These tests can use the Catch2-provided main
#add_executable(tests test.cpp)
#target_link_libraries(tests PRIVATE Catch2::Catch2WithMain)

# These tests need their own main
#add_executable(custom-main-tests test.cpp test-main.cpp)
#target_link_libraries(custom-main-tests PRIVATE Catch2::Catch2)

# Set additional properties for Catch2 test targets
set(CATCH2_TEST_PROPERTIES "--output-on-failure;--reporter=compact")
include(CTest)

# Exclude EXCLUDED_LIST from the list of tests
set(EXCLUDED_LIST "test_to_exclude")
set(TESTS_LIST "")

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add tests to build
if(Catch2_FOUND)
  add_tests(${project_name} EXCLUDED_LIST TESTS_LIST ${CUDA_COMPILE_TARGET} CATCH2_TEST_PROPERTIES Catch2::Catch2WithMain)
else()
  message(STATUS "Catch2 not found. Skipping Catch2 unit tests.")
endif()

# Register project-owned Python tests through the shared template helper.
if(ENABLE_PYTHON_TESTS)
  add_subdirectory(python)
endif()

message(STATUS "List of test targets: ${TESTS_LIST}")
