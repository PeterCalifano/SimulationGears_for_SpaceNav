# Register project-owned Python tests using the shared template test features.
add_tests(
  ${project_name}
  EXCLUDED_LIST
  TESTS_LIST
  ${CUDA_COMPILE_TARGET}
  CATCH2_TEST_PROPERTIES
  Catch2::Catch2WithMain
)

set(TESTS_LIST "${TESTS_LIST}" PARENT_SCOPE)
