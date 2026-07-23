set(lib_name ${LIB_TARGET_NAME}) # Override lets nested consumers avoid target name collisions.

# Add config.h file to build
configure_file(config.h.in config.h)

# Add src files to library
set(SRC_PATTERN "*.cpp" "*.cu")
file(GLOB srcLibFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${SRC_PATTERN})
file(GLOB srcCudaFilesToPTX "*.ptx.cu")
list(FILTER srcLibFiles EXCLUDE REGEX "\\.ptx\\.cu$")

# Get current folder name
get_filename_component(CURRENT_FOLDER_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message(STATUS "Library src files found in ${CURRENT_FOLDER_NAME}: ${srcLibFiles}")

# Add subdirectories with src files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/utils/CMakeLists.txt")
  message(STATUS "Adding module: utils")
  add_subdirectory(utils)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/attitude/CMakeLists.txt")
  message(STATUS "Adding module: attitude")
  add_subdirectory(attitude)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/wrapped_impl/CMakeLists.txt")
  message(STATUS "Adding module: wrapped_impl")
  add_subdirectory(wrapped_impl)
endif()

if(BUILD_AS_MAIN_PROJECT AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/bin/CMakeLists.txt")
  message(STATUS "Adding module: bin")
  add_subdirectory(bin)
endif()

# Exclude EXCLUDED_LIST from the list of src files
set(EXCLUDED_LIST "")
list(REMOVE_ITEM srcLibFiles ${EXCLUDED_LIST})

# Install headers from this directory
file(GLOB installable_headers RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.h" "*.cuh")
if(installable_headers)
  install(FILES ${installable_headers}
          DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${project_name})
endif()

list(LENGTH srcLibFiles src_lib_files_count)
if(src_lib_files_count EQUAL 0)
  set(lib_is_header_only TRUE)
  set(lib_link_scope INTERFACE)
  set(lib_target_type "INTERFACE")
  add_library(${lib_name} INTERFACE)
else()
  set(lib_is_header_only FALSE)
  set(lib_link_scope PUBLIC)
  if (BUILD_SHARED_LIBS)
    set(lib_target_type "SHARED")
  else()
    set(lib_target_type "STATIC")
  endif()
  add_library(${lib_name} ${lib_target_type}
              ${srcLibFiles})
  set_target_properties(${lib_name} PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

set_target_properties(${lib_name} PROPERTIES EXPORT_NAME ${project_name})
target_compile_features(${lib_name} ${lib_link_scope} cxx_std_20)

message(STATUS "Configured ${lib_name} as a ${lib_target_type} library target")
set(PROJECT_LIBRARY_KIND "${lib_target_type}" PARENT_SCOPE)

if(NOT TARGET ${LIB_NAMESPACE}::${project_name})
  add_library(${LIB_NAMESPACE}::${project_name} ALIAS ${lib_name})
endif()

if(NOT lib_is_header_only AND SANITIZE_BUILD AND SANITIZER_TARGET AND TARGET ${SANITIZER_TARGET})
  target_link_libraries(${lib_name} PRIVATE ${SANITIZER_TARGET})
endif()

if (ENABLE_OPTIX)
  if (lib_is_header_only)
    message(FATAL_ERROR
      "Internal configuration error: ENABLE_OPTIX passed the root preflight, "
      "but ${lib_name} resolved to a header-only target in src/CMakeLists.txt.")
  endif()

  if (NOT srcCudaFilesToPTX)
    message(FATAL_ERROR
      "Internal configuration error: ENABLE_OPTIX passed the root preflight, "
      "but no PTX kernel sources reached src/CMakeLists.txt.")
  endif()

  message("Library .cu kernels files to compile as PTX: ${srcCudaFilesToPTX}")

  get_target_property(project_interface_INCLUDE_DIRS ${project_name}_include INTERFACE_INCLUDE_DIRECTORIES)
  if (NOT project_interface_INCLUDE_DIRS)
    set(project_interface_INCLUDE_DIRS "")
  endif()

  set(PTX_INCLUDE_DIRS ${CUDAToolkit_INCLUDE_DIRS}
                       ${OptiX_INCLUDE_DIRS}
                       ${project_interface_INCLUDE_DIRS})

  cuda_compile_and_embed(embedded_ptx_code ${sm_version} "${PTX_INCLUDE_DIRS}" ${srcCudaFilesToPTX})
  target_sources(${lib_name} PRIVATE ${embedded_ptx_code})
endif()

target_link_libraries(${lib_name} ${lib_link_scope}
                      ${project_name}_include
                      ${LIB_COMPILE_TARGET}
                      Eigen3::Eigen
                      Threads::Threads)

if(ENABLE_CUDA)
  target_link_libraries(${lib_name} ${lib_link_scope} ${CUDA_COMPILE_TARGET})
endif()

if(ENABLE_OPTIX)
  target_link_libraries(${lib_name} ${lib_link_scope} ${OPTIX_COMPILE_TARGET})
endif()

if(ENABLE_OPENGL)
  target_link_libraries(${lib_name} ${lib_link_scope} ${OPENGL_COMPILE_TARGET})
endif()

if(NOT lib_is_header_only AND TARGET ${PROFILING_COMPILE_TARGET})
  if(PROFILING_PROPAGATES_TO_CONSUMERS)
    target_link_libraries(${lib_name} PUBLIC ${PROFILING_COMPILE_TARGET})
  else()
    target_link_libraries(${lib_name} PRIVATE ${PROFILING_COMPILE_TARGET})
  endif()
endif()

set(lib_name_INCLUDE_DIRS ${lib_name_INCLUDE_DIRS} PARENT_SCOPE)

set(installable_targets ${installable_targets}
                        ${lib_name}
                        ${project_name}_include
                        ${LIB_COMPILE_TARGET})
if(TARGET ${PROFILING_COMPILE_TARGET})
  list(APPEND installable_targets ${PROFILING_COMPILE_TARGET})
endif()
if(ENABLE_CUDA)
  list(APPEND installable_targets ${CUDA_COMPILE_TARGET})
  if(ENABLE_OPTIX)
    list(APPEND installable_targets ${OPTIX_COMPILE_TARGET})
  endif()
endif()
if(ENABLE_OPENGL)
  list(APPEND installable_targets ${OPENGL_COMPILE_TARGET})
endif()
set(installable_targets ${installable_targets} PARENT_SCOPE)
message(NOTICE "Added target to installable list: ${installable_targets}")

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/global_includes.h")
  install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/global_includes.h"
          DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${project_name})
endif()

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config.h"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${project_name})

include(CMakePackageConfigHelpers)

install(
  TARGETS ${installable_targets}
  EXPORT ${project_name}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT ${project_name}
        FILE "${project_name}Target.cmake"
        NAMESPACE ${project_name}::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${project_name})

write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/${project_name}ConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${project_name}Config.cmake.in"
  "${PROJECT_BINARY_DIR}/${project_name}Config.cmake"
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${project_name})

install(FILES
  "${PROJECT_BINARY_DIR}/${project_name}Config.cmake"
  "${PROJECT_BINARY_DIR}/${project_name}ConfigVersion.cmake"
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${project_name})

export(EXPORT ${project_name}
       FILE "${PROJECT_BINARY_DIR}/${project_name}Target.cmake"
       NAMESPACE ${project_name}::)
