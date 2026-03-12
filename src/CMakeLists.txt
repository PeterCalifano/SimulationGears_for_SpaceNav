set(lib_name ${project_name}) # Default name for the library is project_name

# Add config.h file to build
configure_file(config.h.in config.h)

# Add src files to library
set(SRC_PATTERN "*.cpp; *.cu")
file(GLOB srcLibFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${SRC_PATTERN})
get_filename_component(CURRENT_FOLDER_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message(STATUS "Library src files found in ${CURRENT_FOLDER_NAME}: ${srcLibFiles}")

if (ENABLE_CUDA AND ENABLE_OPTIX)
  file(GLOB srcCudaFilesToPTX RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "kernels/*.cu")
endif()

# Define INCLUDE_DIRECTORIES (subdirectories append to this)
set(lib_name_INCLUDE_DIRS ${lib_name_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

# Add subdirectories with src files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/template_folder/CMakeLists.txt")
  message(STATUS "Adding module: template_folder")
  add_subdirectory(template_folder)
endif()

# Exclude EXCLUDED_LIST from the list of src files
set(EXCLUDED_LIST "")
list(REMOVE_ITEM srcLibFiles ${EXCLUDED_LIST})

# Install headers from this directory
file(GLOB installable_headers "*.h")
install(FILES ${installable_headers}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${project_name})

list(LENGTH srcLibFiles src_lib_files_count)
if (src_lib_files_count EQUAL 0)
  set(lib_is_header_only TRUE)
  set(lib_link_scope INTERFACE)
  set(lib_target_type "INTERFACE")
  add_library(${lib_name} INTERFACE)
else()
  set(lib_is_header_only FALSE)
  set(lib_link_scope PUBLIC)
  set(lib_target_type "SHARED")
  if (DEFINED BUILD_SHARED_LIBS AND NOT BUILD_SHARED_LIBS)
    set(lib_target_type "STATIC")
  endif()
  add_library(${lib_name} ${lib_target_type}
              ${srcLibFiles})
  set_target_properties(${lib_name} PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

message(STATUS "Configured ${lib_name} as a ${lib_target_type} library target")
set(PROJECT_LIBRARY_KIND "${lib_target_type}" PARENT_SCOPE)

if (SPDLOG_ENABLED)
  target_compile_definitions(${lib_name} ${lib_link_scope} SPDLOG_UTILS_ENABLED=1)
else()
  target_compile_definitions(${lib_name} ${lib_link_scope} SPDLOG_UTILS_ENABLED=0)
endif()

target_link_libraries(${lib_name} ${lib_link_scope}
                      ${project_name}_include
                      ${LIB_COMPILE_TARGET}
                      Eigen3::Eigen)
target_include_directories(${lib_name} ${lib_link_scope} ${lib_name_INCLUDE_DIRS})

if (ENABLE_CUDA)
  target_link_libraries(${lib_name} ${lib_link_scope} ${CUDA_COMPILE_TARGET})
endif()

if(ENABLE_OPTIX)
  if (lib_is_header_only)
    message(FATAL_ERROR "ENABLE_OPTIX requires at least one compilable source file in src/ to embed PTX code into ${lib_name}.")
  endif()

  message("Library .cu kernels files to compile as PTX: ${srcCudaFilesToPTX}")
  cuda_compile_and_embed(embedded_ptx_code ${srcCudaFilesToPTX})
  target_sources(${lib_name} PRIVATE ${embedded_ptx_code})
  target_link_libraries(${lib_name} ${lib_link_scope} ${OPTIX_COMPILE_TARGET})
endif()

if (ENABLE_OPENGL)
  target_link_libraries(${lib_name} ${lib_link_scope} ${OPENGL_COMPILE_TARGET})
endif()

if (NOT lib_is_header_only AND TARGET ${PROFILING_COMPILE_TARGET})
  target_link_libraries(${lib_name} PRIVATE ${PROFILING_COMPILE_TARGET})
endif()

set(lib_name_INCLUDE_DIRS ${lib_name_INCLUDE_DIRS} PARENT_SCOPE)

set(installable_targets ${installable_targets}
                        ${lib_name}
                        ${project_name}_include
                        ${LIB_COMPILE_TARGET})
if (ENABLE_CUDA)
  list(APPEND installable_targets ${CUDA_COMPILE_TARGET})
  if (ENABLE_OPTIX)
    list(APPEND installable_targets ${OPTIX_COMPILE_TARGET})
  endif()
endif()
if (ENABLE_OPENGL)
  list(APPEND installable_targets ${OPENGL_COMPILE_TARGET})
endif()
set(installable_targets ${installable_targets} PARENT_SCOPE)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config.h"
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${project_name})

install(
	TARGETS ${installable_targets}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
