set(installable_headers
    "${CMAKE_CURRENT_SOURCE_DIR}/CLogger.h")

file(RELATIVE_PATH relative_path_from_root
     "${PROJECT_SOURCE_DIR}"
     "${CMAKE_CURRENT_SOURCE_DIR}")
string(REPLACE "src/" "" relative_path_without_src "${relative_path_from_root}")

set(srcLibFiles_local
    "${CMAKE_CURRENT_SOURCE_DIR}/CLogger.cpp")

set(srcLibFiles ${srcLibFiles} ${srcLibFiles_local} PARENT_SCOPE)
set(srcCudaFilesToPTX ${srcCudaFilesToPTX} PARENT_SCOPE)

install(
  FILES ${installable_headers}
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${NATIVE_LIBRARY_NAME}/${relative_path_without_src})
