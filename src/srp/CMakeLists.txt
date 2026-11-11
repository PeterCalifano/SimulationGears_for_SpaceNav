file(GLOB srp_sources CONFIGURE_DEPENDS "*.cpp")
set(srcLibFiles ${srcLibFiles} ${srp_sources} PARENT_SCOPE)
file(GLOB srp_headers "*.h")
install(FILES ${srp_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${NATIVE_LIBRARY_NAME}/srp)
