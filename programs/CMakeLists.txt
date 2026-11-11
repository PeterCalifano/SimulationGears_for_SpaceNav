add_executable(srp_lut srp_lut.cpp)
target_link_libraries(srp_lut PRIVATE ${LIB_TARGET_NAME})
install(TARGETS srp_lut RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
