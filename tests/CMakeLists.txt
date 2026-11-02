# Loads every module name from the module just built (the surfaces from the source tree, since they are only
# installed later) and checks the collision pairs, the mount surface and the floating-base variants
add_executable(test_openarm_modules test_openarm_modules.cpp)
target_link_libraries(test_openarm_modules PUBLIC mc_rtc::mc_rbdyn)
add_test(NAME test_openarm_modules COMMAND test_openarm_modules "$<TARGET_FILE_DIR:openarm>"
                                           "${PROJECT_SOURCE_DIR}/rsdf")
