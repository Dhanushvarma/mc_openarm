cmake_minimum_required(VERSION 3.21)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 17)

set(PROJECT_NAME mc_openarm)
set(PROJECT_DESCRIPTION "OpenArm robot module for mc_rtc")

project(
  ${PROJECT_NAME}
  LANGUAGES CXX
  VERSION 1.0.0)

find_package(mc_rtc REQUIRED)

find_description_package(openarm_description)
set(MC_OPENARM_DESCRIPTION_PATH "${OPENARM_DESCRIPTION_PATH}")

# Generate the URDFs from the xacro files of openarm_description
find_program(XACRO_EXECUTABLE xacro REQUIRED)
get_filename_component(OPENARM_DESCRIPTION_PREFIX
                       "${OPENARM_DESCRIPTION_PATH}/../.." ABSOLUTE)
set(MC_OPENARM_URDF_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/urdf")

function(generate_urdf name xacro_file)
  set(urdf_file "${CMAKE_CURRENT_BINARY_DIR}/${name}.urdf")
  add_custom_command(
    OUTPUT "${urdf_file}"
    COMMAND
      "${CMAKE_COMMAND}" -E env
      "AMENT_PREFIX_PATH=${OPENARM_DESCRIPTION_PREFIX}:$ENV{AMENT_PREFIX_PATH}"
      "${XACRO_EXECUTABLE}" "${OPENARM_DESCRIPTION_PATH}/${xacro_file}" ${ARGN}
      -o "${urdf_file}"
    DEPENDS "${OPENARM_DESCRIPTION_PATH}/${xacro_file}")
  add_custom_target(${name}_urdf ALL DEPENDS "${urdf_file}")
  install(FILES "${urdf_file}" DESTINATION "${MC_OPENARM_URDF_DIR}")
endfunction()

generate_urdf(openarm_v1 assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro
              bimanual:=true ros2_control:=false)
generate_urdf(openarm_v2 assets/robot/openarm_v2.0/urdf/openarm_v20.urdf.xacro
              robot_preset:=default_bimanual)

# Surfaces (rsdf/<robot name>/*.rsdf): the module points its rsdf_dir here
set(MC_OPENARM_RSDF_DIR "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/rsdf")
install(DIRECTORY rsdf/ DESTINATION "${MC_OPENARM_RSDF_DIR}")

add_subdirectory(src)

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
