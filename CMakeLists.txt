cmake_minimum_required(VERSION 3.8)
project(eddie-ros)

# Default to C20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# system dependencies
find_package(soem REQUIRED)
find_package(robif2b REQUIRED)
find_package(coord2b REQUIRED)

# ros dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
  ${kdl_parser_INCLUDE_DIRS}
  ${robif2b_INCLUDE_DIRS}
  ${coord2b_INCLUDE_DIRS}
)

# interface node
add_executable(eddie_ros_interface
  src/interface.cpp
  src/interface-parameters.cpp
)

ament_target_dependencies(eddie_ros_interface
  rclcpp
  kdl_parser
  ament_index_cpp
)
target_link_libraries(eddie_ros_interface
  soem
  orocos-kdl
  robif2b::ethercat
  robif2b::kelo
  robif2b::kinova_gen3
  coord2b
)

# test node
add_executable(eddie_ros_interface_test
  src/interface-test.cpp
  src/interface-parameters.cpp
)

ament_target_dependencies(eddie_ros_interface_test
  rclcpp
  kdl_parser
  ament_index_cpp
)
target_link_libraries(eddie_ros_interface_test
  soem
  orocos-kdl
  robif2b::ethercat
  robif2b::kelo
  robif2b::kinova_gen3
  coord2b
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  TARGETS eddie_ros_interface eddie_ros_interface_test
  # EXPORT export_eddie_ros_interface
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

set_target_properties(eddie_ros_interface eddie_ros_interface_test
  PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
    INSTALL_RPATH_USE_LINK_PATH TRUE
    BUILD_WITH_INSTALL_RPATH TRUE
)

set(POST_INSTALL_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/set_interface_permissions.sh")

install(PROGRAMS ${POST_INSTALL_SCRIPT}
        DESTINATION lib/${PROJECT_NAME}
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                    GROUP_READ GROUP_EXECUTE
                    WORLD_READ WORLD_EXECUTE)

# install(
#   CODE "execute_process(COMMAND bash \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/set_interface_permissions.sh\")"
# )

ament_package()
