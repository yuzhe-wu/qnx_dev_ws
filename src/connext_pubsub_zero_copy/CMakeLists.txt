cmake_minimum_required(VERSION 3.8)
project(connext_pubsub_zero_copy)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DDEFAULT_RMW_IMPLEMENTATION=rmw_connextdds)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_dds REQUIRED)
find_package(builtin_interfaces REQUIRED)


find_package(yaml-cpp REQUIRED)  # Ensure yaml-cpp is found
link_directories(/home/wayve/yuzhe/ros2_qnx_workspace/ros2/install/aarch64le/lib)

include_directories("/home/wayve/qnx710/target/qnx7/aarch64le/opt/ros/humble/include/console_bridge_vendor")

# Specify the paths for the metp library and headers
set(metp_INCLUDE_DIR "/home/wayve/rti_connext_dds-7.3.0/include/ndds/metp")
set(metp_LIBRARY "/home/wayve/rti_connext_dds-7.3.0/lib/armv8QNX7.1qcc_gpp8.3.0/libnddsmetp.so")

if(NOT TARGET RTIConnextDDS::metp)
    add_library(RTIConnextDDS::metp UNKNOWN IMPORTED)
    set_property(TARGET RTIConnextDDS::metp PROPERTY IMPORTED_LOCATION ${metp_LIBRARY})
    set_property(TARGET RTIConnextDDS::metp PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${metp_INCLUDE_DIR})
endif()

################################################################################
# Generate type supports with `rtiddsgen` from IDL files generated by ROS 2
# and build a library with all type supports.
# We also generate some type support from "standard" IDL files. In this case we
# use the syntax `<file>[@<prefix>]` to specify an optional prefix for the
# location where files will be generated (and that will be used to `#include`
# them in code, e.g. `#include "my/custom/prefix/Foo.hpp"`). This option is
# useful to use an "include policy" for type definition files that is consistent
# with the one used by ROS 2 in the automatically generated IDL files.
################################################################################
connext_generate_typesupport_library(${PROJECT_NAME}_types
  IDLS
    idl/Chatter.idl
  ZEROCOPY)

add_library(${PROJECT_NAME} SHARED
  src/chatter_publisher.cpp
  src/chatter_subscriber.cpp
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "RCLCPP_DDS_EXAMPLES_BUILDING_DLL")
target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

# Add RTI Connext DDS libraries
find_library(NDDS_CPP_LIBRARY libnddscpp.so PATHS /home/wayve/rti_connext_dds-7.3.0/lib/armv8QNX7.1qcc_gpp8.3.0)

if(NDDS_CPP_LIBRARY)
    message(STATUS "NDDS_CPP_LIBRARY found: ${NDDS_CPP_LIBRARY}")
else()
    message(FATAL_ERROR "NDDS_CPP_LIBRARY not found!")
endif()

find_library(NDDSC_LIBRARY libnddsc.so PATHS /home/wayve/rti_connext_dds-7.3.0/lib/armv8QNX7.1qcc_gpp8.3.0)
if(NDDSC_LIBRARY)
    message(STATUS "NDDSC_LIBRARY found: ${NDDSC_LIBRARY}")
else()
    message(FATAL_ERROR "NDDSC_LIBRARY not found!")
endif()

find_library(NDDSCORE_LIBRARY libnddscore.so PATHS /home/wayve/rti_connext_dds-7.3.0/lib/armv8QNX7.1qcc_gpp8.3.0)
if(NDDSCORE_LIBRARY)
    message(STATUS "NDDSCORE_LIBRARY found: ${NDDSCORE_LIBRARY}")
else()
    message(FATAL_ERROR "NDDSCORE_LIBRARY not found!")
endif()

find_library(NDDSMETP_LIBRARY libnddsmetp.so PATHS /home/wayve/rti_connext_dds-7.3.0/lib/armv8QNX7.1qcc_gpp8.3.0)
if(NDDSMETP_LIBRARY)
    message(STATUS "NDDSMETP_LIBRARY found: ${NDDSMETP_LIBRARY}")
else()
    message(FATAL_ERROR "NDDSMETP_LIBRARY not found!")
endif()

target_link_directories(${PROJECT_NAME} PRIVATE /home/wayve/rti_connext_dds-7.3.0/lib/armv8QNX7.1qcc_gpp8.3.0)

target_link_libraries(${PROJECT_NAME}
  ${NDDSCORE_LIBRARY}
  ${NDDSC_LIBRARY}
  ${NDDS_CPP_LIBRARY}
  ${NDDSMETP_LIBRARY}
  ${PROJECT_NAME}_types
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp_dds
  rclcpp_components
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

################################################################################
# Build executables for each component using a custom (but similar) macro
################################################################################
connext_components_register_node(${PROJECT_NAME}
  PLUGIN "MinimalPublisher"
  EXECUTABLE talker)
connext_components_register_node(${PROJECT_NAME}
  PLUGIN "rti::connext_nodes_cpp::DdsListener"
  EXECUTABLE listener)

connext_add_executable(
  NAME talker_main
  SOURCES
    src/chatter_publisher_main.cpp
  LIBRARIES
    ${PROJECT_NAME}
    ${NDDSCORE_LIBRARY}
    ${NDDSC_LIBRARY}
    ${NDDS_CPP_LIBRARY}
    ${NDDSMETP_LIBRARY}
)

connext_add_executable(
  NAME listener_main
  SOURCES
    src/chatter_subscriber_main.cpp
  LIBRARIES
    ${PROJECT_NAME}
    ${NDDSCORE_LIBRARY}
    ${NDDSC_LIBRARY}
    ${NDDS_CPP_LIBRARY}
    ${NDDSMETP_LIBRARY}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
