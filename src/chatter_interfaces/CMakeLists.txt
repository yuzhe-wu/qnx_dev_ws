cmake_minimum_required(VERSION 3.8)
project(chatter_interfaces)

# add_definitions(-U NPY_HAVE_ENDIAN_H -U NPY_HAVE_SYS_ENDIAN_H)

# set(CMAKE_LINKER /home/wayve/qnx710/host/linux/x86_64/usr/bin/aarch64-unknown-nto-qnx7.1.0-ld)
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fuse-ld=/home/wayve/qnx710/host/linux/x86_64/usr/bin/aarch64-unknown-nto-qnx7.1.0-ld")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rosidl_default_generators REQUIRED)

# include_directories(/home/wayve/qnx710/target/qnx7/aarch64le/opt/ros/humble/include/dds/ddsrt)


# include_directories(/home/wayve/qnx710/target/qnx7/usr/include/qh)

if(CMAKE_SYSTEM_NAME STREQUAL "QNX")
    add_definitions(-Dendian_h="<qh/endian.h>")
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Chatter.msg"
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
