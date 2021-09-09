
if(QNX)
  set(foonathan_memory_INCLUDE_DIR ${ROS_EXTERNAL_DEPS_INSTALL}/usr/include;${ROS_EXTERNAL_DEPS_INSTALL})
  find_library(foonathan_memory_LIBRARY foonathan_memory HINTS ${ROS_EXTERNAL_DEPS_INSTALL}/${CPUVARDIR})
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(foonathan_memory  DEFAULT_MSG foonathan_memory_LIBRARY foonathan_memory_INCLUDE_DIR)
  mark_as_advanced(foonathan_memory_INCLUDE_DIR foonathan_memory_LIBRARY )
  set(foonathan_memory_LIBRARIES ${foonathan_memory_LIBRARY} )
  set(foonathan_memory_INCLUDE_DIRS ${foonathan_memory_INCLUDE_DIR} )
  set(foonathan_memory_FOUND TRUE)
endif()

