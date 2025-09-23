# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_sdv_msgs_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED sdv_msgs_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(sdv_msgs_FOUND FALSE)
  elseif(NOT sdv_msgs_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(sdv_msgs_FOUND FALSE)
  endif()
  return()
endif()
set(_sdv_msgs_CONFIG_INCLUDED TRUE)

# output package information
if(NOT sdv_msgs_FIND_QUIETLY)
  message(STATUS "Found sdv_msgs: 0.0.0 (${sdv_msgs_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'sdv_msgs' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT sdv_msgs_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(sdv_msgs_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${sdv_msgs_DIR}/${_extra}")
endforeach()
