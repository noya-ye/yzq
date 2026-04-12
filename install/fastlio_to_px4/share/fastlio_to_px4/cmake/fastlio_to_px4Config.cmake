# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_fastlio_to_px4_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED fastlio_to_px4_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(fastlio_to_px4_FOUND FALSE)
  elseif(NOT fastlio_to_px4_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(fastlio_to_px4_FOUND FALSE)
  endif()
  return()
endif()
set(_fastlio_to_px4_CONFIG_INCLUDED TRUE)

# output package information
if(NOT fastlio_to_px4_FIND_QUIETLY)
  message(STATUS "Found fastlio_to_px4: 0.0.1 (${fastlio_to_px4_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'fastlio_to_px4' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${fastlio_to_px4_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(fastlio_to_px4_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${fastlio_to_px4_DIR}/${_extra}")
endforeach()
