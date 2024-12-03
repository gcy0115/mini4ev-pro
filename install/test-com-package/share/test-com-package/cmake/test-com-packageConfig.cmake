# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_test-com-package_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED test-com-package_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(test-com-package_FOUND FALSE)
  elseif(NOT test-com-package_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(test-com-package_FOUND FALSE)
  endif()
  return()
endif()
set(_test-com-package_CONFIG_INCLUDED TRUE)

# output package information
if(NOT test-com-package_FIND_QUIETLY)
  message(STATUS "Found test-com-package: 0.0.0 (${test-com-package_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'test-com-package' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT test-com-package_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(test-com-package_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${test-com-package_DIR}/${_extra}")
endforeach()
