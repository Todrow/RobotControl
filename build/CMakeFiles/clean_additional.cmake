# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "Release")
  file(REMOVE_RECURSE
  [[CMakeFiles\control_autogen.dir\AutogenUsed.txt]]
  [[CMakeFiles\control_autogen.dir\ParseCache.txt]]
  "control_autogen"
  )
endif()
