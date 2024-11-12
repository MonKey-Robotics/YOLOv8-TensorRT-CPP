# YoloV8_TRTConfig.cmake
# This file will be installed into a location that other projects can find via `find_package`

# Set the location of the library and include directories
set(YoloV8_TRT_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
set(YoloV8_TRT_LIBRARIES "${CMAKE_INSTALL_PREFIX}/lib/libYoloV8_TRT.so")

# Provide CMake variables for the include and lib directories
if(NOT TARGET YoloV8_TRT)
  add_library(YoloV8_TRT SHARED IMPORTED)
  set_target_properties(YoloV8_TRT PROPERTIES
    IMPORTED_LOCATION ${YoloV8_TRT_LIBRARIES}
    INTERFACE_INCLUDE_DIRECTORIES ${YoloV8_TRT_INCLUDE_DIRS}
  )
endif()

# Optionally, define other variables if needed
set(YoloV8_TRT_VERSION "1.0.0")
