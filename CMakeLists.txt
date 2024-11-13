cmake_minimum_required(VERSION 3.22)
project(yolo_tensor_cpp)

# Use ccache to speed up rebuilds
include(cmake/ccache.cmake)

# Set C++ version and optimization level
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Ofast -DNDEBUG -Wno-deprecated-declarations")

# CMake Options
option(ENABLE_BENCHMARKS "Benchmark the preprocessing, inference, and post processing" OFF)
if (ENABLE_BENCHMARKS)
    add_compile_definitions(ENABLE_BENCHMARKS)
endif()

find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# find_package(hand_interfaces REQUIRED)
# Specify that we are using ROS 2
find_package(ament_cmake REQUIRED)

# TODO: Specify the path to TensorRT root dir
set(TensorRT_DIR /home/cyrus/work/libs/TensorRT-10.0.0.6/)

include_directories(${CMAKE_SOURCE_DIR}/include)

# Build the TensorRT inference engine library
# TensorRT is found and linked by the tensorrt-cpp-api
add_subdirectory(libs/tensorrt-cpp-api)

# Build the YoloV8 library
add_library(YoloV8_TRT SHARED src/yolov8.cpp)
target_link_libraries(YoloV8_TRT PUBLIC tensorrt_cpp_api ${OpenCV_LIBS})
target_include_directories(YoloV8_TRT PUBLIC libs/tensorrt-cpp-api/src)

# # Build and link the executables
# add_executable(detect_object_image src/object_detection_image.cpp)
# target_link_libraries(detect_object_image YoloV8_TRT)

# add_executable(detect_object_video src/object_detection_video_stream.cpp)
# target_link_libraries(detect_object_video YoloV8_TRT)

add_executable(trunk_detection src/trunk_detection.cpp)
target_link_libraries(trunk_detection YoloV8_TRT)
ament_target_dependencies(trunk_detection rclcpp rclcpp_components sensor_msgs cv_bridge)


install(TARGETS YoloV8_TRT
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files to share with other packages
install(DIRECTORY include/
        DESTINATION include/)


install(TARGETS   
# detect_object_image
trunk_detection
# detect_object_video
  YoloV8_TRT
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(YoloV8_TRT)
# ament_export_libraries(${LIBRARY_NAME})
ament_export_dependencies(${THIS_PACKAGE_DEPS})
ament_package()