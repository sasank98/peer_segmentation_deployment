cmake_minimum_required(VERSION 3.5)
project(pallet_detection_onnx)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ament_index_cpp REQUIRED)
# Set the path to the ONNX Runtime installation
# this is the onnxruntime path in Ultralytics container so the following line of code is changed
# wget https://nvidia.box.com/shared/static/48dtuob7meiw6ebgfsfqakc9vse62sg4.whl -O onnxruntime_gpu-1.18.0-cp310-cp310-linux_aarch64.whl
# pip install onnxruntime_gpu-1.18.0-cp310-cp310-linux_aarch64.whl

set(CMAKE_PREFIX_PATH "/usr/local/onnxruntime_gpu-1.18.0-cp310-cp310-linux_aarch64/lib/cmake/onnxruntime" ${CMAKE_PREFIX_PATH})

find_package(onnxruntime REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${onnxruntime_INCLUDE_DIRS}
)

add_executable(pallet_detection_node src/pallet_onnx.cpp)

ament_target_dependencies(pallet_detection_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  onnxruntime
)

install(TARGETS
  pallet_detection_node
  DESTINATION lib/${PROJECT_NAME}
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