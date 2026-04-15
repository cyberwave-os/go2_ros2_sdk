cmake_minimum_required(VERSION 3.8)
project(lidar_processor_cpp)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(go2_interfaces REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED)
include(FetchContent)

FetchContent_Declare(
  wasm3
  GIT_REPOSITORY https://github.com/wasm3/wasm3.git
  GIT_TAG main
)
# Only fetch wasm3 sources; avoid add_subdirectory() target collisions.
FetchContent_GetProperties(wasm3)
if(NOT wasm3_POPULATED)
  FetchContent_Populate(wasm3)
endif()

file(GLOB WASM3_SOURCES "${wasm3_SOURCE_DIR}/source/*.c")
add_library(wasm3_static STATIC ${WASM3_SOURCES})
target_include_directories(wasm3_static PUBLIC ${wasm3_SOURCE_DIR}/source)

include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(dependencies
  ament_index_cpp
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  go2_interfaces
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
)

add_executable(lidar_to_pointcloud_node src/lidar_to_pointcloud_node.cpp)
ament_target_dependencies(lidar_to_pointcloud_node ${dependencies})
target_link_libraries(lidar_to_pointcloud_node ${PCL_LIBRARIES})

add_executable(pointcloud_aggregator_node src/pointcloud_aggregator_node.cpp)
ament_target_dependencies(pointcloud_aggregator_node ${dependencies})
target_link_libraries(pointcloud_aggregator_node ${PCL_LIBRARIES})

add_executable(voxel_decoder_node src/voxel_decoder_node.cpp)
target_include_directories(voxel_decoder_node PRIVATE ${wasm3_SOURCE_DIR}/source)
ament_target_dependencies(voxel_decoder_node ${dependencies})
target_link_libraries(voxel_decoder_node ${PCL_LIBRARIES} wasm3_static m)

install(TARGETS
  lidar_to_pointcloud_node
  pointcloud_aggregator_node
  voxel_decoder_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
