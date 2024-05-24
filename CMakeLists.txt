cmake_minimum_required(VERSION 3.10)
project(cloud_segment)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -pthread")

set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(common_lib "")
list(APPEND common_lib ${PCL_LIBRARIES})

add_executable(euclidean_cluster test/euclidean_cluster_node.cpp src/euclidean_cluster.cpp)
target_link_libraries(euclidean_cluster ${common_lib})

add_executable(linefit_ground_segmentation test/linefit_ground_segmentation_node.cpp src/linefit_ground_segmentation.cpp)
target_link_libraries(linefit_ground_segmentation ${common_lib})

add_executable(ray_ground_filter test/ray_ground_filter_node.cpp src/ray_ground_filter.cpp)
target_link_libraries(ray_ground_filter ${common_lib})

add_executable(ring_ground_filter test/ring_ground_filter_node.cpp src/ring_ground_filter.cpp)
target_link_libraries(ring_ground_filter ${common_lib})