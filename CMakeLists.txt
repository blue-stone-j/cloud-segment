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

# generate node for test
add_executable(euclidean_cluster_node test/euclidean_cluster_node.cpp src/euclidean_cluster.cpp)
target_link_libraries(euclidean_cluster_node ${common_lib})

add_executable(linefit_ground_segmentation_node test/linefit_ground_segmentation_node.cpp src/linefit_ground_segmentation.cpp)
target_link_libraries(linefit_ground_segmentation_node ${common_lib})

add_executable(ray_ground_filter_node test/ray_ground_filter_node.cpp src/ray_ground_filter.cpp)
target_compile_options(ray_ground_filter_node PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(ray_ground_filter_node ${common_lib} ${OpenMP_CXX_FLAGS})

add_executable(ring_ground_filter_node test/ring_ground_filter_node.cpp src/ring_ground_filter.cpp)
target_link_libraries(ring_ground_filter_node ${common_lib})

add_executable(patchworkpp_node test/patchworkpp_node.cpp src/patchworkpp.cpp)
target_compile_options(patchworkpp_node PRIVATE

  # some varibles never be used, compiler will throw warn. This can depress these warns.
  $<$<CXX_COMPILER_ID:GNU>:-Wno-unused-variable>
  $<$<CXX_COMPILER_ID:Clang>:-Wno-unused-variable>

  $<$<CXX_COMPILER_ID:GNU>:-Wno-unused-but-set-variable>
  $<$<CXX_COMPILER_ID:Clang>:-Wno-unused-but-set-variable>
)
target_link_libraries(patchworkpp_node ${common_lib})

add_executable(cylinder_node test/cylinder_node.cpp src/cylinder.cpp)
target_link_libraries(cylinder_node ${common_lib})

# generate lib and install
add_library(euclidean_cluster STATIC src/euclidean_cluster.cpp)
target_link_libraries(euclidean_cluster ${common_lib})

add_library(linefit_ground_segmentation STATIC src/linefit_ground_segmentation.cpp)
target_link_libraries(linefit_ground_segmentation ${common_lib})

add_library(ray_ground_filter STATIC src/ray_ground_filter.cpp)
target_compile_options(ray_ground_filter PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(ray_ground_filter ${common_lib} ${OpenMP_CXX_FLAGS})

add_library(ring_ground_filter STATIC src/ring_ground_filter.cpp)
target_link_libraries(ring_ground_filter ${common_lib})

add_library(patchworkpp src/patchworkpp.cpp)
target_compile_options(patchworkpp PRIVATE

  # some varibles never be used, compiler will throw warn. This can depress these warns.
  $<$<CXX_COMPILER_ID:GNU>:-Wno-unused-variable>
  $<$<CXX_COMPILER_ID:Clang>:-Wno-unused-variable>

  $<$<CXX_COMPILER_ID:GNU>:-Wno-unused-but-set-variable>
  $<$<CXX_COMPILER_ID:Clang>:-Wno-unused-but-set-variable>
)
target_link_libraries(patchworkpp ${common_lib})

add_library(cylinder src/cylinder.cpp)
target_link_libraries(cylinder ${common_lib})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install
  PATTERN "*.h")
install(TARGETS euclidean_cluster linefit_ground_segmentation ray_ground_filter ring_ground_filter patchworkpp cylinder
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install)