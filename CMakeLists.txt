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
add_executable(euclidean_cluster_test test/euclidean_cluster_test.cpp src/euclidean_cluster.cpp)
target_link_libraries(euclidean_cluster_test ${common_lib})

add_executable(line_fit_ground_segmentation_test test/line_fit_ground_segmentation_test.cpp src/line_fit_ground_segmentation.cpp)
target_link_libraries(line_fit_ground_segmentation_test ${common_lib})

add_executable(ray_ground_filter_test test/ray_ground_filter_test.cpp src/ray_ground_filter.cpp)
target_compile_options(ray_ground_filter_test PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(ray_ground_filter_test ${common_lib} ${OpenMP_CXX_FLAGS})

add_executable(ring_ground_filter_test test/ring_ground_filter_test.cpp src/ring_ground_filter.cpp)
target_link_libraries(ring_ground_filter_test ${common_lib})

add_executable(patchworkpp_test test/patchworkpp_test.cpp src/patchworkpp.cpp)
target_compile_options(patchworkpp_test PRIVATE)
target_link_libraries(patchworkpp_test ${common_lib})

add_executable(cylinder_test test/cylinder_test.cpp src/cylinder.cpp)
target_link_libraries(cylinder_test ${common_lib})

# generate lib and install
add_library(euclidean_cluster STATIC src/euclidean_cluster.cpp)
target_link_libraries(euclidean_cluster ${common_lib})

add_library(line_fit_ground_segmentation STATIC src/line_fit_ground_segmentation.cpp)
target_link_libraries(line_fit_ground_segmentation ${common_lib})

add_library(ray_ground_filter STATIC src/ray_ground_filter.cpp)
target_compile_options(ray_ground_filter PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(ray_ground_filter ${common_lib} ${OpenMP_CXX_FLAGS})

add_library(ring_ground_filter STATIC src/ring_ground_filter.cpp)
target_link_libraries(ring_ground_filter ${common_lib})

add_library(patchworkpp src/patchworkpp.cpp)
target_compile_options(patchworkpp PRIVATE)
target_link_libraries(patchworkpp ${common_lib})

add_library(cylinder src/cylinder.cpp)
target_link_libraries(cylinder ${common_lib})

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install
  PATTERN "*.h")
install(TARGETS euclidean_cluster line_fit_ground_segmentation ray_ground_filter ring_ground_filter patchworkpp cylinder
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/install)