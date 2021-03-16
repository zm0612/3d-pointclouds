cmake_minimum_required(VERSION 3.10)

project(lecture_2)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(ALL_TARGET_LIBRARIES "")

include(cmake/pcl.cmake)
include(cmake/eigen.cmake)

include_directories(include)

add_library(DEPEND_LIB SHARED
    src/kdtree.cpp
    src/octree.cpp
    src/read_data.cpp
    src/pcl_viewer.cpp
    src/knn_result_set.cpp
    src/brute_force_search.cpp
)
target_link_libraries(DEPEND_LIB ${ALL_TARGET_LIBRARIES})

add_executable(kdtree_app app/kdtree_app.cpp)
target_link_libraries(kdtree_app DEPEND_LIB)

#add_executable(octree_app app/octree_app.cpp)
#target_link_libraries(octree_app DEPEND_LIB)

add_executable(read_data_app app/read_data_app.cpp)
target_link_libraries(read_data_app DEPEND_LIB)

add_executable(compare_my_to_pcl app/compare_my_to_pcl.cpp)
target_link_libraries(compare_my_to_pcl DEPEND_LIB)
