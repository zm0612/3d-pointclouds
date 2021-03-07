cmake_minimum_required(VERSION 3.10)

project(lecture_1)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

set(ALL_TARGET_LIBRARIES "")

include(cmake/eigen.cmake)
include(cmake/pcl.cmake)

include_directories(include)

add_library(DEP SHARED
    src/model_net_tool.cpp
    src/pcl_viewer.cpp
    src/principle_component_analysis.cpp
    src/surface_normal_estimation.cpp
    src/voxel_filter.cpp
)
target_link_libraries(DEP ${ALL_TARGET_LIBRARIES})

#add_executable(test_main app/test.cpp)
#target_link_libraries(test_main DEP)

####### TEST ########
add_executable(test_read_model_data test/test_read_model_data.cpp)
target_link_libraries(test_read_model_data DEP)

add_executable(test_pca test/test_pca.cpp)
target_link_libraries(test_pca DEP)

add_executable(test_normal_vector test/test_normal_vector.cpp)
target_link_libraries(test_normal_vector DEP)

add_executable(test_voxel test/test_voxel.cpp)
target_link_libraries(test_voxel DEP)