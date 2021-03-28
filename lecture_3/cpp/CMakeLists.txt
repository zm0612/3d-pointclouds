cmake_minimum_required(VERSION 3.10)

project(cluster)

set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

set(ALL_TARGET_LIBRARIES "")

include(cmake/pcl.cmake)
include(cmake/eigen.cmake)

include_directories(include)

add_library(DEP_LIB SHARED
    src/kmeans.cpp
    src/viewer.cpp
    src/GMM.cpp
)
target_link_libraries(DEP_LIB ${ALL_TARGET_LIBRARIES})

add_executable(test_kmeans test/test_kmeans.cpp)
target_link_libraries(test_kmeans DEP_LIB)

add_executable(test_GMM test/test_GMM.cpp)
target_link_libraries(test_GMM DEP_LIB)

#add_executable(test_PDF test/test_PDF.cpp)
#target_link_libraries(test_PDF DEP_LIB)
