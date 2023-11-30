cmake_minimum_required(VERSION 2.8.3)
project(inset_test_node)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")
get_filename_component(TEST_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
set(iris_DIR "${TEST_DIR}/../../../install/lib/cmake/iris")
set(mosek_DIR "${TEST_DIR}/../../../install/lib/cmake/iris")

find_package(iris REQUIRED)
find_package(mosek REQUIRED)
find_package(casadi REQUIRED)
find_package(catkin_simple REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(inset_util REQUIRED)
include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${INSET_UTIL_INCLUDE_DIRS}
    ${IRIS_INCLUDE_DIRS}
    ${MOSEK_INCLUDE_DIRS})
link_libraries("${IRIS_LIBRARY_DIRS}/libiris_geometry.so.0")
link_directories(${IRIS_LIBRARY_DIRS} ${INSET_UTIL_LIBRARIES})

catkin_simple()

include_directories(include)

cs_add_executable(test_inset src/test_inset.cpp)

target_link_libraries(test_inset iris inset)

cs_install()

cs_export()
