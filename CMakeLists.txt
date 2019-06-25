cmake_minimum_required(VERSION 2.8.12)
project(test_fcl)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(fcl)
find_package(Eigen3)
# find_package(Boost)
find_package(octomap)
# set(EIGEN_ROOT /usr/include/eigen3)
# set(FCL_ROOT /home/pradeepr/Desktop/fcl)
# set(OCTOMAP_ROOT /home/pradeepr/Desktop/AVAStar/MOctomap/octomap)
include_directories(
    # ${EIGEN_ROOT}
    # ${FCL_ROOT}/include
    # ${FCL_ROOT}/build/include
    # ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    # ${OCTOMAP_ROOT}/octomap/include
    ${OCTOMAP_INCLUDE_DIRS}
    )

get_cmake_property(_variableNames VARIABLES)
foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
endforeach()

add_executable(fcl_test fcl_test.cpp)
target_link_libraries(fcl_test
    /usr/local/lib/libfcl.a
    # ${FCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    /usr/lib/x86_64-linux-gnu/libccd.so
    )
