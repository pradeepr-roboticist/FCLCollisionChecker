cmake_minimum_required (VERSION 3.5)
project (test_basic_collision)
set (CMAKE_CXX_STANDARD 14)

# list(APPEND CMAKE_MODULE_PATH "/home/pradeepr/Desktop/AVAStar/FCLCollisionChecker/dependencies/fcl_old/fcl/build/install/lib/cmake")
# list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../dependencies/octomap/octomap/build/install/share)

add_definitions(-DFCL_OLD)
# project
# find_package (ccd REQUIRED)
# find_package (fcl REQUIRED)
# find_package (octomap REQUIRED)

# list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../dependencies/octomap/octomap/build/install/share)

set(DEP ${CMAKE_CURRENT_LIST_DIR}/../dependencies)
set(FCL_INCLUDE_DIRS ${DEP}/fcl_old/fcl/build/install/include)
set(OCTOMAP_INCLUDE_DIRS ${DEP}/octomap/octomap/build/install/include)
set(fcl_LIBRARIES
${DEP}/fcl_old/fcl/build/install/lib/libfcl.a
)
set(OCTOMAP_LIBRARIES
${DEP}/octomap/octomap/build/install/lib/liboctomap.a
${DEP}/octomap/octomap/build/install/lib/liboctomath.a
)
set(CCD_LIBRARIES /usr/lib/x86_64-linux-gnu/libccd.so)

include_directories(../include ${FCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
add_executable(test_basic_collision test_basic_collision.cpp)
target_link_libraries(test_basic_collision ${fcl_LIBRARIES} ${OCTOMAP_LIBRARIES} ${CCD_LIBRARIES})

# macro(print_all_variables)
#     message(STATUS "print_all_variables------------------------------------------{")
#     get_cmake_property(_variableNames VARIABLES)
#     foreach (_variableName ${_variableNames})
#         message(STATUS "${_variableName}=${${_variableName}}")
#     endforeach()
#     message(STATUS "print_all_variables------------------------------------------}")
# endmacro()
# print_all_variables()