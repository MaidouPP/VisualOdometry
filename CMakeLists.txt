cmake_minimum_required( VERSION 2.8 )
project( visual_odometry )

# debug - debug - -... release - faster
#set( CMAKE_BUILD_TYPE Debug )
set( CMAKE_BUILD_TYPE Release )

# enable C11
 set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -march=native -O3 -pthread" )

# cv, eigen, pcl...
find_package( OpenCV REQUIRED )
find_package( Eigen3 REQUIRED )
#INCLUDE_DIRECTORIES ( "$ENV{EIGEN3_INCLUDE_DIR}" )
#SET( EIGEN3_INCLUDE_DIR "$ENV{EIGEN3_INCLUDE_DIR}" )
#IF( NOT EIGEN3_INCLUDE_DIR )
#    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
#ENDIF()

#INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

#add_definitions(${EIGEN3_DEFINITIONS})
#INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})
#LINK_DIRECTORIES(${EIGEN3_LIBRARY_DIRS})

# find_package( PCL 1.7 REQUIRED )

#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

# output -> bin
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
# library (.a) -> lib
# set lib directory
SET( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# headers -> include
include_directories(
    ${PROJECT_SOURCE_DIR}/
    ${PROJECT_SOURCE_DIR}/include/
    )

# source file -> src, experiment
add_subdirectory( ${PROJECT_SOURCE_DIR}/src/ )
add_subdirectory( ${PROJECT_SOURCE_DIR}/experiment/ )
