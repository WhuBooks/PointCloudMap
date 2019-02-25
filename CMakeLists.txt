cmake_minimum_required(VERSION 3.5)
project(PointCloudMap)

set(CMAKE_CXX_STANDARD 11)

# output paths settings
set(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "postfix applied to debug build of libraries")
set(CMAKE_RELEASE_POSTFIX "" CACHE STRING "postfix applied to release build of libraries")

# work out the postfix; required where we use OUTPUT_NAME
if(CMAKE_BUILD_TYPE MATCHES Release)
    set(EXE_POSTFIX)
elseif(CMAKE_BUILD_TYPE MATCHES Debug)
    set(EXE_POSTFIX ${CMAKE_DEBUG_POSTFIX})
endif(CMAKE_BUILD_TYPE MATCHES Release)

if(WIN32)
    MESSAGE(STATUS "Compiling for Windows")
elseif(ANDROID)
    MESSAGE(STATUS "Compiling for Android")
elseif(APPLE)
    MESSAGE(STATUS "Compiling for OSX")
elseif(UNIX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
    MESSAGE(STATUS "Compiling for Unix")
else()
    MESSAGE(STATUS "Platform can't be known")
endif(WIN32)

# binary directory
if(NOT WIN32)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
else()
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR})
endif()

# library directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

MESSAGE(STATUS "Executable Output Path : ${EXECUTABLE_OUTPUT_PATH}")
MESSAGE(STATUS "Library Output Path : ${LIBRARY_OUTPUT_PATH}")

# Eigen
include_directories( "/usr/include/eigen3" )

## Sophus
#find_package(Sophus REQUIRED)
#include_directories(${Sophus_INCLUDE_DIRS})
#link_libraries(${Sophus_LIBRARIES})
#message(STATUS "Sophus Dir : ${Sophus_INCLUDE_DIRS}")
#message(STATUS "Sophus Lib : ${Sophus_LIBRARIES}")

# libLas
find_package(libLAS REQUIRED)
include_directories(${libLAS_INCLUDE_DIRS})
link_directories(${libLAS_LIBRARY_DIRS})
link_libraries(${libLAS_LIBRARIES})
#message(STATUS "LibLAS Include Directory : ${libLAS_INCLUDE_DIRS}")
#message(STATUS "LibLAS Library : ${libLAS_LIBRARIES}")

# OpenCV
find_package(OpenCV REQUIRED HINTS "/opt/ros/kinetic/share/OpenCV-3.3.1-dev")
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBS})
#message(STATUS "OpenCV Dir : ${OpenCV_INCLUDE_DIRS}")
#message(STATUS "OpenCV Lib : ${OpenCV_LIBS}")

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_libraries(${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})
#message(STATUS "PCL Include Dir : ${PCL_INCLUDE_DIRS}")
#message(STATUS "PCL Library Dir : ${PCL_LIBRARY_DIRS}")

OPTION(UseTest "Test Option" ON)

include_directories(${PROJECT_SOURCE_DIR}/3rdParty)
include_directories(${PROJECT_SOURCE_DIR}/src)
add_subdirectory(src)
add_subdirectory(app)
if(UseTest)
   # add_subdirectory(test)
  #  add_subdirectory(test2)
    add_subdirectory(test3)
endif()
