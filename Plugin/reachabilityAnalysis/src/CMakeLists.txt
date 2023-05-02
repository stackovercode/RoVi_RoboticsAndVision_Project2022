cmake_minimum_required(VERSION 3.10)

PROJECT(reachabilityAnalysis)
MESSAGE(${PROJECT_NAME} ":")

MESSAGE(${PROJECT_NAME} " Begin")

SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
SET(RW_ROOT ~/RobWork/RobWork)
set(CMAKE_VERBOSE_MAKEFILE false)
SET(CMAKE_BUILD_TYPE Release)

MESSAGE("-- Build types: " ${CMAKE_BUILD_TYPE})
SET(RobWork_DIR ${RW_ROOT}/cmake) 

FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

SET(SrcFiles reachability.cpp reachability.hpp main.cpp)

ADD_EXECUTABLE(reachabilityAnalysis ${SrcFiles})
TARGET_LINK_LIBRARIES(reachabilityAnalysis ${ROBWORK_LIBRARIES})

MESSAGE(${PROJECT_NAME} " completed!")
