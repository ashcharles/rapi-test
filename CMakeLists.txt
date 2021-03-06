cmake_minimum_required(VERSION 2.6)
PROJECT(ash-test)

SET(TARGET ash-test)

SET(CMAKE_BUILD_TYPE debug)
ADD_DEFINITIONS(-g -Wall)

INCLUDE(FindPkgConfig)
PKG_SEARCH_MODULE(RAPI REQUIRED autolab-rapi)
PKG_SEARCH_MODULE(RAPICB REQUIRED autolab-rapi-chatterbox)
PKG_SEARCH_MODULE(TRACKER REQUIRED autolab-rapi-autolabtracker)

PKG_SEARCH_MODULE(AUTOLABND REQUIRED autolab-nd)

MESSAGE(STATUS "tracker dir ${TRACKER_INCLUDE_DIRS}" )	
INCLUDE_DIRECTORIES(
	${RAPI_INCLUDE_DIRS}
	${RAPICB_INCLUDE_DIRS}
    ${TRACKER_INCLUDE_DIRS}
    ${AUTOLABND_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
	${RAPI_LIBRARY_DIRS}
	${RAPICB_LIBRARY_DIRS}
    ${TRACKER_LIBRARY_DIRS}
	${AUTOLABND_LIBRARY_DIRS}
)

ADD_EXECUTABLE(ash-test
  ash-test.cpp
  chatterboxctrl.cpp
  task.cpp
  waypoint.cpp
)


TARGET_LINK_LIBRARIES(ash-test
  autolab-rapi
  autolab-rapi-chatterbox
  autolab-rapi-autolabtracker
)
