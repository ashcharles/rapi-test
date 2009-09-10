cmake_minimum_required(VERSION 2.6)

INCLUDE(FindPkgConfig)
PKG_SEARCH_MODULE(RAPI REQUIRED autolab-rapi)
PKG_SEARCH_MODULE(RAPICB REQUIRED autolab-rapi-chatterbox)
PKG_SEARCH_MODULE(AUTOLABND REQUIRED autolab-nd)

INCLUDE_DIRECTORIES(
	${RAPI_INCLUDE_DIRS}
	${RAPICB_INCLUDE_DIRS}
)

LINK_DIRECTORIES(
	${RAPI_LIBRARY_DIRS}
	${RAPICB_LIBRARY_DIRS}
	${AUTOLABND_LIBRARY_DIRS}
)

ADD_EXECUTABLE(ash-test
  ash-test.cpp
  chatterboxctrl.cpp
  waypoint.cpp
)

TARGET_LINK_LIBRARIES(ash-test
  autolab-rapi
  autolab-rapi-chatterbox
  autolab-nd
)

ADD_DEFINITIONS(-g -Wall)
