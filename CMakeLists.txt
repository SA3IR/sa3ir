# RoboComp global CMake
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(SAC3IR)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

# Find universal dependencies
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules/" ${CMAKE_MODULE_PATH})


# Update the documentation string of CMAKE_BUILD_TYPE for GUIs
SET(CMAKE_BUILD_TYPE "${CMAKE_BUILD_TYPE}" CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel Maintainer."
    FORCE)

SUBDIRS(
	contextProviderComp
	navigationAgent
	objectRecognitionAgent
	pickUpAgent
	deliverAgent
	personAgent
	ZMQServer
	behaviorTreeAgent
	mira/localNavigationComp
	realsenseComp
	webInterfaceAgent
	mironAgent/
)


