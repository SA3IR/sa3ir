
MIRA_REQUIRE_PACKAGE(MiraNavigation)

###############################################################################

MIRA_ADD_DOCUMENTATION(MiraNavigation
	DIRS
		doc
		include
	DEPENDS
		#RobotDataTypes
)

###############################################################################

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR}/include)

###############################################################################

MIRA_ADD_LIBRARY(MiraNavigation
	SHARED
	PACKAGE MiraNavigation
	SOURCE
		src/MiraNavigation.C
		src/Navigator.cpp
#		src/personFollower.cpp
		src/MiraLaser.cpp
	LINK_LIBS
		MIRABase
		MIRAFramework
		Navigation
		RobotDataTypes
		Ice
)
