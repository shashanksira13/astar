cmake_minimum_required(VERSION 2.8.3)
project(astar)

######################################
## Find catkin macros and libraries ##
######################################

find_package(catkin REQUIRED COMPONENTS
#	${PROJECT_NAME}_msgs
	roscpp
)

################################
## include common CMake setup ##
################################

find_package(cmake_modules REQUIRED)
find_package(OpenCV REQUIRED )


#######################################
## dynamic_reconfigure configuration ##
#######################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
#	INCLUDE_DIRS include
#	LIBRARIES ...
	CATKIN_DEPENDS
	DEPENDS
		OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(SYSTEM
	${catkin_INCLUDE_DIRS}
)



############################
## Build node(let) AtlasRoadFollower ##
############################


add_executable( showpath src/astar/main.cpp src/astar/astar.h src/astar/astar.cpp )
target_link_libraries( showpath ${catkin_LIBRARIES} )


#############
## Install ##
#############



## Mark executables and/or libraries for installation
install(
	TARGETS
	showpath
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
install(
	DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

