cmake_minimum_required(VERSION 2.8.3)
project(pcr_cov_slam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# SET(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
#set(INSTALL_LIB_DIR lib CACHE PATH "/usr/local/include/opengv")
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

FIND_PACKAGE(OpenCV 2 REQUIRED)
LINK_DIRECTORIES( ${LINK_DIRECTORIES} /home/leo/opengv/lib)
find_package(PCL 1.7.2 REQUIRED)
find_package(
	libpointmatcher
	REQUIRED
	)
include_directories(
  ${PCL_INCLUDE_DIRS}
  )
link_directories(
  ${PCL_LIBRARY_DIRS}
  )
add_definitions(${PCL_DEFINITIONS})
#find_package( opengv REQUIRED )


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  /home/leo/opengv/include
)


## Declare a cpp executable
#add_executable(ma_slam_node src/ma_slam_node.cpp src/CentralStorage.cpp src/RobotListener_Keyframes.cpp src/HelperFcts.cpp src/PtClListener.cpp src/PoseListener.cpp src/SyncListener.cpp src/random_generators.cpp src/experiment_helpers.cpp src/time_measurement.cpp)
#add_executable(main main.cpp 3dcov/cbshot.cpp)
add_executable(main main.cpp 3dcov/cbshot.cpp kalman/KalmanFilterScale.cpp )



## Specify libraries to link a library or executable target against
target_link_libraries(main ${OpenCV_LIBRARIES} opengv ${PCL_LIBRARIES} ${libpointmatcher_LIBRARIES} )
#ExtrMatchDescrCl RobotListener_Keyframes 
