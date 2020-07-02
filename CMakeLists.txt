cmake_minimum_required(VERSION 3.5)
#project(hs_handrobot_visdetect)
project(openni2_tracker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS     
                                        geometry_msgs
                                        std_msgs					
					                    roscpp
					                    roslib
                                        tf
					                    tf2
										OpenCV
										cv_bridge
										image_transport
                                        message_generation)

include_directories(${catkin_INCLUDE_DIRS})
find_package(orocos_kdl REQUIRED)

add_message_files(
	FILES
	TrackerUser.msg
	TrackerUserArray.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs geometry_msgs message_runtime
)
# Find OpenNI2
#find_package(PkgConfig)
#pkg_check_modules(OpenNI2 REQUIRED libopenni2)
find_path(OpenNI2_INCLUDEDIR
	  NAMES OpenNI.h
	  HINTS /usr/include/openni2 /home/fshs/catkin_ws/OpenNI2/Include)
find_library(OpenNI2_LIBRARIES
	     NAMES OpenNI2 DummyDevice OniFile PS1090
	     HINTS /usr/lib/ /usr/lib/OpenNI2/Drivers /home/fshs/catkin_ws/OpenNI2/Bin/x64-Release/OpenNI2/Drivers
	     PATH_SUFFIXES lib) 
message(STATUS ${OpenNI2_LIBRARIES})

# Find Nite2
message(status $ENV{NITE2_INCLUDE})
message(status $ENV{NITE2_REDIST64})
find_path(Nite2_INCLUDEDIR
	  NAMES NiTE.h
	  HINTS /home/fshs/catkin_ws/src/NiTE-Linux-x64-2.2/Include)
find_library(Nite2_LIBRARY
	     NAMES NiTE2
	     HINTS /home/fshs/catkin_ws/src/NiTE-Linux-x64-2.2/Redist
	     PATH_SUFFIXES lib) 

#catkin_package()

include_directories(/opt/ros/kinetic/include ${catkin_INCLUDEDIR}
		    ${OpenNI2_INCLUDEDIR}
		    ${Nite2_INCLUDEDIR})

# add_subdirectory(jsoncpp)

add_executable(openni2_tracker src/openni2_tracker.cpp)
target_link_libraries(openni2_tracker ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY})

add_executable(tracker src/tracker.cpp)
target_link_libraries(tracker ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY})
add_dependencies(tracker openni2_tracker_generate_messages_cpp)

install(TARGETS openni2_tracker RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES openni2_tracker.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(hello src/hello.cpp)
target_link_libraries(hello ${catkin_LIBRARIES} -ljson)

add_executable(vision_detect src/vision_detect.cpp)
target_link_libraries(vision_detect ${catkin_LIBRARIES} ${OpenNI2_LIBRARIES} ${Nite2_LIBRARY})
