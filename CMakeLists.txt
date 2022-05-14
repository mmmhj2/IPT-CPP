cmake_minimum_required(VERSION 3.15)  # CMake version check
project(ipt_cpp)  # Create project "simple_example"
set(CMAKE_CXX_STANDARD 14)  # Enable c++14 standard

add_compile_options(-Wall -Wextra -pedantic)

# Specify build type
if(NOT DEFINED CMAKE_BUILD_TYPE)
	MESSAGE(WARNING "Build type unspecified, default to release")
	set(CMAKE_BUILD_TYPE "Release")
endif()
MESSAGE("Build type:" ${CMAKE_BUILD_TYPE})

# Set up envrionments
if(NOT ("/opt/ros/noetic" IN_LIST CMAKE_PREFIX_PATH))
	MESSAGE("Catkin envrionment is not defined.")
	# Set the location of ROS python packages
	set(ENV{PYTHONPATH} "/opt/ros/noetic/lib/python3/dist-packages")
	# Set the prefix for ROS
	list(APPEND CMAKE_PREFIX_PATH "~/ncatkin_ws/devel")
	list(APPEND CMAKE_PREFIX_PATH "/opt/ros/noetic")
	set(catkin_DIR "/opt/ros/noetic/share/catkin/cmake")
	if("/opt/ros/noetic" IN_LIST CMAKE_PREFIX_PATH)
		MESSAGE("Envrionment setup succeeded")
	endif()
else()
	MESSAGE("Catkin envrionment is already initialized.")
endif()

find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(Threads)

# Configuration for catkin
find_package(catkin REQUIRED COMPONENTS
	roscpp roslib rospy
)
catkin_package(
)

find_package(PkgConfig)
pkg_search_module(apriltag REQUIRED apriltag)
#find_package(apriltag REQUIRED)
#find_package(nlohmann_json REQUIRED)


# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(. DIR_ROOT)
aux_source_directory(./src DIR_SRC)
aux_source_directory(./include DIR_INCLUDE)

aux_source_directory(./ros/include DIR_ROS_INCLUDE)
aux_source_directory(./ros/src DIR_ROS_SRC)

# 指定生成目标，可以指定多个. 有executable和library等等。
set(apriltag_INCLUDE_DIRS "${apriltag_INCLUDE_DIRS}/apriltag")

link_directories(${apriltag_LIBDIR})

include_directories(${apriltag_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/include/nlohmann")
include_directories(${catkin_INCLUDE_DIRS})

include_directories("include")
include_directories("ros/include")

add_executable(IPT_cpp 
	"main/main.cpp" 
	${DIR_SRC} 
	#${DIR_INCLUDE}
)
target_link_libraries(IPT_cpp 
	${apriltag_LIBRARIES} 
	${OpenCV_LIBS} 
	${CMAKE_THREAD_LIBS_INIT}
	nlohmann_json::nlohmann_json
)

#add_executable(record_video record_video.c src/avilib.c src/v4l2uvc.c include/avilib.h include/uvcvideo.h include/v4l2uvc.h)

add_executable(ipt_node 
	"main/main_ros.cpp"
	${DIR_SRC}
	#${DIR_INCLUDE}
	${DIR_ROS_SRC}
	#${DIR_ROS_INCLUDE}
)
target_link_libraries(ipt_node
	${apriltag_LIBRARIES} 
	${OpenCV_LIBS} 
	${CMAKE_THREAD_LIBS_INIT}
	nlohmann_json::nlohmann_json
	${catkin_LIBRARIES}
)

add_executable(ipt_streamer_node
	"main/main_streamer.cpp"
	${DIR_ROS_SRC}
)
target_link_libraries(ipt_streamer_node
	${OpenCV_LIBS}
	${catkin_LIBRARIES}
)

add_executable(ipt_pose_receiver_node
	"main/main_receiver.cpp"
	"ros/src/ipt_ros_base.cpp"
	"ros/src/ipt_ros_pose_receiver.cpp"		# Keep it simple and avoid unnecessary includes such as OpenCV libs
)
target_link_libraries(ipt_pose_receiver_node
	${catkin_LIBRARIES}
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	
	MESSAGE(NOTICE "Generating unit tests")
	add_executable(ipt_estimation_test
		tests/ipt_refactor_test.cpp
		${DIR_SRC}
		#${DIR_INCLUDE}
	)
	target_link_libraries(ipt_estimation_test 
		${apriltag_LIBRARIES} 
		${OpenCV_LIBS} 
		${CMAKE_THREAD_LIBS_INIT}
		nlohmann_json::nlohmann_json
	)

	add_executable(video_capture_test
		tests/video_test.cpp
	)
	target_link_libraries(video_capture_test
		${OpenCV_LIBS}
	)

	add_executable(rotation_convertion_test
		tests/rotation_test.cpp
		src/tools.cpp
	)
	target_link_libraries(rotation_convertion_test
		${OpenCV_LIBS}
	)

	add_executable(streaming_tcp_test
		tests/streaming_tcp_test.cpp
		${DIR_SRC}
	)
	target_link_libraries(streaming_tcp_test
		${apriltag_LIBRARIES} 
		${OpenCV_LIBS}
		${CMAKE_THREAD_LIBS_INIT}
	)
endif()
