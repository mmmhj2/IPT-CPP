cmake_minimum_required(VERSION 3.15)  # CMake version check
project(ipt_msys)
set(CMAKE_CXX_STANDARD 14)  # Enable c++14 standard

add_compile_options(-Wall -Wextra -pedantic)

find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
#find_package(PkgConfig)
#pkg_search_module(apriltag REQUIRED apriltag)

aux_source_directory("../include" DIR_INCLUDE)
aux_source_directory("../ros/include" DIR_ROS_INCLUDE)

include_directories("../include")
include_directories("../ros/include")
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("/usr/include/nlohmann")

add_executable(ipt_windows
	"main_mingw.cpp"
	"../src/ipt_receiver_new.cpp"
	"../src/tools.cpp"
	"../src/socket_data.cpp"
)
target_link_libraries(ipt_windows
	${OpenCV_LIBS}
	nlohmann_json::nlohmann_json
	wsock32
	ws2_32
	"libapriltag.dll.a"
)
